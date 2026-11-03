"""The occupancy map: saving and loading, wall-erosion resistance, and the background mapper."""

import asyncio
import json
import math
import os
import threading
import time
from pathlib import Path

from lidar_to_map import gps_to_offset
from occupancy_grid import OCC_THRESHOLD, OccupancyGrid
from plan_from_lidar import scan_to_rays

from .constants import MAP_FILE_VERSION


# ----------------------------------------------------------------------
# Saving and loading the map between runs
# ----------------------------------------------------------------------
def _geometry(grid):
    return {
        "min_north_m": grid.min_north_m,
        "min_east_m": grid.min_east_m,
        "resolution_m": grid.resolution_m,
        "rows": grid.rows,
        "cols": grid.cols,
    }


def _same_geometry(saved, grid):
    now = _geometry(grid)
    try:
        return (
            saved["rows"] == now["rows"]
            and saved["cols"] == now["cols"]
            and all(
                abs(float(saved[k]) - now[k]) < 1e-6
                for k in ("min_north_m", "min_east_m", "resolution_m")
            )
        )
    except (KeyError, TypeError, ValueError):
        return False


def save_grid(grid, path, origin):
    """Write every cell the lidar has seen to `path` (JSON). Returns the count.

    Only seen cells are stored, as [index, log_odds]. The file is written to
    a temporary name and then renamed, so a crash never leaves half a map.
    """
    cells = [[i, round(grid.log_odds[i], 3)] for i, s in enumerate(grid.seen) if s]
    data = {
        "version": MAP_FILE_VERSION,
        "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "origin_lat_lon": [origin[0], origin[1]],
        "grid": _geometry(grid),
        "cells": cells,
    }
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(data, separators=(",", ":")))
    os.replace(tmp, path)
    return len(cells)


def load_grid(grid, path, origin, origin_tol_m, decay):
    """Load a saved map into an empty grid.

    Returns (loaded, message). Nothing is loaded (and the grid is left
    untouched) if the file is unreadable, was made with different grid
    settings, or was made from a different start point: an old map placed on
    the wrong spot would put obstacles where there are none.

    decay (0..1) scales the old log-odds, so fresh scans can overrule old
    ones quickly (1.0 = trust the old map completely).
    """
    path = Path(path)
    try:
        data = json.loads(path.read_text())
        if data.get("version") != MAP_FILE_VERSION:
            return False, f"{path.name} has an unknown format version."
        cells = data["cells"]
        saved_origin = data["origin_lat_lon"]
        saved_grid = data["grid"]
    except (OSError, ValueError, KeyError, AttributeError) as error:
        return False, f"Could not read {path.name} ({error}). Starting with an empty map."

    if not _same_geometry(saved_grid, grid):
        return False, (
            f"{path.name} was made with different grid settings (area or "
            "resolution in mission.yaml changed). Starting with an empty map."
        )

    d_north, d_east = gps_to_offset(
        float(saved_origin[0]), float(saved_origin[1]), origin[0], origin[1]
    )
    shift = math.hypot(d_north, d_east)
    if shift > origin_tol_m:
        return False, (
            f"{path.name} was made from a start point {shift:.1f} m away from "
            f"this one (limit {origin_tol_m:g} m). Starting with an empty map."
        )

    size = grid.rows * grid.cols
    try:
        parsed = [(int(i), float(v)) for i, v in cells]
    except (TypeError, ValueError):
        return False, f"{path.name} contains bad cell data. Starting with an empty map."
    if any(not 0 <= i < size for i, _ in parsed):
        return False, f"{path.name} contains out-of-range cells. Starting with an empty map."

    for i, v in parsed:
        grid.log_odds[i] = v * decay
        grid.seen[i] = True
    # The grid caches its inflated obstacle set; make it recompute.
    grid._version += 1
    grid._inflated_cache.clear()

    occupied = sum(1 for v in grid.log_odds if v > OCC_THRESHOLD)
    when = data.get("saved_at", "unknown time")
    return True, (
        f"Loaded map from {path.name} (saved {when}): {len(parsed)} known "
        f"cells, {occupied} occupied."
    )


def persist_map(grid, args, origin, lock=None):
    """Save the map if saving is on. Never raises (a failed save must not
    hide the flight result or stop the landing)."""
    if not args.save_map or origin is None:
        return False
    try:
        if lock is not None:
            with lock:
                count = save_grid(grid, args.map_file, origin)
        else:
            count = save_grid(grid, args.map_file, origin)
    except Exception as error:
        print(f"(Could not save the map to {args.map_file}: {error})")
        return False
    print(f"Saved map ({count} cells) to {args.map_file}")
    return True


class ResistantGrid(OccupancyGrid):
    """OccupancyGrid where cells already seen as occupied are harder to
    erase.

    Why: a beam that runs almost parallel to a wall (for example looking
    down an aisle from its end) passes through the wall's edge cells before
    it ends, and marks them free. That punched holes in the walls and left
    obstacles only 0.25 m from a planned path. Free evidence against an
    occupied cell now counts `occupied_resistance` times as much (1.0 gives
    the original behaviour). Real obstacles that are gone still clear, just
    a little slower.
    """

    def __init__(self, *args, occupied_resistance=0.5, **kwargs):
        super().__init__(*args, **kwargs)
        self.occupied_resistance = occupied_resistance

    def _add(self, row, col, delta):
        if delta < 0 and self.log_odds[self._idx(row, col)] > OCC_THRESHOLD:
            delta *= self.occupied_resistance
        super()._add(row, col, delta)


# ----------------------------------------------------------------------
# Map + planner, shared between threads
# ----------------------------------------------------------------------
class MapService:
    """The occupancy grid and planner behind one lock.

    Updating the grid and planning both take tens of milliseconds, so they
    run in worker threads; the lock keeps them from touching the grid at the
    same time.
    """

    def __init__(self, planner, grid):
        self.planner = planner
        self.grid = grid
        self.lock = threading.Lock()
        # Which heavy jobs are running right now (read by the flight log).
        self.busy = {"map": False, "plan": False, "frontier": False}

    def update(self, north, east, rays):
        with self.lock:
            self.busy["map"] = True
            try:
                self.grid.update(north, east, rays)
            finally:
                self.busy["map"] = False

    def _path_ok(self, pts):
        p = self.planner
        if len(pts) < 2 or not p.is_point_free(pts[-1]):
            return False
        return all(p.is_segment_free(a, b) for a, b in zip(pts, pts[1:]))

    def plan_job(self, start, target, old_path, force):
        """Returns (kind, route, seconds); kind is 'keep', 'new' or
        'target-blocked'. 'keep' means the old path is still collision-free."""
        t0 = time.monotonic()
        with self.lock:
            if not self.planner.is_point_free(target):
                return "target-blocked", None, time.monotonic() - t0
            if old_path is not None and not force and self._path_ok(old_path):
                return "keep", None, time.monotonic() - t0
            self.busy["plan"] = True
            try:
                route = self.planner.plan(start, target, allow_blocked_start=True)
            finally:
                self.busy["plan"] = False
        return "new", route, time.monotonic() - t0


# ----------------------------------------------------------------------
# Mapper: scans -> grid, in the background
# ----------------------------------------------------------------------
class Mapper:
    def __init__(self, stream, tracker, svc, stats, args, max_tilt):
        self.stream = stream
        self.tracker = tracker
        self.svc = svc
        self.stats = stats
        self.args = args
        self.max_tilt = max_tilt
        self.error = None
        self._task = None

    def start(self):
        self._task = asyncio.create_task(self._guard())

    async def stop(self):
        if self._task is not None:
            self._task.cancel()
            await asyncio.gather(self._task, return_exceptions=True)
            self._task = None

    def check(self):
        if self.error is not None:
            raise RuntimeError(f"Mapping failed: {self.error!r}")

    async def _guard(self):
        try:
            await self._run()
        except asyncio.CancelledError:
            raise
        except Exception as error:
            self.error = error

    async def _run(self):
        args = self.args
        min_gap = 1.0 / args.map_rate
        last_seq = 0
        last_used = 0.0

        while True:
            await asyncio.sleep(0.02)
            if self.stream.error is not None:
                raise self.stream.error

            item, seq = self.stream.latest, self.stream.seq
            if item is None or seq == last_seq:
                continue
            last_seq = seq

            if time.monotonic() - last_used < min_gap:
                self.stats.skipped_rate += 1
                continue

            t_scan, scan = item
            pose = self.tracker.pose_at(t_scan - args.scan_delay)
            if pose is None:
                self.stats.skipped_pose += 1
                continue

            north, east, heading, tilt = pose
            if tilt > self.max_tilt:
                self.stats.skipped_tilt += 1
                continue

            rays = scan_to_rays(scan, north, east, heading)[:: args.ray_stride]
            await asyncio.to_thread(self.svc.update, north, east, rays)
            last_used = time.monotonic()
            self.stats.integrated += 1
