"""Stage 4 (fast version): fly to targets using ONLY what the lidar sees,
without stopping to scan.

How it works now (compared with the first stop-and-go version):

  * LidarStream keeps ONE `gz topic -e` process running and always holds the
    newest scan, so reading a scan costs nothing (no new process per scan).
  * PoseTracker remembers the last ~20 s of poses. Every scan is placed on
    the map using the drone's pose AT THE MOMENT OF THE SCAN, so the drone
    can keep flying while it maps.
  * Mapper (background task) adds scans to the occupancy grid ~5 times a
    second. Scans taken while the drone is tilted are skipped: a tilted 2D
    lidar sees the floor as a fake wall.
  * A planner job runs about every 0.3 s. It keeps the current path while it
    is still collision-free (no jitter) and only replans when it is not.
  * The controller (10 Hz) follows the path with a "carrot" setpoint a few
    metres ahead, as far as the lidar has CONFIRMED the space to be free.
    In open space it flies at full speed; where the map is still unknown
    (behind a corner) it slows down and waits for the map to fill in.
  * Yaw is changed while flying; the drone only stops to turn for big turns.
  * Between targets it does not stop: it switches to the next target when
    within --corner-radius of the current one.
  * A timing summary at the end shows where the time went.

Also provides the helpers yaw_test.py imports:
    PoseTracker, angle_diff, arm_and_takeoff, land_and_wait, wait_for_health

Run with the simulator up (gz_x500_lidar_2d) and no other MAVSDK script:
    python3 scripts/online_navigation.py --target 8,8
    python3 scripts/online_navigation.py            # targets from mission.yaml
"""

import argparse
import asyncio
import collections
import math
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path

from mavsdk_grpc.offboard import PositionNedYaw

from lidar_scan import LidarError, find_scan_topic, parse_scan
from lidar_to_map import connect, gps_to_offset, read_origin
from occupancy_grid import OccupancyGrid, render_ascii
from path_planning import Point2D
from plan_from_lidar import parse_target, scan_to_rays
from plan_route import (
    DEFAULT_CONFIG,
    ConfigError,
    build_planner,
    load_point,
    load_yaml,
)

SETPOINT_RATE_HZ = 10.0
CONTROL_PERIOD_S = 0.1


# ----------------------------------------------------------------------
# Small helpers
# ----------------------------------------------------------------------
def angle_diff(a, b):
    """Signed smallest difference a - b in degrees, in [-180, 180).

    Handles wrap-around: angle_diff(350, 10) == -20, angle_diff(10, 350) == 20.
    """
    return (a - b + 180.0) % 360.0 - 180.0


def _wrap_180(deg):
    return (deg + 180.0) % 360.0 - 180.0


async def wait_for_health(drone, timeout):
    """Wait until PX4 has a valid global position and Home."""
    print("Waiting for valid position and Home...")

    async def check():
        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                return

    try:
        await asyncio.wait_for(check(), timeout)
    except asyncio.TimeoutError:
        raise RuntimeError(
            f"PX4 did not report a valid position and Home within {timeout:g} s."
        ) from None
    print("Position and Home are valid.")


async def set_telemetry_rates(drone, hz=20.0):
    """Ask PX4 for faster telemetry (better pose for fast flight). Optional."""
    for name in (
        "set_rate_position_velocity_ned",
        "set_rate_attitude_euler",
        "set_rate_position",
    ):
        setter = getattr(drone.telemetry, name, None)
        if setter is None:
            continue
        try:
            await setter(hz)
        except Exception as error:  # not fatal: PX4 defaults still work
            print(f"(Could not set {name[9:]} rate: {error})")


async def arm_and_takeoff(drone, altitude):
    """Send the arm and takeoff commands (does not wait for the climb)."""
    await drone.action.set_takeoff_altitude(float(altitude))
    await drone.action.arm()
    await drone.action.takeoff()


async def land_and_wait(drone, timeout=60.0):
    """Land and wait until PX4 reports the vehicle disarmed."""
    await drone.action.land()

    async def check():
        async for armed in drone.telemetry.armed():
            if not armed:
                return

    try:
        await asyncio.wait_for(check(), timeout)
    except asyncio.TimeoutError:
        raise RuntimeError(
            "Landing was not confirmed (still armed). Inspect PX4/Gazebo."
        ) from None
    print("Landed and disarmed.")


# ----------------------------------------------------------------------
# Pose tracking (with history, so scans can be placed accurately)
# ----------------------------------------------------------------------
class PoseTracker:
    """Keeps the latest position, heading and speed from PX4 in the background.

    Map frame = north/east metres from `origin` (PX4 home), the same frame as
    mission.yaml and the planner. PX4's local NED frame is normally identical
    at start-up; any small difference is measured once in wait_ready() and
    corrected, so setpoints and the map always agree.

    Attributes: heading (deg, 0 = north, 90 = east), speed (m/s, horizontal).
    pose_at(t) gives the interpolated pose at an earlier time.monotonic().
    """

    def __init__(self, drone, origin):
        self.drone = drone
        self.origin = origin
        self.heading = None
        self.roll = None
        self.pitch = None
        self.has_tilt = False
        self.speed = 0.0
        self.error = None
        self.last_update = None
        self.offset_north = 0.0
        self.offset_east = 0.0
        self._ned = None  # raw local NED (north, east, down)
        self._gps = None
        self._tasks = []
        # (time, raw north, raw east, heading, tilt) - newest last
        self.history = collections.deque(maxlen=2000)

    # ---- lifecycle ---------------------------------------------------
    def start(self):
        self._tasks = [
            asyncio.create_task(self._guard(self._ned_loop())),
            asyncio.create_task(self._guard(self._gps_loop())),
            asyncio.create_task(self._guard(self._heading_loop())),
        ]

    async def stop(self):
        for task in self._tasks:
            task.cancel()
        await asyncio.gather(*self._tasks, return_exceptions=True)
        self._tasks = []

    async def wait_ready(self, timeout):
        deadline = time.monotonic() + timeout
        while self._ned is None or self._gps is None or self.heading is None:
            if self.error is not None:
                raise RuntimeError(f"Telemetry failed: {self.error!r}")
            if time.monotonic() > deadline:
                raise RuntimeError("Timed out waiting for position and heading.")
            await asyncio.sleep(0.05)

        # Measure how the map frame (GPS offset from origin) differs from
        # PX4's local NED frame, once, while still on the ground.
        gps_n, gps_e = gps_to_offset(
            self._gps[0], self._gps[1], self.origin[0], self.origin[1]
        )
        self.offset_north = gps_n - self._ned[0]
        self.offset_east = gps_e - self._ned[1]
        if math.hypot(self.offset_north, self.offset_east) > 0.5:
            print(
                f"Note: map frame is {self.offset_north:.2f} N / "
                f"{self.offset_east:.2f} E away from PX4's local frame "
                "(corrected)."
            )
        if not self.has_tilt:
            print("Note: no attitude stream, so tilted lidar scans cannot "
                  "be filtered out.")

    # ---- readings ----------------------------------------------------
    @property
    def north(self):
        return self._ned[0] + self.offset_north

    @property
    def east(self):
        return self._ned[1] + self.offset_east

    @property
    def down(self):
        return self._ned[2]

    def ned(self):
        return self._ned

    def pose(self):
        """(north_m, east_m, heading_deg) in the map frame."""
        return self.north, self.east, self.heading

    def map_to_ned(self, north, east):
        return north - self.offset_north, east - self.offset_east

    def pose_at(self, t):
        """(north, east, heading, tilt_deg) at time t, or None if unknown.

        Interpolated between the two nearest telemetry samples.
        """
        samples = list(self.history)
        if not samples or t < samples[0][0]:
            return None

        if t >= samples[-1][0]:
            _, n, e, h, tilt = samples[-1]
        else:
            for i in range(len(samples) - 1, 0, -1):
                a, b = samples[i - 1], samples[i]
                if a[0] <= t <= b[0]:
                    span = b[0] - a[0]
                    f = (t - a[0]) / span if span > 1e-6 else 1.0
                    n = a[1] + f * (b[1] - a[1])
                    e = a[2] + f * (b[2] - a[2])
                    h = (a[3] + f * angle_diff(b[3], a[3])) % 360.0
                    tilt = max(a[4], b[4])
                    break
            else:
                return None
        return n + self.offset_north, e + self.offset_east, h, tilt

    def check(self):
        """Raise if telemetry died or stopped updating."""
        if self.error is not None:
            raise RuntimeError(f"Telemetry failed: {self.error!r}")
        if self.last_update is None or time.monotonic() - self.last_update > 2.0:
            raise RuntimeError("Position telemetry stopped updating.")

    # ---- background loops --------------------------------------------
    async def _guard(self, coro):
        try:
            await coro
        except asyncio.CancelledError:
            raise
        except Exception as error:  # remembered, raised by check()
            self.error = error

    def _record(self):
        if self._ned is None or self.heading is None:
            return
        if self.roll is not None:
            tilt = max(abs(self.roll), abs(self.pitch))
        else:
            tilt = 0.0
        self.history.append(
            (time.monotonic(), self._ned[0], self._ned[1], self.heading, tilt)
        )

    async def _ned_loop(self):
        async for pv in self.drone.telemetry.position_velocity_ned():
            self._ned = (
                pv.position.north_m,
                pv.position.east_m,
                pv.position.down_m,
            )
            self.speed = math.hypot(pv.velocity.north_m_s, pv.velocity.east_m_s)
            self.last_update = time.monotonic()
            self._record()

    async def _gps_loop(self):
        async for position in self.drone.telemetry.position():
            if math.isfinite(position.latitude_deg) and math.isfinite(
                position.longitude_deg
            ):
                self._gps = (position.latitude_deg, position.longitude_deg)

    async def _heading_loop(self):
        # Prefer the attitude stream: higher rate, and it gives roll/pitch.
        try:
            stream = self.drone.telemetry.attitude_euler()
        except AttributeError:
            stream = None

        if stream is not None:
            self.has_tilt = True
            async for euler in stream:
                if math.isfinite(euler.yaw_deg):
                    self.roll = euler.roll_deg
                    self.pitch = euler.pitch_deg
                    self.heading = euler.yaw_deg % 360.0
                    self._record()
        else:
            async for heading in self.drone.telemetry.heading():
                if math.isfinite(heading.heading_deg):
                    self.heading = heading.heading_deg % 360.0
                    self._record()


# ----------------------------------------------------------------------
# Lidar stream (one long-running `gz topic -e`, newest scan always ready)
# ----------------------------------------------------------------------
class LidarStream:
    def __init__(self, topic):
        self.topic = topic
        self.proc = None
        self.thread = None
        self.latest = None  # (arrival time.monotonic(), parsed scan)
        self.seq = 0
        self.bad = 0
        self.error = None
        self._stopping = False

    def start(self):
        if shutil.which("gz") is None:
            raise LidarError(
                "Could not find the 'gz' command. Is Gazebo installed, and is "
                "this terminal set up like the one running the simulator?"
            )
        cmd = ["gz", "topic", "-e", "-t", self.topic]
        if shutil.which("stdbuf"):  # ask for line-by-line output on a pipe
            cmd = ["stdbuf", "-oL"] + cmd
        self.proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _publish(self, lines):
        try:
            scan = parse_scan("\n".join(lines))
        except LidarError:
            self.bad += 1
            return
        self.latest = (time.monotonic(), scan)
        self.seq += 1

    def _reader(self):
        """Split the endless text output into one message per scan."""
        buf = []
        has_ranges = False
        try:
            for raw in self.proc.stdout:
                line = raw.rstrip("\n")
                blank = not line.strip()
                if has_ranges:
                    key = line.split(":", 1)[0].split("{", 1)[0].strip()
                    new_message = blank or (
                        not line[0].isspace()
                        and key not in ("ranges", "intensities")
                    )
                    if new_message:
                        self._publish(buf)
                        buf, has_ranges = [], False
                if blank:
                    continue
                buf.append(line)
                if line.lstrip().startswith("ranges:"):
                    has_ranges = True
            if has_ranges:
                self._publish(buf)
        except Exception as error:
            if not self._stopping:
                self.error = error
            return

        if not self._stopping:
            try:
                detail = self.proc.stderr.read().strip()
            except Exception:
                detail = ""
            self.error = LidarError(
                f"'gz topic -e -t {self.topic}' stopped unexpectedly. {detail}"
            )

    async def wait_first(self, timeout):
        deadline = time.monotonic() + timeout
        while self.seq == 0:
            if self.error is not None:
                raise self.error
            if time.monotonic() > deadline:
                raise LidarError(
                    f"No lidar scan arrived from {self.topic} within "
                    f"{timeout:g} s. Check what `gz topic -e -t {self.topic}` "
                    "prints in a terminal."
                )
            await asyncio.sleep(0.1)

    def stop(self):
        self._stopping = True
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.proc.kill()


# ----------------------------------------------------------------------
# Offboard control
# ----------------------------------------------------------------------
class OffboardController:
    """Streams the current position+yaw setpoint to PX4 at 10 Hz.

    Callers only change the target; the stream never pauses, even while the
    program is planning (PX4 drops Offboard if it stops).
    """

    def __init__(self, drone):
        self.drone = drone
        self.down = 0.0
        self.error = None
        self._target = None  # (north_ned, east_ned, down_ned, yaw_deg)
        self._task = None
        self._active = False

    async def _send(self):
        n, e, d, yaw = self._target
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(n, e, d, float(_wrap_180(yaw)))
        )

    async def start(self, tracker, down=None, warmup=10):
        """down: NED 'down' value to hold (default: where the drone is now)."""
        n, e, d = tracker.ned()
        self.down = d if down is None else down
        self._target = (n, e, self.down, tracker.heading)

        # PX4 requires a setpoint stream before Offboard can start.
        for _ in range(warmup):
            await self._send()
            await asyncio.sleep(1.0 / SETPOINT_RATE_HZ)

        await self.drone.offboard.start()
        self._active = True
        self._task = asyncio.create_task(self._run())
        print("Offboard started.")

    async def _run(self):
        try:
            while True:
                await self._send()
                await asyncio.sleep(1.0 / SETPOINT_RATE_HZ)
        except asyncio.CancelledError:
            raise
        except Exception as error:
            self.error = error

    async def stop(self):
        if self._task is not None:
            self._task.cancel()
            await asyncio.gather(self._task, return_exceptions=True)
            self._task = None
        if self._active:
            self._active = False
            try:
                await self.drone.offboard.stop()
                print("Offboard stopped.")
            except Exception as error:
                print(f"(Offboard stop failed: {error})")

    def check(self):
        if self.error is not None:
            raise RuntimeError(f"Offboard setpoint stream failed: {self.error!r}")

    def set_target_map(self, tracker, north, east, yaw):
        """Fly towards a map-frame (north, east) at the hold altitude."""
        n, e = tracker.map_to_ned(north, east)
        self._target = (n, e, self.down, yaw)

    def hold(self, tracker, yaw=None):
        """Stay where the drone is (facing `yaw`, default: as it is now)."""
        n, e, _ = tracker.ned()
        self._target = (n, e, self.down, tracker.heading if yaw is None else yaw)


# ----------------------------------------------------------------------
# Path geometry
# ----------------------------------------------------------------------
def path_length(pts):
    return sum(
        math.hypot(b.north_m - a.north_m, b.east_m - a.east_m)
        for a, b in zip(pts, pts[1:])
    )


def trim_path(path, north, east):
    """Path from the point on it closest to (north, east) to its end."""
    if len(path) < 2:
        return list(path)

    best_d2, best_i, best_p = math.inf, 0, path[0]
    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        dn, de = b.north_m - a.north_m, b.east_m - a.east_m
        seg2 = dn * dn + de * de
        if seg2 < 1e-12:
            t = 0.0
        else:
            t = ((north - a.north_m) * dn + (east - a.east_m) * de) / seg2
            t = max(0.0, min(1.0, t))
        pn, pe = a.north_m + t * dn, a.east_m + t * de
        d2 = (north - pn) ** 2 + (east - pe) ** 2
        if d2 < best_d2:
            best_d2, best_i, best_p = d2, i, Point2D(pn, pe)
    return [best_p] + list(path[best_i + 1:])


def point_along(pts, distance):
    """The point `distance` metres along the polyline (clamped to its end)."""
    for a, b in zip(pts, pts[1:]):
        seg = math.hypot(b.north_m - a.north_m, b.east_m - a.east_m)
        if seg > 1e-9 and distance <= seg:
            f = distance / seg
            return Point2D(
                a.north_m + f * (b.north_m - a.north_m),
                a.east_m + f * (b.east_m - a.east_m),
            )
        distance -= seg
    return pts[-1]


def cell_seen(grid, north, east):
    """True if the lidar has looked at this spot (free or occupied)."""
    row, col = grid.to_cell(north, east)
    return grid.in_grid(row, col) and grid.seen[row * grid.cols + col]


def free_extent(grid, pts, max_distance):
    """How far along the path the map is confirmed (seen) before a gap.

    A gap is two samples in a row (about 1 m) on cells the lidar never
    looked at. Returns the distance to the start of the gap, or math.inf if
    there is none within max_distance. A single unseen cell is ignored: the
    lidar's thin rays can leave tiny holes far away.
    """
    step = grid.resolution_m
    walked = 0.0
    next_sample = step
    run = 0
    first = None

    for a, b in zip(pts, pts[1:]):
        seg = math.hypot(b.north_m - a.north_m, b.east_m - a.east_m)
        if seg < 1e-9:
            continue
        while next_sample <= walked + seg:
            if next_sample > max_distance:
                return math.inf
            f = (next_sample - walked) / seg
            n = a.north_m + f * (b.north_m - a.north_m)
            e = a.east_m + f * (b.east_m - a.east_m)
            if cell_seen(grid, n, e):
                run, first = 0, None
            else:
                run += 1
                if first is None:
                    first = next_sample
                if run >= 2:
                    return first
            next_sample += step
        walked += seg
    return math.inf


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

    def update(self, north, east, rays):
        with self.lock:
            self.grid.update(north, east, rays)

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
            route = self.planner.plan(start, target, allow_blocked_start=True)
        return "new", route, time.monotonic() - t0


class Stats:
    def __init__(self):
        self.t0 = time.monotonic()
        self.state_time = collections.defaultdict(float)
        self.plan_times = []
        self.plans_kept = 0
        self.integrated = 0
        self.skipped_rate = 0
        self.skipped_tilt = 0
        self.skipped_pose = 0

    def add_plan(self, kind, seconds):
        if kind == "keep":
            self.plans_kept += 1
        elif kind == "new":
            self.plan_times.append(seconds)

    def report(self, stream, flown):
        total = max(1e-6, time.monotonic() - self.t0)
        print("\n--- Timing summary ---")
        print(
            f"Navigation time {total:.1f} s, distance flown {flown:.1f} m, "
            f"average speed {flown / total:.2f} m/s"
        )
        print("Where the time went:")
        for state, secs in sorted(self.state_time.items(), key=lambda kv: -kv[1]):
            print(f"  {state:<9} {secs:6.1f} s  ({100 * secs / total:.0f}%)")
        print(
            f"Lidar: {stream.seq} scans received ({stream.seq / total:.1f}/s), "
            f"{self.integrated} added to the map; skipped: "
            f"{self.skipped_rate} (rate limit), {self.skipped_tilt} (drone "
            f"tilted), {self.skipped_pose} (no pose)"
        )
        if self.plan_times:
            mean = 1000 * sum(self.plan_times) / len(self.plan_times)
            worst = 1000 * max(self.plan_times)
            print(
                f"Planner: {len(self.plan_times)} replans (mean {mean:.0f} ms, "
                f"worst {worst:.0f} ms), path kept {self.plans_kept} times"
            )


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


# ----------------------------------------------------------------------
# Navigator: plan in the background, follow the path continuously
# ----------------------------------------------------------------------
async def wait_still(tracker, timeout=4.0, speed_tol=0.2, hold_s=0.5):
    still_since = None
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        tracker.check()
        if tracker.speed < speed_tol:
            if still_since is None:
                still_since = time.monotonic()
            if time.monotonic() - still_since >= hold_s:
                return True
        else:
            still_since = None
        await asyncio.sleep(0.1)
    return False


class Navigator:
    def __init__(self, tracker, ctrl, svc, mapper, stream, stats, args):
        self.tracker = tracker
        self.ctrl = ctrl
        self.svc = svc
        self.grid = svc.grid
        self.mapper = mapper
        self.stream = stream
        self.stats = stats
        self.args = args
        self.yaw_target = tracker.heading
        self.flown = 0.0
        self.trail = [tracker.pose()[:2]]
        self._last_pos = tracker.pose()[:2]

    def _check(self):
        self.ctrl.check()
        self.tracker.check()
        self.mapper.check()

    def _account(self, north, east):
        self.flown += math.hypot(north - self._last_pos[0], east - self._last_pos[1])
        self._last_pos = (north, east)
        if math.hypot(north - self.trail[-1][0], east - self.trail[-1][1]) > 0.5:
            self.trail.append((north, east))

    async def fly_target(self, index, count, target, final):
        """Returns 'reached', 'skipped' (target unusable) or 'failed'."""
        args, tracker, ctrl = self.args, self.tracker, self.ctrl
        reach = args.tolerance if final else args.corner_radius
        lookahead = max(0.5, args.speed)

        t_start = time.monotonic()
        last_tick = t_start
        last_status = 0.0
        last_plan_launch = -1e9
        best_dist, best_t = math.inf, t_start

        path = None
        path_planned = 0.0
        plan_task = None
        plan_start = None
        no_path_since = None
        wait_map_since = None
        turning = False

        try:
            while True:
                await asyncio.sleep(CONTROL_PERIOD_S)
                now = time.monotonic()
                dt, last_tick = now - last_tick, now
                self._check()

                north, east, heading = tracker.pose()
                self._account(north, east)
                dist = math.hypot(target.north_m - north, target.east_m - east)

                if dist <= reach:
                    return "reached"
                if now - t_start > args.target_timeout:
                    print(f"Target {index}: timed out after {args.target_timeout:g} s.")
                    return "failed"
                if dist < best_dist - 0.5:
                    best_dist, best_t = dist, now
                elif now - best_t > args.stall_timeout:
                    print(f"Target {index}: no progress for {args.stall_timeout:g} s.")
                    return "failed"

                # ---- collect a finished planner job -------------------
                if plan_task is not None and plan_task.done():
                    kind, route, seconds = plan_task.result()
                    plan_task = None
                    self.stats.add_plan(kind, seconds)
                    if kind == "target-blocked":
                        print(
                            f"Target {index} is now inside the safety margin "
                            "of a detected obstacle (or outside the flight "
                            "area). Skipping it."
                        )
                        return "skipped"
                    if kind == "new":
                        if route is None:
                            path = None
                            if no_path_since is None:
                                no_path_since = now
                        else:
                            path = [plan_start] + list(route)
                            path_planned = now
                            no_path_since = None

                # ---- start a new planner job --------------------------
                if plan_task is None and now - last_plan_launch >= args.replan_period:
                    old = None if path is None else trim_path(path, north, east)
                    force = path is not None and (
                        now - path_planned > args.reoptimize_period
                    )
                    plan_start = Point2D(north, east)
                    plan_task = asyncio.create_task(
                        asyncio.to_thread(
                            self.svc.plan_job, plan_start, target, old, force
                        )
                    )
                    last_plan_launch = now

                # ---- decide the setpoint ------------------------------
                state = "cruise"
                horizon = math.inf

                if path is None:
                    if no_path_since is not None:
                        state = "no-path"
                        ctrl.hold(tracker, self.yaw_target)
                        if now - no_path_since > args.no_path_timeout:
                            print(f"Target {index}: no safe path with the "
                                  "current map.")
                            return "failed"
                    else:
                        # First plan not ready yet: keep the last setpoint
                        # (no braking at corners).
                        state = "planning"
                else:
                    pts = trim_path(path, north, east)
                    remaining = path_length(pts)
                    carrot_d = min(lookahead, remaining)

                    # Only run ahead as far as the lidar has confirmed free.
                    horizon = free_extent(
                        self.grid, pts, carrot_d + args.stop_margin
                    )
                    if horizon < carrot_d + args.stop_margin:
                        carrot_d = max(0.0, horizon - args.stop_margin)

                    if carrot_d < 0.3:
                        state = "wait-map"
                        if wait_map_since is None:
                            wait_map_since = now
                        if now - wait_map_since > 3.0:
                            # The map is not filling in: creep forward slowly.
                            state = "creep"
                            carrot_d = min(0.6, remaining)
                        else:
                            carrot_d = 0.0
                    else:
                        wait_map_since = None

                    carrot = point_along(pts, carrot_d)

                    if not args.hold_yaw:
                        to_carrot = math.hypot(
                            carrot.north_m - north, carrot.east_m - east
                        )
                        if to_carrot > 0.5:
                            bearing = math.degrees(
                                math.atan2(
                                    carrot.east_m - east, carrot.north_m - north
                                )
                            ) % 360.0
                            if abs(angle_diff(bearing, self.yaw_target)) > 10.0:
                                self.yaw_target = bearing

                        err = abs(angle_diff(self.yaw_target, heading))
                        if turning and err < 35.0:
                            turning = False
                        elif not turning and err > args.turn_in_place:
                            turning = True
                        if turning:
                            state = "turning"
                            carrot = Point2D(north, east)

                    ctrl.set_target_map(
                        tracker, carrot.north_m, carrot.east_m, self.yaw_target
                    )

                self.stats.state_time[state] += dt

                if now - last_status >= 2.0:
                    last_status = now
                    extent = "all clear" if math.isinf(horizon) else f"{horizon:.1f} m"
                    print(
                        f"  t={now - self.stats.t0:5.1f}s  "
                        f"pos=({north:5.1f}, {east:5.1f})  {dist:5.1f} m to go  "
                        f"speed {tracker.speed:.1f} m/s  {state}  "
                        f"(known free ahead: {extent})"
                    )
        finally:
            if plan_task is not None and not plan_task.done():
                plan_task.cancel()

    async def run(self, targets):
        """Visit every target. Returns True unless something failed."""
        self.stats.t0 = time.monotonic()
        skipped = 0

        for i, target in enumerate(targets, start=1):
            final = i == len(targets)
            print(
                f"\n=== Target {i}/{len(targets)}: "
                f"north={target.north_m:.1f}, east={target.east_m:.1f} ==="
            )
            t0 = time.monotonic()
            result = await self.fly_target(i, len(targets), target, final)
            print(f"Target {i}: {result} ({time.monotonic() - t0:.1f} s)")

            if result == "failed":
                return False
            if result == "skipped":
                skipped += 1
            elif final:
                # Come to rest on the last target (corners are flown through).
                self.ctrl.set_target_map(
                    self.tracker, target.north_m, target.east_m, self.yaw_target
                )
                await wait_still(self.tracker)

            if self.args.show_map:
                cfg = self.svc.planner.config
                north, east, _ = self.tracker.pose()
                print(render_ascii(self.grid, cfg.safety_margin_m, None,
                                   (north, east),
                                   (target.north_m, target.east_m)))

        if skipped:
            print(f"\n{skipped} target(s) were skipped as unusable.")
        return True


# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------
async def wait_for_altitude(tracker, altitude, timeout=30.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        tracker.check()
        if -tracker.down >= altitude * 0.85:
            return
        await asyncio.sleep(0.1)
    raise TimeoutError(f"Did not reach {altitude:.1f} m within {timeout:g} s.")


async def wait_until(condition, timeout, what):
    deadline = time.monotonic() + timeout
    while not condition():
        if time.monotonic() > deadline:
            raise RuntimeError(f"Timed out waiting for {what}.")
        await asyncio.sleep(0.05)


async def run(args, planner, grid, targets):
    cfg = planner.config
    drone = await connect(args.address, args.timeout)
    tracker = ctrl = stream = mapper = None
    airborne = False

    try:
        await wait_for_health(drone, 60)
        await set_telemetry_rates(drone)

        if args.origin:
            try:
                origin = tuple(float(v) for v in args.origin.split(","))
                if len(origin) != 2:
                    raise ValueError
            except ValueError:
                raise LidarError("--origin must look like LAT,LON") from None
        else:
            try:
                origin = await read_origin(drone, args.timeout)
            except AttributeError:
                raise LidarError(
                    "This MAVSDK version has no home() telemetry. "
                    "Pass the start point with --origin LAT,LON."
                ) from None

        topic = args.topic or await asyncio.to_thread(find_scan_topic, args.timeout)
        print(f"Lidar topic: {topic}")
        stream = LidarStream(topic)
        stream.start()
        await stream.wait_first(args.timeout)

        tracker = PoseTracker(drone, origin)
        tracker.start()
        await tracker.wait_ready(args.timeout)
        ground_down = tracker.down

        print(f"Taking off to {args.altitude:.1f} m...")
        await arm_and_takeoff(drone, args.altitude)
        airborne = True
        await wait_for_altitude(tracker, args.altitude)

        ctrl = OffboardController(drone)
        await ctrl.start(tracker, down=ground_down - args.altitude)

        # A level 2D lidar at this height never sees the floor within range;
        # tilted past this angle it would, and report fake walls.
        max_tilt = args.max_tilt
        if max_tilt is None:
            max_tilt = 0.8 * math.degrees(
                math.atan2(args.altitude, args.max_range)
            )
        print(f"Lidar scans are skipped while tilted more than {max_tilt:.1f} deg.")

        stats = Stats()
        svc = MapService(planner, grid)
        mapper = Mapper(stream, tracker, svc, stats, args, max_tilt)
        mapper.start()
        await wait_until(lambda: stats.integrated >= 1 or mapper.error,
                         8.0, "the first lidar scan to reach the map")
        mapper.check()

        nav = Navigator(tracker, ctrl, svc, mapper, stream, stats, args)
        success = await nav.run(targets)
        print("\nNavigation finished." if success else "\nNavigation FAILED.")
        stats.report(stream, nav.flown)

        await mapper.stop()
        mapper = None

        final = tracker.pose()
        print("\n# = seen obstacle, + = safety margin, . = seen free, "
              "blank = unknown")
        print("D = drone, T = last target, * = path flown\n")
        print(render_ascii(grid, cfg.safety_margin_m, nav.trail,
                           (final[0], final[1]),
                           (targets[-1].north_m, targets[-1].east_m)))

        print("\nLanding...")
        await ctrl.stop()
        await land_and_wait(drone)
        airborne = False

    finally:
        if airborne:
            print("\nCleanup: landing...")
            try:
                if ctrl is not None:
                    await ctrl.stop()
                await land_and_wait(drone)
            except Exception as error:
                print(
                    f"CRITICAL: could not confirm landing: {error}\n"
                    "Inspect the vehicle in Gazebo/PX4 before continuing."
                )
        if mapper is not None:
            await mapper.stop()
        if stream is not None:
            stream.stop()
        if tracker is not None:
            await tracker.stop()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--target", action="append",
                        help="NORTH,EAST in metres; repeat for several. "
                             "Default: `targets` from mission.yaml")
    parser.add_argument("--altitude", type=float, default=1.5,
                        help="flight altitude in metres (default 1.5)")
    parser.add_argument("--speed", type=float, default=2.0,
                        help="approximate cruise speed in m/s (default 2.0)")
    parser.add_argument("--tolerance", type=float, default=0.6,
                        help="distance (m) that counts as reaching the LAST "
                             "target (default 0.6)")
    parser.add_argument("--corner-radius", type=float, default=1.5,
                        help="distance (m) at which earlier targets count as "
                             "reached, so the drone flies through them "
                             "instead of stopping (default 1.5)")
    parser.add_argument("--stop-margin", type=float, default=0.5,
                        help="stay this far (m) short of unmapped space "
                             "(default 0.5)")
    parser.add_argument("--turn-in-place", type=float, default=100.0,
                        help="stop and turn first if the heading is off by "
                             "more than this many degrees (default 100)")
    parser.add_argument("--hold-yaw", action="store_true",
                        help="never turn to face the direction of travel "
                             "(only for a 360-degree lidar)")
    parser.add_argument("--max-range", type=float, default=10.0,
                        help="ignore lidar hits beyond this many metres")
    parser.add_argument("--map-rate", type=float, default=5.0,
                        help="scans added to the map per second (default 5)")
    parser.add_argument("--ray-stride", type=int, default=3,
                        help="use every Nth lidar ray (default 3)")
    parser.add_argument("--max-tilt", type=float, default=None,
                        help="skip scans when roll/pitch exceeds this many "
                             "degrees (default: computed from altitude and "
                             "max range)")
    parser.add_argument("--scan-delay", type=float, default=0.05,
                        help="seconds between a scan being taken and it "
                             "arriving here (default 0.05)")
    parser.add_argument("--replan-period", type=float, default=0.3,
                        help="seconds between path checks (default 0.3)")
    parser.add_argument("--reoptimize-period", type=float, default=4.0,
                        help="replan from scratch at least this often, even "
                             "if the current path is still free (default 4)")
    parser.add_argument("--target-timeout", type=float, default=120.0,
                        help="give up on a target after this many seconds")
    parser.add_argument("--stall-timeout", type=float, default=15.0,
                        help="give up if no progress for this many seconds")
    parser.add_argument("--no-path-timeout", type=float, default=6.0,
                        help="give up if no safe path exists for this long")
    parser.add_argument("--use-file-obstacles", action="store_true",
                        help="also use the obstacles listed in mission.yaml")
    parser.add_argument("--show-map", action="store_true",
                        help="print the map after every target")
    parser.add_argument("--address", help="PX4 address (default: from mission.yaml)")
    parser.add_argument("--topic", help="Gazebo scan topic (default: auto-detect)")
    parser.add_argument("--origin", help="start point as LAT,LON (default: PX4 home)")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    if min(args.altitude, args.speed, args.map_rate, args.replan_period) <= 0:
        sys.exit("ERROR: --altitude, --speed, --map-rate and --replan-period "
                 "must be positive.")
    if args.ray_stride < 1:
        sys.exit("ERROR: --ray-stride must be at least 1.")

    try:
        config = load_yaml(args.config)
        planner = build_planner(config)
        if not args.use_file_obstacles:
            planner.clear_obstacles()
        if not args.address:
            args.address = (config.get("connection") or {}).get("address")

        if args.target:
            targets = [parse_target(t) for t in args.target]
        else:
            raw = config.get("targets")
            if not raw:
                raise ConfigError(
                    "No --target given and mission.yaml has no `targets`."
                )
            targets = [load_point(e, f"Target {i}") for i, e in enumerate(raw, 1)]
    except (ValueError, ConfigError) as error:
        sys.exit(f"ERROR: {error}")
    args.address = args.address or "udpin://0.0.0.0:14540"

    for i, t in enumerate(targets, start=1):
        if not planner.is_point_free(t):
            sys.exit(f"ERROR: target {i} ({t.north_m}, {t.east_m}) is outside "
                     "the flight area or too close to a known obstacle.")

    cfg = planner.config
    grid = OccupancyGrid(
        cfg.min_north_m, cfg.max_north_m, cfg.min_east_m, cfg.max_east_m,
        resolution_m=cfg.resolution_m, max_range_m=args.max_range,
    )
    planner.set_occupancy_grid(grid)

    try:
        asyncio.run(run(args, planner, grid, targets))
    except (LidarError, RuntimeError, TimeoutError) as error:
        sys.exit(f"ERROR: {error}")
    except KeyboardInterrupt:
        print("\nInterrupted.")


if __name__ == "__main__":
    main()
