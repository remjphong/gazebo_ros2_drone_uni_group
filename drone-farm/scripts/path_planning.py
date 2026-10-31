"""2D grid A* path planning for the drone farm simulation.

Uses the same local frame as config/mission.yaml: offsets in metres from the
drone's initial GPS position (north_m: positive = north, east_m: positive = east).
The planner output can be pasted into mission.yaml under `mission: waypoints:`.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import Optional


@dataclass(frozen=True)
class Point2D:
    north_m: float = 0.0
    east_m: float = 0.0


@dataclass(frozen=True)
class Obstacle:
    """Circular no-go zone (tree, tower, building, ...)."""

    center: Point2D
    radius_m: float

    def __post_init__(self):
        if self.radius_m < 0:
            raise ValueError("Obstacle radius must not be negative.")

    def distance_to(self, p: "Point2D") -> float:
        """Distance from p to the obstacle's surface (negative inside)."""
        return _distance(p, self.center) - self.radius_m


@dataclass(frozen=True)
class RectObstacle:
    """Axis-aligned rectangle (crop row, building, ...).

    size_north_m is the length along north, size_east_m the width along east.
    """

    center: Point2D
    size_north_m: float
    size_east_m: float

    def __post_init__(self):
        if self.size_north_m <= 0 or self.size_east_m <= 0:
            raise ValueError("Rectangle sizes must be greater than zero.")

    def distance_to(self, p: "Point2D") -> float:
        """Distance from p to the rectangle's surface (negative inside)."""
        dn = abs(p.north_m - self.center.north_m) - self.size_north_m / 2.0
        de = abs(p.east_m - self.center.east_m) - self.size_east_m / 2.0
        if dn > 0 or de > 0:
            return math.hypot(max(dn, 0.0), max(de, 0.0))
        return max(dn, de)


@dataclass
class PlannerConfig:
    # Rectangular flight area; the planner never leaves it.
    min_north_m: float = -20.0
    max_north_m: float = 20.0
    min_east_m: float = -20.0
    max_east_m: float = 20.0

    # Grid cell size. Smaller = finer paths, slower search.
    resolution_m: float = 0.5

    # Extra clearance kept around every obstacle (drone size + GPS error).
    safety_margin_m: float = 1.0

    # Remove unnecessary intermediate waypoints after A*.
    simplify_path: bool = True

    def __post_init__(self):
        if self.resolution_m <= 0:
            raise ValueError("resolution_m must be greater than zero.")
        if self.safety_margin_m < 0:
            raise ValueError("safety_margin_m must not be negative.")
        if (
            self.max_north_m <= self.min_north_m
            or self.max_east_m <= self.min_east_m
        ):
            raise ValueError("Flight area bounds are invalid.")


def _distance(a: Point2D, b: Point2D) -> float:
    return math.hypot(a.north_m - b.north_m, a.east_m - b.east_m)


class PathPlanner:
    def __init__(self, config: PlannerConfig):
        self.config = config
        self.obstacles: list = []
        self.occupancy = None  # optional OccupancyGrid built from sensor data

        self.rows = int(
            math.floor(
                (config.max_north_m - config.min_north_m) / config.resolution_m
            )
        ) + 1
        self.cols = int(
            math.floor(
                (config.max_east_m - config.min_east_m) / config.resolution_m
            )
        ) + 1

    # ------------------------------------------------------------------
    # Obstacles and collision checks
    # ------------------------------------------------------------------
    def add_obstacle(self, obstacle) -> None:
        """Add an Obstacle (circle) or RectObstacle (rectangle)."""
        self.obstacles.append(obstacle)

    def clear_obstacles(self) -> None:
        self.obstacles.clear()

    def set_occupancy_grid(self, grid) -> None:
        """Also avoid everything an OccupancyGrid (e.g. from a lidar) has seen.

        Unknown space counts as free, so the planner only avoids what the
        sensor has actually detected.
        """
        self.occupancy = grid

    def is_point_free(self, p: Point2D) -> bool:
        """True if a drone centred at p is inside the flight area and at
        least safety_margin_m away from every obstacle."""
        if not (math.isfinite(p.north_m) and math.isfinite(p.east_m)):
            return False

        cfg = self.config
        if not (
            cfg.min_north_m <= p.north_m <= cfg.max_north_m
            and cfg.min_east_m <= p.east_m <= cfg.max_east_m
        ):
            return False

        for obstacle in self.obstacles:
            # Tiny tolerance so points exactly on the margin count as free.
            if obstacle.distance_to(p) < cfg.safety_margin_m - 1e-9:
                return False

        if self.occupancy is not None and self.occupancy.is_blocked(
            p.north_m, p.east_m, cfg.safety_margin_m
        ):
            return False
        return True

    def is_segment_free(self, a: Point2D, b: Point2D) -> bool:
        """True if the straight segment a -> b stays entirely in free space."""
        # Sample finer than the grid so thin obstacles cannot slip through.
        step = self.config.resolution_m / 2.0
        samples = max(1, math.ceil(_distance(a, b) / step))

        for i in range(samples + 1):
            t = i / samples
            p = Point2D(
                a.north_m + t * (b.north_m - a.north_m),
                a.east_m + t * (b.east_m - a.east_m),
            )
            if not self.is_point_free(p):
                return False
        return True

    # ------------------------------------------------------------------
    # Grid helpers
    # ------------------------------------------------------------------
    def _in_grid(self, row: int, col: int) -> bool:
        return 0 <= row < self.rows and 0 <= col < self.cols

    def _to_cell(self, p: Point2D) -> tuple[int, int]:
        cfg = self.config
        return (
            round((p.north_m - cfg.min_north_m) / cfg.resolution_m),
            round((p.east_m - cfg.min_east_m) / cfg.resolution_m),
        )

    def _to_point(self, row: int, col: int) -> Point2D:
        cfg = self.config
        return Point2D(
            cfg.min_north_m + row * cfg.resolution_m,
            cfg.min_east_m + col * cfg.resolution_m,
        )

    def _is_cell_free(self, row: int, col: int) -> bool:
        return self._in_grid(row, col) and self.is_point_free(
            self._to_point(row, col)
        )

    # ------------------------------------------------------------------
    # Planning
    # ------------------------------------------------------------------
    def plan(
        self,
        start: Point2D,
        goal: Point2D,
        allow_blocked_start: bool = False,
    ) -> Optional[list[Point2D]]:
        """A* search from start to goal.

        Returns the waypoints to fly AFTER start (the last one is the goal),
        or None if start/goal are blocked or no path exists.

        allow_blocked_start lets the search begin inside a safety margin
        (for example a drone that ended up close to a sensed obstacle), so
        it can find its way out instead of giving up.
        """
        if not self.is_point_free(goal):
            return None
        if not allow_blocked_start and not self.is_point_free(start):
            return None

        # Straight line is free: no search needed.
        if self.is_segment_free(start, goal):
            return [goal]

        start_cell = self._to_cell(start)
        goal_cell = self._to_cell(goal)

        if not self._is_cell_free(*goal_cell):
            return None
        start_blocked = not self._is_cell_free(*start_cell)
        if start_blocked and not allow_blocked_start:
            return None

        res = self.config.resolution_m
        sqrt2 = math.sqrt(2.0)

        def heuristic(cell: tuple[int, int]) -> float:
            # Octile distance: exact cost on an obstacle-free 8-connected grid.
            dr = abs(cell[0] - goal_cell[0])
            dc = abs(cell[1] - goal_cell[1])
            return res * ((dr + dc) + (sqrt2 - 2.0) * min(dr, dc))

        g_cost = {start_cell: 0.0}
        parent: dict[tuple[int, int], tuple[int, int]] = {}
        closed: set[tuple[int, int]] = set()
        open_heap = [(heuristic(start_cell), start_cell)]

        found = False
        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            closed.add(current)

            if current == goal_cell:
                found = True
                break

            row, col = current
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    if dr == 0 and dc == 0:
                        continue

                    neighbor = (row + dr, col + dc)
                    if neighbor in closed or not self._is_cell_free(*neighbor):
                        continue

                    # Do not cut diagonally between two blocked cells
                    # (except when stepping out of a blocked start cell:
                    # a boxed-in start must be allowed to leave).
                    escaping = start_blocked and current == start_cell
                    if dr != 0 and dc != 0 and not escaping:
                        if not self._is_cell_free(
                            row + dr, col
                        ) or not self._is_cell_free(row, col + dc):
                            continue

                    step = res * (sqrt2 if dr != 0 and dc != 0 else 1.0)
                    tentative = g_cost[current] + step

                    if tentative < g_cost.get(neighbor, math.inf):
                        g_cost[neighbor] = tentative
                        parent[neighbor] = current
                        heapq.heappush(
                            open_heap, (tentative + heuristic(neighbor), neighbor)
                        )

        if not found:
            return None

        # Rebuild path (goal -> start), then reverse.
        cells = [goal_cell]
        while cells[-1] in parent:
            cells.append(parent[cells[-1]])
        cells.reverse()

        path = [self._to_point(r, c) for r, c in cells]

        # Use the exact start/goal instead of snapped cell centres.
        path[0] = start
        path[-1] = goal

        if self.config.simplify_path:
            path = self._simplify(path)

        return path[1:]  # caller already knows the start

    def _simplify(self, path: list[Point2D]) -> list[Point2D]:
        """Greedy line-of-sight: from each anchor, jump to the farthest
        waypoint reachable in a straight collision-free line."""
        if len(path) <= 2:
            return path

        result = [path[0]]
        anchor = 0

        while anchor < len(path) - 1:
            next_index = anchor + 1
            for i in range(len(path) - 1, anchor + 1, -1):
                if self.is_segment_free(path[anchor], path[i]):
                    next_index = i
                    break
            result.append(path[next_index])
            anchor = next_index

        return result


def waypoints_to_yaml(waypoints: list[Point2D]) -> str:
    """Format waypoints to paste under `mission: waypoints:` in mission.yaml."""
    blocks = [
        f"    - north_m: {wp.north_m:.1f}\n      east_m: {wp.east_m:.1f}\n"
        for wp in waypoints
    ]
    return "\n".join(blocks)


if __name__ == "__main__":
    config = PlannerConfig(
        min_north_m=-5.0, max_north_m=20.0, min_east_m=-5.0, max_east_m=20.0
    )
    planner = PathPlanner(config)
    planner.add_obstacle(Obstacle(Point2D(4.0, 5.0), 1.5))  # e.g. a tree

    path = planner.plan(Point2D(0.0, 0.0), Point2D(8.0, 5.0))
    if path is None:
        raise SystemExit("No collision-free path found.")

    print("waypoints:")
    print(waypoints_to_yaml(path))
