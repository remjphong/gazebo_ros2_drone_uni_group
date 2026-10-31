"""Stage 3: plan a path using ONLY what the lidar has seen.

Takes one scan, builds an occupancy grid from it (obstacles seen within
--max-range metres), then plans from the drone's current position to a
target. Nothing is read from the obstacle list in mission.yaml; only the
planner settings (flight area, grid size, safety margin) are used.

Run with the simulator up and no other MAVSDK script connected:
    python3 scripts/plan_from_lidar.py --target 8,8

--target is NORTH,EAST in metres from the start point (like mission.yaml).

Remember: the planner assumes everything it has not seen is free. One scan
from one spot only shows part of the world (objects hide what is behind
them), so a path may cross space that is not mapped yet. Repeating
scan -> plan while flying (stage 4) is what fixes that.
"""

import argparse
import asyncio
import math
import sys
from pathlib import Path

from lidar_scan import LidarError
from lidar_to_map import body_to_map, run as take_scan
from occupancy_grid import OccupancyGrid, render_ascii
from path_planning import Point2D, waypoints_to_yaml
from plan_route import DEFAULT_CONFIG, ConfigError, build_planner, load_yaml


def scan_to_rays(scan, drone_north, drone_east, heading_deg):
    """Every usable lidar reading -> (end_north, end_east, is_hit)."""
    rays = []
    for i, r in enumerate(scan["ranges"]):
        if math.isnan(r) or r < scan["range_min"]:
            continue
        a = scan["angle_min"] + i * scan["angle_step"]
        is_hit = math.isfinite(r) and r < scan["range_max"] - 1e-6
        length = r if is_hit else scan["range_max"]
        n, e = body_to_map(
            length * math.cos(a), length * math.sin(a),
            drone_north, drone_east, heading_deg,
        )
        rays.append((n, e, is_hit))
    return rays


def parse_target(text):
    try:
        north, east = (float(v) for v in text.split(","))
    except ValueError:
        raise ValueError("--target must look like NORTH,EAST (for example 8,8)") from None
    return Point2D(north, east)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--target", required=True, help="NORTH,EAST in metres")
    parser.add_argument("--max-range", type=float, default=10.0,
                        help="ignore lidar hits beyond this many metres (default 10)")
    parser.add_argument("--use-file-obstacles", action="store_true",
                        help="also use the obstacles listed in mission.yaml")
    parser.add_argument("--address", help="PX4 address (default: from mission.yaml)")
    parser.add_argument("--topic", help="Gazebo scan topic (default: auto-detect)")
    parser.add_argument("--origin", help="start point as LAT,LON (default: PX4 home)")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    try:
        target = parse_target(args.target)
        config = load_yaml(args.config)
        planner = build_planner(config)
        if not args.use_file_obstacles:
            planner.clear_obstacles()
        if not args.address:
            args.address = (config.get("connection") or {}).get("address")
    except (ValueError, ConfigError) as error:
        sys.exit(f"ERROR: {error}")
    args.address = args.address or "udpin://0.0.0.0:14540"

    cfg = planner.config
    grid = OccupancyGrid(
        cfg.min_north_m, cfg.max_north_m, cfg.min_east_m, cfg.max_east_m,
        resolution_m=cfg.resolution_m, max_range_m=args.max_range,
    )
    planner.set_occupancy_grid(grid)

    try:
        origin, before, after, scan = asyncio.run(take_scan(args))
    except (LidarError, ValueError) as error:
        sys.exit(f"ERROR: {error}")

    north, east, heading = before
    print(f"Drone: north={north:.2f} m, east={east:.2f} m, heading={heading:.1f} deg")

    grid.update(north, east, scan_to_rays(scan, north, east, heading))
    occupied = len(grid.occupied_cells())
    print(f"Map: {occupied} occupied cells from lidar hits within {args.max_range:g} m")

    start = Point2D(north, east)
    if not planner.is_point_free(start):
        sys.exit("ERROR: the drone is inside the safety margin of a detected "
                 "obstacle (or outside the flight area), so no path can start here.")
    if not planner.is_point_free(target):
        sys.exit("ERROR: the target is outside the flight area or within the "
                 "safety margin of a detected obstacle.")

    route = planner.plan(start, target)
    if route is None:
        print(render_ascii(grid, cfg.safety_margin_m, None, (north, east),
                           (target.north_m, target.east_m)))
        sys.exit("ERROR: no safe path to the target through the mapped area.")

    print(f"\nPlanned {len(route)} waypoints (north/east from the start point):")
    for i, p in enumerate(route, start=1):
        print(f"  WP{i}: north={p.north_m:.1f} m, east={p.east_m:.1f} m")

    path = [(north, east)] + [(p.north_m, p.east_m) for p in route]
    print("\n# = seen obstacle, + = safety margin, . = seen free, blank = unknown")
    print("D = drone, T = target, * = planned path\n")
    print(render_ascii(grid, cfg.safety_margin_m, path, (north, east),
                       (target.north_m, target.east_m)))


if __name__ == "__main__":
    main()
