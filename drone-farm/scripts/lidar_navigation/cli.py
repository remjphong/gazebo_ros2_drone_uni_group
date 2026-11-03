"""Command-line options and the main() entry point."""

import argparse
import asyncio
import sys
from pathlib import Path

from lidar_scan import LidarError
from plan_from_lidar import parse_target
from plan_route import (
    ConfigError,
    DEFAULT_CONFIG,
    build_planner,
    load_point,
    load_yaml,
)

from .constants import DEFAULT_MAP_FILE, SCRIPT_DIR
from .mapping import ResistantGrid
from .mission import run


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--explore", action="store_true",
                        help="no targets needed: fly to the edge of the "
                             "known map again and again until nothing "
                             "reachable is unexplored, then return to the "
                             "start. The map is saved for later runs")
    parser.add_argument("--explore-timeout", type=float, default=900.0,
                        help="stop exploring after this many seconds and go "
                             "home (default 900)")
    parser.add_argument("--min-frontier", type=int, default=3,
                        help="ignore unexplored gaps smaller than this many "
                             "map cells (default 3)")
    parser.add_argument("--no-return", action="store_true",
                        help="after exploring, land where the drone is "
                             "instead of flying back to the start")
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
    parser.add_argument("--no-governor", dest="governor", action="store_false",
                        help="do not slow down before turns (always aim a "
                             "full lookahead ahead, as before)")
    parser.add_argument("--corner-speed", type=float, default=1.0,
                        help="speed (m/s) to slow to at a 90 degree turn "
                             "(default 1.0)")
    parser.add_argument("--turn-speed", type=float, default=0.5,
                        help="speed (m/s) to slow to where the path "
                             "reverses, 150 degrees or more (default 0.5)")
    parser.add_argument("--waypoint-speed", type=float, default=1.0,
                        help="speed (m/s) when reaching a target whose next "
                             "direction is not known, as in --explore "
                             "(default 1.0)")
    parser.add_argument("--plan-decel", type=float, default=1.2,
                        help="how gently to plan slowing down, in m/s^2 "
                             "(default 1.2; PX4 itself can brake harder)")
    parser.add_argument("--plan-accel", type=float, default=1.5,
                        help="how quickly to speed up again after a turn or "
                             "a stop, in m/s^2 (default 1.5)")
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
    parser.add_argument("--accel", type=float, default=None,
                        help="limit how quickly the drone may speed up or "
                             "slow down, in m/s^2 (default: no limit). Warning: "
                             "a low limit makes it stop later, possibly "
                             "inside the safety margin")
    parser.add_argument("--yaw-rate", type=float, default=70.0,
                        help="fastest the heading may turn, in degrees per "
                             "second (default 70)")
    parser.add_argument("--no-shaping", action="store_true",
                        help="send raw setpoints without smoothing (the "
                             "previous behaviour, to compare flight logs)")
    parser.add_argument("--no-log", dest="log", action="store_false",
                        help="do not write the flight log CSV")
    parser.add_argument("--log-dir", type=Path,
                        default=SCRIPT_DIR / "flight_logs",
                        help="folder for the flight log (default flight_logs/ "
                             "next to this script)")
    parser.add_argument("--log-file", type=Path,
                        help="exact log file name (default: last_flight.csv "
                             "in --log-dir, overwritten by every flight)")
    parser.add_argument("--log-rate", type=float, default=10.0,
                        help="flight log rows per second (default 10)")
    parser.add_argument("--occupied-resistance", type=float, default=0.5,
                        help="how strongly a wall resists being erased by "
                             "beams that graze past it, 0-1 (default 0.5; "
                             "1 = original behaviour, lower = walls erase "
                             "more slowly)")
    parser.add_argument("--map-file", type=Path, default=DEFAULT_MAP_FILE,
                        help="where the map is saved between runs "
                             f"(default {DEFAULT_MAP_FILE.name} next to this "
                             "script)")
    parser.add_argument("--no-load-map", dest="load_map", action="store_false",
                        help="start with an empty map even if a saved one "
                             "exists (a new one is still saved at the end)")
    parser.add_argument("--no-save-map", dest="save_map", action="store_false",
                        help="do not write the map file at the end")
    parser.add_argument("--map-decay", type=float, default=0.6,
                        help="how much to trust a loaded map, 0-1 (default "
                             "0.6). Lower lets fresh scans overrule it "
                             "faster; 1 trusts it fully")
    parser.add_argument("--map-origin-tol", type=float, default=1.0,
                        help="ignore the saved map if this run's start point "
                             "is more than this many metres from the one it "
                             "was made at (default 1.0)")
    parser.add_argument("--use-file-obstacles", action="store_true",
                        help="also use the obstacles listed in mission.yaml")
    parser.add_argument("--show-map", action="store_true",
                        help="print the map after every target")
    parser.add_argument("--address", help="PX4 address (default: from mission.yaml)")
    parser.add_argument("--topic", help="Gazebo scan topic (default: auto-detect)")
    parser.add_argument("--origin", help="start point as LAT,LON (default: PX4 home)")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--timeout", type=float, default=10.0)
    return parser


def main():
    args = build_parser().parse_args()

    if min(args.altitude, args.speed, args.map_rate, args.replan_period,
           args.log_rate, args.yaw_rate) <= 0 or (
               args.accel is not None and args.accel <= 0):
        sys.exit("ERROR: --altitude, --speed, --map-rate, --replan-period, "
                 "--log-rate, --yaw-rate and --accel must be positive.")
    if args.ray_stride < 1:
        sys.exit("ERROR: --ray-stride must be at least 1.")
    if not 0.0 < args.map_decay <= 1.0:
        sys.exit("ERROR: --map-decay must be above 0 and at most 1.")
    if not 0.0 < args.occupied_resistance <= 1.0:
        sys.exit("ERROR: --occupied-resistance must be above 0 and at most 1.")
    if min(args.corner_speed, args.turn_speed, args.waypoint_speed,
           args.plan_decel, args.plan_accel) <= 0:
        sys.exit("ERROR: --corner-speed, --turn-speed, --waypoint-speed, "
                 "--plan-decel and --plan-accel must be positive.")
    if args.explore and args.target:
        sys.exit("ERROR: use either --explore or --target, not both.")
    if args.min_frontier < 1 or args.explore_timeout <= 0:
        sys.exit("ERROR: --min-frontier must be at least 1 and "
                 "--explore-timeout positive.")

    try:
        config = load_yaml(args.config)
        planner = build_planner(config)
        if not args.use_file_obstacles:
            planner.clear_obstacles()
        if not args.address:
            args.address = (config.get("connection") or {}).get("address")

        if args.explore:
            targets = []  # chosen by the drone itself
        elif args.target:
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
    grid = ResistantGrid(
        cfg.min_north_m, cfg.max_north_m, cfg.min_east_m, cfg.max_east_m,
        resolution_m=cfg.resolution_m, max_range_m=args.max_range,
        occupied_resistance=args.occupied_resistance,
    )
    planner.set_occupancy_grid(grid)

    try:
        asyncio.run(run(args, planner, grid, targets))
    except (LidarError, RuntimeError, TimeoutError) as error:
        sys.exit(f"ERROR: {error}")
    except KeyboardInterrupt:
        print("\nInterrupted.")
