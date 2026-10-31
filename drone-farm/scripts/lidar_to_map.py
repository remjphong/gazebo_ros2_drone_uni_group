"""Stage 2: put ONE lidar scan on the map.

Takes a scan from the simulator, reads the drone's position and heading from
PX4, and converts every lidar hit to north/east metres from the start point,
the same frame as config/mission.yaml. Then it compares the hits with the
obstacles listed in mission.yaml.

Run it with the simulator up (gz_x500_lidar_2d) and NO other MAVSDK script
connected (they would fight over UDP port 14540):
    python3 scripts/lidar_to_map.py

The real test: run it with the drone at the start, then again after moving or
turning it (for example with QGroundControl). The same obstacles should stay
at the same map positions. If they move, the frame maths is wrong somewhere.
"""

import argparse
import asyncio
import csv
import math
import sys
from pathlib import Path

from mavsdk_grpc import System

from lidar_scan import (
    LidarError,
    find_scan_topic,
    grab_scan_text,
    parse_scan,
    scan_to_hits,
)
from path_planning import Point2D
from plan_route import DEFAULT_CONFIG, ConfigError, build_planner, load_yaml

EARTH_RADIUS_M = 6378137.0  # same value as waypoint_mission.py


# ----------------------------------------------------------------------
# Frame maths
# ----------------------------------------------------------------------
def gps_to_offset(lat, lon, origin_lat, origin_lon):
    """GPS -> (north_m, east_m) from the origin (inverse of offset_to_gps)."""
    north = math.radians(lat - origin_lat) * EARTH_RADIUS_M
    east = (
        math.radians(lon - origin_lon)
        * EARTH_RADIUS_M
        * math.cos(math.radians(origin_lat))
    )
    return north, east


def body_to_map(x_forward, y_left, drone_north, drone_east, heading_deg):
    """Lidar point (x ahead, y left of the drone) -> (north_m, east_m).

    heading_deg is the compass heading: 0 = north, 90 = east (clockwise).
    The lidar's own position on the drone is ignored (a few cm).
    """
    h = math.radians(heading_deg)
    north = drone_north + x_forward * math.cos(h) + y_left * math.sin(h)
    east = drone_east + x_forward * math.sin(h) - y_left * math.cos(h)
    return north, east


# ----------------------------------------------------------------------
# PX4 telemetry
# ----------------------------------------------------------------------
async def first(stream, timeout, what):
    async def get():
        async for item in stream:
            return item

    try:
        return await asyncio.wait_for(get(), timeout)
    except asyncio.TimeoutError:
        raise LidarError(f"Timed out waiting for {what} from PX4.") from None


async def connect(address, timeout):
    drone = System()
    await drone.connect(system_address=address)

    async def wait_connected():
        async for state in drone.core.connection_state():
            if state.is_connected:
                return

    try:
        await asyncio.wait_for(wait_connected(), timeout)
    except asyncio.TimeoutError:
        raise LidarError(
            f"Could not connect to PX4 at {address}. Is the simulator "
            "running, and is another MAVSDK script already connected?"
        ) from None
    return drone


async def read_heading(drone, timeout):
    try:
        heading = await first(drone.telemetry.heading(), timeout, "heading")
        return heading.heading_deg % 360.0
    except AttributeError:
        euler = await first(
            drone.telemetry.attitude_euler(), timeout, "attitude"
        )
        return euler.yaw_deg % 360.0


async def read_origin(drone, timeout):
    home = await first(drone.telemetry.home(), timeout, "home position")
    return home.latitude_deg, home.longitude_deg


async def read_pose(drone, origin, timeout):
    """Return (north_m, east_m, heading_deg) relative to origin."""
    position = await first(drone.telemetry.position(), timeout, "position")
    heading = await read_heading(drone, timeout)
    north, east = gps_to_offset(
        position.latitude_deg, position.longitude_deg, origin[0], origin[1]
    )
    return north, east, heading


# ----------------------------------------------------------------------
# Comparing with the known map
# ----------------------------------------------------------------------
def compare_with_known(points, obstacles, tolerance):
    """For each map point, find the nearest known obstacle surface."""
    explained, unexplained = [], []
    per_obstacle = [0] * len(obstacles)

    for north, east in points:
        p = Point2D(north, east)
        best_index, best_dist = None, math.inf
        for i, obstacle in enumerate(obstacles):
            d = abs(obstacle.distance_to(p))
            if d < best_dist:
                best_index, best_dist = i, d
        if best_index is not None and best_dist <= tolerance:
            explained.append(best_dist)
            per_obstacle[best_index] += 1
        else:
            unexplained.append((north, east))

    return explained, unexplained, per_obstacle


def describe(obstacle):
    c = obstacle.center
    if hasattr(obstacle, "radius_m"):
        return f"circle at ({c.north_m:g} N, {c.east_m:g} E), r={obstacle.radius_m:g}"
    return (
        f"rectangle at ({c.north_m:g} N, {c.east_m:g} E), "
        f"{obstacle.size_north_m:g} x {obstacle.size_east_m:g}"
    )


# ----------------------------------------------------------------------
async def run(args):
    drone = await connect(args.address, args.timeout)

    if args.origin:
        try:
            lat, lon = (float(v) for v in args.origin.split(","))
            origin = (lat, lon)
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

    topic = args.topic or find_scan_topic(args.timeout)

    before = await read_pose(drone, origin, args.timeout)
    scan = parse_scan(grab_scan_text(topic, args.timeout))
    after = await read_pose(drone, origin, args.timeout)

    return origin, before, after, scan


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--address", help="PX4 address (default: from mission.yaml)")
    parser.add_argument("--topic", help="Gazebo scan topic (default: auto-detect)")
    parser.add_argument("--origin", help="start point as LAT,LON (default: PX4 home)")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--tolerance", type=float, default=0.3,
                        help="max distance (m) from a known surface to count "
                             "as matching it (default 0.3)")
    parser.add_argument("--csv", help="also write the map points to this file")
    args = parser.parse_args()

    obstacles = []
    try:
        config = load_yaml(args.config)
        obstacles = build_planner(config).obstacles
        if not args.address:
            args.address = (config.get("connection") or {}).get("address")
    except ConfigError as error:
        print(f"(No comparison with known obstacles: {error})")
    args.address = args.address or "udpin://0.0.0.0:14540"

    try:
        origin, before, after, scan = asyncio.run(run(args))
    except (LidarError, ValueError) as error:
        sys.exit(f"ERROR: {error}")

    north, east, heading = before
    print(f"Start point (origin): {origin[0]:.7f}, {origin[1]:.7f}")
    print(f"Drone: north={north:.2f} m, east={east:.2f} m, heading={heading:.1f} deg")

    moved = math.hypot(after[0] - north, after[1] - east)
    turned = abs((after[2] - heading + 180.0) % 360.0 - 180.0)
    if moved > 0.3 or turned > 3.0:
        print(
            f"WARNING: the drone moved {moved:.2f} m and turned {turned:.1f} deg "
            "during the scan, so the map points are blurred. Hold still."
        )

    hits = scan_to_hits(scan)
    points = [
        body_to_map(h[0], h[1], north, east, heading) for h in hits
    ]
    print(f"Lidar hits placed on the map: {len(points)}")

    if args.csv:
        with open(args.csv, "w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(["north_m", "east_m"])
            writer.writerows((f"{n:.3f}", f"{e:.3f}") for n, e in points)
        print(f"Wrote {args.csv}")

    if not obstacles or not points:
        return

    explained, unexplained, per_obstacle = compare_with_known(
        points, obstacles, args.tolerance
    )
    pct = 100 * len(explained) / len(points)
    print(f"\nComparison with mission.yaml (tolerance {args.tolerance:.2f} m):")
    if explained:
        print(
            f"  matching a known obstacle: {len(explained)} ({pct:.0f}%), "
            f"mean error {sum(explained) / len(explained):.2f} m, "
            f"worst {max(explained):.2f} m"
        )
    else:
        print("  matching a known obstacle: 0 (0%)")
    for i, count in enumerate(per_obstacle, start=1):
        if count:
            print(f"    obstacle {i}: {count} hits  [{describe(obstacles[i - 1])}]")

    print(f"  NOT matching anything known: {len(unexplained)}")
    for n, e in unexplained[:8]:
        print(f"    north={n:.2f}, east={e:.2f}")
    if len(unexplained) > 8:
        print(f"    ... and {len(unexplained) - 8} more")
    if unexplained and pct < 80:
        print("  Many hits match nothing: suspect the heading or frame "
              "conversion (or the world differs from mission.yaml).")
    elif unexplained:
        print("  These could be a new obstacle, or something missing from "
              "mission.yaml.")


if __name__ == "__main__":
    main()
