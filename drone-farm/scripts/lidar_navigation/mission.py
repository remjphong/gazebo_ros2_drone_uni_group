"""Setting up and running one flight (takeoff, navigate, land)."""

import asyncio
import math
import time

from lidar_scan import LidarError, find_scan_topic
from lidar_to_map import connect, read_origin
from occupancy_grid import render_ascii

from .exploration import coverage
from .flight import (
    OffboardController,
    PoseTracker,
    SetpointShaper,
    arm_and_takeoff,
    land_and_wait,
    set_telemetry_rates,
    wait_for_health,
)
from .lidar import LidarStream
from .mapping import MapService, Mapper, load_grid, persist_map
from .navigator import Navigator
from .reporting import FlightLogger, Stats


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
    tracker = ctrl = stream = mapper = svc = origin = logger = None
    airborne = False
    saved = False

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

        if args.load_map:
            if args.map_file.exists():
                _, message = load_grid(grid, args.map_file, origin,
                                       args.map_origin_tol, args.map_decay)
                print(message)
            else:
                print(f"No saved map at {args.map_file}: starting with an "
                      "empty map (one will be saved at the end).")

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

        shaper = None if args.no_shaping else SetpointShaper(
            args.accel, args.yaw_rate
        )
        ctrl = OffboardController(drone, shaper)
        if shaper is None:
            print("Setpoint smoothing is OFF (--no-shaping).")
        else:
            accel = ("not limited" if args.accel is None
                     else f"limited to about {args.accel:g} m/s^2")
            print(f"Setpoint smoothing: turning at most {args.yaw_rate:g} "
                  f"deg/s, acceleration {accel}.")
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
        if args.log:
            # One log file, overwritten by every flight, so logs never pile
            # up on disk. Copy it elsewhere to keep one.
            log_path = args.log_file or (args.log_dir / "last_flight.csv")
            logger = FlightLogger(log_path, tracker, ctrl, svc, nav, stats,
                                  args.log_rate)
            try:
                logger.start()
                print(f"Logging flight data to {log_path}")
            except OSError as error:
                print(f"(Flight log not started: {error})")
                logger = None
        if args.explore:
            success = await nav.explore()
        else:
            success = await nav.run(targets)
        if logger is not None:
            await logger.stop()   # before landing, so the descent is not logged
            logger = None
        print("\nNavigation finished." if success else "\nNavigation FAILED.")
        stats.report(stream, nav.flown)

        await mapper.stop()
        mapper = None
        saved = persist_map(grid, args, origin, svc.lock)

        final = tracker.pose()
        print("\n# = seen obstacle, + = safety margin, . = seen free, "
              "blank = unknown")
        print("D = drone, T = last target, * = path flown\n")
        if args.explore:
            print(f"(Exploration covered {100 * coverage(grid):.0f}% of the "
                  "flight area; the rest is out of reach or hidden.)\n")
        print(render_ascii(grid, cfg.safety_margin_m, nav.trail,
                           (final[0], final[1]),
                           (targets[-1].north_m, targets[-1].east_m)
                           if targets else None))

        print("\nLanding...")
        await ctrl.stop()
        await land_and_wait(drone)
        airborne = False

    finally:
        # Keep what was learned even if the flight ended early (a quick
        # write; the lock stops a half-finished scan update from being saved).
        if logger is not None:
            await logger.stop()
            logger = None
        if mapper is not None:
            await mapper.stop()
            mapper = None
        if not saved and svc is not None:
            persist_map(grid, args, origin, svc.lock)

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
        if stream is not None:
            stream.stop()
        if tracker is not None:
            await tracker.stop()
