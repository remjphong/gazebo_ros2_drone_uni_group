import argparse
import asyncio
import time

from mavsdk_grpc.offboard import PositionNedYaw

from lidar_to_map import connect, read_origin
from online_navigation import (
    PoseTracker,
    angle_diff,
    arm_and_takeoff,
    land_and_wait,
    wait_for_health,
)


async def get_ned_position(drone):
    async for pv in drone.telemetry.position_velocity_ned():
        return (
            pv.position.north_m,
            pv.position.east_m,
            pv.position.down_m,
        )

    raise RuntimeError("Could not read NED position")


async def send_position_setpoint(drone, north, east, down, yaw):
    await drone.offboard.set_position_ned(
        PositionNedYaw(
            north,
            east,
            down,
            float(yaw),
        )
    )


async def run_yaw_test(drone, tracker, position, max_wait):
    north, east, down = position
    targets = [350, 10, 180, 360, 5, 355, 90, 270]

    print(f"\nInitial heading: {tracker.heading:.1f} degrees")
    print("\nPreparing Offboard position hold...")

    # PX4 requires a continuous setpoint stream before Offboard starts.
    initial_yaw = tracker.heading
    for _ in range(20):
        await send_position_setpoint(
            drone, north, east, down, initial_yaw
        )
        await asyncio.sleep(0.1)

    await drone.offboard.start()
    print("Offboard started.")

    try:
        for target in targets:
            start = tracker.heading

            print(f"\nCommanding yaw {target} degrees...")

            stable_since = None
            deadline = time.monotonic() + max_wait

            while time.monotonic() < deadline:
                await send_position_setpoint(
                    drone, north, east, down, target
                )

                actual = tracker.heading
                error = abs(angle_diff(target, actual))

                if error < 5:
                    if stable_since is None:
                        stable_since = time.monotonic()

                    if time.monotonic() - stable_since >= 1.0:
                        break
                else:
                    stable_since = None

                await asyncio.sleep(0.1)

            actual = tracker.heading
            error = abs(angle_diff(target, actual))

            print(
                f"Start: {start:.1f} | "
                f"Target: {target:.1f} | "
                f"Actual: {actual:.1f} | "
                f"Error: {error:.1f}"
            )

            if error < 5:
                print("RESULT: OK")
            else:
                print("RESULT: NOT FOLLOWED")

            await asyncio.sleep(2)

    finally:
        print("\nStopping Offboard...")
        await drone.offboard.stop()


async def wait_for_takeoff_altitude(drone, target_altitude, timeout=30):
    print(f"Waiting for altitude of {target_altitude:.1f} m...")

    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        position = await get_ned_position(drone)

        # NED down is negative when the drone is above its origin.
        altitude = -position[2]

        print(f"\rCurrent altitude: {altitude:.2f} m", end="", flush=True)

        if altitude >= target_altitude * 0.9:
            print("\nTarget altitude reached.")
            return

        await asyncio.sleep(0.2)

    raise TimeoutError(
        f"Drone did not reach {target_altitude:.1f} m within {timeout} seconds."
    )

async def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--address",
        default="udpin://0.0.0.0:14540",
    )
    parser.add_argument("--timeout", type=float, default=10)
    parser.add_argument("--altitude", type=float, default=1.5)
    parser.add_argument("--max-wait", type=float, default=12)

    args = parser.parse_args()

    drone = await connect(args.address, args.timeout)

    tracker = None
    airborne = False

    try:
        await wait_for_health(drone, 60)

        origin = await read_origin(drone, args.timeout)

        tracker = PoseTracker(drone, origin)
        tracker.start()

        await tracker.wait_ready(args.timeout)

        print(f"Taking off to {args.altitude} m...")

        await arm_and_takeoff(drone, args.altitude)
        airborne = True

        await wait_for_takeoff_altitude(drone, args.altitude)

        await asyncio.sleep(2)

        position = await get_ned_position(drone)

        print(
            "\nPosition hold point: "
            f"N={position[0]:.2f}, "
            f"E={position[1]:.2f}, "
            f"D={position[2]:.2f}"
        )

        await run_yaw_test(
            drone,
            tracker,
            position,
            args.max_wait,
        )

        print("\nLanding...")
        await land_and_wait(drone)
        airborne = False

    finally:
        if airborne:
            print("\nCleanup: landing...")
            await land_and_wait(drone)

        if tracker is not None:
            await tracker.stop()


if __name__ == "__main__":
    asyncio.run(main())
