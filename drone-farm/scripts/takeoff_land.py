import asyncio

from mavsdk_grpc import System


CONNECTION_ADDRESS = "udpin://0.0.0.0:14540"

TAKEOFF_ALTITUDE_M = 5.0
ALTITUDE_TOLERANCE_M = 0.8
HOVER_TIME_S = 5


async def connect_drone():
    """Connect to PX4 SITL and wait for a valid connection."""
    drone = System()

    print("[1/6] Connecting to PX4 SITL...")

    await drone.connect(
        system_address=CONNECTION_ADDRESS
    )

    async def wait_for_connection():
        async for state in drone.core.connection_state():
            if state.is_connected:
                return

    await asyncio.wait_for(
        wait_for_connection(),
        timeout=20
    )

    print("Connected to PX4 SITL.")

    return drone


async def wait_for_health(drone):
    """Wait until global position and Home are valid."""

    print("[2/6] Waiting for valid position and Home...")

    async def check_health():
        async for health in drone.telemetry.health():
            print(
                f"Global position OK: "
                f"{health.is_global_position_ok}, "
                f"Home OK: {health.is_home_position_ok}"
            )

            if (
                health.is_global_position_ok
                and health.is_home_position_ok
            ):
                return

    await asyncio.wait_for(
        check_health(),
        timeout=60
    )

    print("Position and Home are valid.")


async def wait_for_altitude(drone, target_altitude):
    """Wait until the target relative altitude is reached."""

    async def check_altitude():
        async for position in drone.telemetry.position():
            altitude = position.relative_altitude_m

            print(f"Relative altitude: {altitude:.2f} m")

            if altitude >= target_altitude:
                return

    await asyncio.wait_for(
        check_altitude(),
        timeout=30
    )


async def wait_until_landed(drone):
    """Wait until PX4 reports that the vehicle is disarmed."""

    async def check_landed():
        async for armed in drone.telemetry.armed():
            if not armed:
                return

    await asyncio.wait_for(
        check_landed(),
        timeout=60
    )


async def main():
    drone = None
    airborne = False

    try:
        drone = await connect_drone()

        await wait_for_health(drone)

        # Configure takeoff altitude before arming.
        await drone.action.set_takeoff_altitude(
            TAKEOFF_ALTITUDE_M
        )

        print("[3/6] Arming...")

        await drone.action.arm()

        print("Arm command accepted.")

        print("[4/6] Taking off...")

        await drone.action.takeoff()

        print("Takeoff command accepted.")

        await wait_for_altitude(
            drone,
            TAKEOFF_ALTITUDE_M - ALTITUDE_TOLERANCE_M
        )

        airborne = True

        print("[5/6] Target altitude reached.")
        print(f"Hovering for {HOVER_TIME_S} seconds...")

        await asyncio.sleep(HOVER_TIME_S)

        print("[6/6] Landing...")

        await drone.action.land()

        print("Landing command accepted.")

        await wait_until_landed(drone)

        airborne = False

        print("SUCCESS: Drone has landed and disarmed.")

    except asyncio.TimeoutError:
        print("ERROR: Operation timed out.")
        print(
            "Check PX4 health, telemetry, GPS/Home, "
            "and vehicle state."
        )

    except Exception as error:
        print(f"ERROR: {type(error).__name__}: {error}")

    finally:
        # Best-effort recovery for this SITL test.
        # Never assume a failed script means the drone has landed.
        if drone is not None and airborne:
            print(
                "WARNING: An error occurred while airborne. "
                "Requesting landing..."
            )

            try:
                await drone.action.land()
                await wait_until_landed(drone)
                print("Landing confirmed.")
            except Exception as recovery_error:
                print(
                    "CRITICAL: Could not confirm landing: "
                    f"{recovery_error}"
                )
                print(
                    "Inspect the vehicle in Gazebo/PX4 "
                    "before continuing."
                )


if __name__ == "__main__":
    asyncio.run(main())
