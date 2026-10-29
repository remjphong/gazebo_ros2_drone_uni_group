import asyncio
import math

from mavsdk_grpc import System
from mavsdk_grpc.mission import MissionItem, MissionPlan

from pathlib import Path

import yaml


PROJECT_ROOT = Path(__file__).resolve().parent.parent
CONFIG_PATH = PROJECT_ROOT / "config" / "planned_mission.yaml"

def load_config():
    with CONFIG_PATH.open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file)

    if not isinstance(config, dict):
        raise ValueError("Mission configuration must be a YAML mapping.")

    mission = config.get("mission", {})
    waypoints = mission.get("waypoints", [])

    if not waypoints:
        raise ValueError("Mission must contain at least one waypoint.")

    if mission.get("altitude_m", 0) <= 0:
        raise ValueError("Mission altitude must be greater than zero.")

    if mission.get("speed_m_s", 0) <= 0:
        raise ValueError("Mission speed must be greater than zero.")

    for index, waypoint in enumerate(waypoints, start=1):
        if "north_m" not in waypoint or "east_m" not in waypoint:
            raise ValueError(
                f"Waypoint {index} must define north_m and east_m."
            )

    return config
    
async def wait_for_connection(drone, connection_config):
    print("[1] Connecting to PX4 SITL...")

    await drone.connect(
        system_address=connection_config["address"]
    )

    async def check():
        async for state in drone.core.connection_state():
            if state.is_connected:
                return

    await asyncio.wait_for(
        check(),
        timeout=connection_config["timeout_seconds"],
    )

    print("Connected.")
    
async def get_initial_position(drone, safety_config):
    print("[2] Waiting for valid global position and Home...")

    async def check_health():
        async for health in drone.telemetry.health():
            if (
                health.is_global_position_ok
                and health.is_home_position_ok
            ):
                return

    await asyncio.wait_for(
        check_health(),
        timeout=safety_config["global_position_timeout_seconds"],
    )

    async for position in drone.telemetry.position():
        lat = position.latitude_deg
        lon = position.longitude_deg

        if math.isfinite(lat) and math.isfinite(lon):
            print(f"Start latitude:  {lat:.7f}")
            print(f"Start longitude: {lon:.7f}")
            return lat, lon

    raise RuntimeError("Could not obtain a valid GPS position.")

def offset_to_gps(lat, lon, north_m, east_m):
    """Convert local north/east offsets to approximate GPS coordinates."""

    earth_radius_m = 6378137.0

    new_lat = lat + math.degrees(
        north_m / earth_radius_m
    )

    cos_lat = math.cos(math.radians(lat))

    if abs(cos_lat) < 1e-6:
        raise ValueError("Longitude conversion is unstable near the poles.")

    new_lon = lon + math.degrees(
        east_m / (earth_radius_m * cos_lat)
    )

    return new_lat, new_lon


def build_mission(home_lat, home_lon, mission_config):
    mission_items = []

    altitude = mission_config["altitude_m"]
    speed = mission_config["speed_m_s"]

    for index, waypoint in enumerate(
        mission_config["waypoints"],
        start=1,
    ):
        lat, lon = offset_to_gps(
            home_lat,
            home_lon,
            waypoint["north_m"],
            waypoint["east_m"],
        )

        item = MissionItem(
            lat,
            lon,
            altitude,
            speed,
            False,
            float("nan"),
            float("nan"),
            MissionItem.CameraAction.NONE,
            float("nan"),
            float("nan"),
            float("nan"),
            float("nan"),
            float("nan"),
            MissionItem.VehicleAction.NONE,
        )

        mission_items.append(item)

        print(
            f"WP{index}: lat={lat:.7f}, "
            f"lon={lon:.7f}, altitude={altitude:.1f} m"
        )

    return MissionPlan(mission_items)
    
async def monitor_progress(drone):
    last_progress = None

    async for progress in drone.mission.mission_progress():
        current = (progress.current, progress.total)

        if current != last_progress:
            print(
                f"Mission progress: "
                f"{progress.current}/{progress.total}"
            )
            last_progress = current

        if progress.total > 0 and progress.current >= progress.total:
            print("Mission waypoints completed.")
            return


async def monitor_landing(drone):
    was_airborne = False

    async for in_air in drone.telemetry.in_air():
        if in_air:
            was_airborne = True
        elif was_airborne:
            print("Landing confirmed by telemetry.")
            return


async def main():

    config = load_config()

    connection = config["connection"]
    mission_config = config["mission"]
    safety = config["safety"]

    drone = System()

    try:
        await wait_for_connection(drone, connection)
        home_lat, home_lon = await get_initial_position(drone, safety)

        mission_plan = build_mission(
	    home_lat,
	    home_lon,
	    mission_config,
	)
        # Set this before uploading the mission.
        await drone.mission.set_return_to_launch_after_mission(
	    mission_config["return_to_launch_after_mission"]
	)
        print("[3] Uploading mission...")

        await drone.mission.upload_mission(mission_plan)

        print("Mission uploaded successfully.")

        print("[4] Arming...")

        await drone.action.arm()

        print("Arm command accepted.")

        print("[5] Starting mission...")

        await drone.mission.start_mission()

        print("Mission start command accepted.")

        progress_task = asyncio.create_task(
            monitor_progress(drone)
        )

        landing_task = asyncio.create_task(
            monitor_landing(drone)
        )

        done, pending = await asyncio.wait(
            [progress_task, landing_task],
            return_when=asyncio.FIRST_EXCEPTION,
        )

        for task in done:
            exception = task.exception()

            if exception is not None:
                raise exception

        # If mission progress finishes first, continue waiting for landing.
        if progress_task in done and not landing_task.done():
            await landing_task

        # If landing is observed first, verify mission progress as well.
        if landing_task in done and not progress_task.done():
            try:
                await asyncio.wait_for(progress_task, timeout=5)
            except asyncio.TimeoutError:
                print(
                    "WARNING: Landed, but mission completion "
                    "was not confirmed."
                )

        for task in pending:
            task.cancel()

        await asyncio.gather(*pending, return_exceptions=True)

        print("Mission test finished.")

    except asyncio.TimeoutError:
        print("ERROR: Timeout while waiting for PX4 or mission.")

    except Exception as error:
        print(f"ERROR: {type(error).__name__}: {error}")
        print(
            "Inspect PX4 and Gazebo before retrying. "
            "Do not assume the drone has landed."
        )


if __name__ == "__main__":
    asyncio.run(main())
