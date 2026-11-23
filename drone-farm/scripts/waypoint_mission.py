import asyncio
import math

from mavsdk_grpc import System
from mavsdk_grpc.mission import MissionItem, MissionPlan

from pathlib import Path

import yaml


PROJECT_ROOT = Path(__file__).resolve().parent.parent
CONFIG_PATH = PROJECT_ROOT / "config" / "mission.yaml"

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
    
# PHONG_WAYPOINT_TIMEOUT_AND_LANDING
async def first_matching(stream, predicate):
    async for sample in stream:
        if predicate(sample):
            return sample
    raise RuntimeError("Telemetry stream ended before the expected state.")


async def monitor_progress(drone):
    last_progress = None
    async for progress in drone.mission.mission_progress():
        current = (progress.current, progress.total)
        if current != last_progress:
            print(f"MISSION_PROGRESS={progress.current}/{progress.total}")
            last_progress = current
        if progress.total > 0 and progress.current >= progress.total:
            return
    raise RuntimeError("Mission progress stream ended early.")


async def monitor_landing(drone):
    await first_matching(drone.telemetry.in_air(), bool)
    print("AIRBORNE_CONFIRMED=1")
    await first_matching(drone.telemetry.in_air(), lambda in_air: not in_air)
    print("LANDING_CONFIRMED=1")
    await first_matching(drone.telemetry.armed(), lambda armed: not armed)
    print("DISARM_CONFIRMED=1")


def horizontal_distance_m(home, position):
    radius = 6378137.0
    dlat = math.radians(position.latitude_deg - home.latitude_deg)
    dlon = math.radians(position.longitude_deg - home.longitude_deg)
    cos_lat = math.cos(math.radians(home.latitude_deg))
    return radius * math.hypot(dlat, dlon * cos_lat)


async def run_mission(drone, mission_config, plan, home):
    await drone.mission.set_return_to_launch_after_mission(
        mission_config["return_to_launch_after_mission"]
    )
    await drone.mission.upload_mission(plan)
    print("MISSION_UPLOADED=1")

    async with asyncio.TaskGroup() as tasks:
        tasks.create_task(monitor_progress(drone))
        tasks.create_task(monitor_landing(drone))
        await drone.action.arm()
        print("ARM_ACCEPTED=1")
        await drone.mission.start_mission()
        print("MISSION_STARTED=1")

    position = await first_matching(
        drone.telemetry.position(),
        lambda sample: all(map(math.isfinite, (sample.latitude_deg, sample.longitude_deg))),
    )
    error = horizontal_distance_m(home, position)
    print(f"HOME_ERROR_M={error:.3f}")
    if mission_config["return_to_launch_after_mission"] and error > 2.0:
        raise RuntimeError(f"Landing exceeded the 2.0 m home tolerance: {error:.3f} m.")


async def main():
    config = load_config()
    connection = config["connection"]
    mission = config["mission"]
    safety = config["safety"]

    drone = System()
    await asyncio.wait_for(
        wait_for_connection(drone, connection),
        timeout=connection["timeout_seconds"],
    )
    lat, lon = await asyncio.wait_for(
        get_initial_position(drone, safety),
        timeout=safety["global_position_timeout_seconds"],
    )
    home = await asyncio.wait_for(
        first_matching(
            drone.telemetry.home(),
            lambda sample: all(map(math.isfinite, (sample.latitude_deg, sample.longitude_deg))),
        ),
        timeout=10,
    )
    print(f"HOME_LAT_LON={home.latitude_deg:.7f},{home.longitude_deg:.7f}")
    plan = build_mission(lat, lon, mission)
    await asyncio.wait_for(
        run_mission(drone, mission, plan, home),
        timeout=safety["mission_timeout_seconds"],
    )
    print("WAYPOINT_MISSION=PASS")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except Exception as error:
        import traceback

        traceback.print_exception(error)
        print("WAYPOINT_MISSION=FAIL", flush=True)
        raise SystemExit(1) from error
