
"""Safety validation helpers for Decision Module A."""

import math
from typing import Any, Dict


def get_battery_threshold(params: Dict[str, Any]) -> float:
    value = params.get("battery_return_threshold_percent", 25.0)

    try:
        threshold = float(value)
    except (TypeError, ValueError):
        return 25.0

    if not math.isfinite(threshold):
        return 25.0

    return min(100.0, max(0.0, threshold))


def battery_is_low(
    drone: Dict[str, Any],
    params: Dict[str, Any]
) -> bool:
    value = drone.get("battery_percent")

    if value is None:
        return False

    try:
        battery = float(value)
    except (TypeError, ValueError):
        return False

    if not math.isfinite(battery):
        return False

    if not 0.0 <= battery <= 100.0:
        return False

    return battery <= get_battery_threshold(params)


def has_valid_position(
    drone: Dict[str, Any],
    navigation: Dict[str, Any]
) -> bool:
    return (
        drone.get("position_valid") is True
        and navigation.get("gps_valid") is True
    )


def validate_waypoint(waypoint: Any) -> bool:
    if not isinstance(waypoint, dict):
        return False

    required = ("north_m", "east_m", "down_m")

    try:
        for key in required:
            value = float(waypoint[key])

            if not math.isfinite(value):
                return False

        return True

    except (KeyError, TypeError, ValueError):
        return False

