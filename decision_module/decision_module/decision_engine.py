
"""Pure decision logic for Decision Module A."""

from typing import Any, Dict, Optional

from .decision_types import (
    Decision,
    CONTINUE_TO_WAYPOINT,
    MOVE_TO_NEXT_WAYPOINT,
    HOLD_POSITION,
    RETURN_TO_LAUNCH,
    LAND,
    EMERGENCY,
)

from .safety_manager import (
    battery_is_low,
    has_valid_position,
    validate_waypoint,
)


class DecisionEngine:

    def __init__(self, params: Optional[Dict[str, Any]] = None):
        self.params = params or {}

    def decide(
        self,
        drone: Dict[str, Any],
        navigation: Dict[str, Any],
        environment: Dict[str, Any],
        motion: Optional[Dict[str, Any]] = None,
    ) -> Decision:

        motion = motion or {}

        # 1. Active failsafe.
        if drone.get("failsafe_active") is True:
            return Decision(
                EMERGENCY,
                "CRITICAL",
                "PX4 reports an active failsafe."
            )

        # 2. Connection check.
        if drone.get("connected") is not True:
            return Decision(
                HOLD_POSITION,
                "CRITICAL",
                "Drone connection is not confirmed."
            )

        # 3. Position validation.
        position_ok = has_valid_position(drone, navigation)

        # 4. Battery check.
        if battery_is_low(drone, self.params):

            if position_ok:
                return Decision(
                    RETURN_TO_LAUNCH,
                    "HIGH",
                    "Battery is below the configured threshold."
                )

            return Decision(
                LAND,
                "CRITICAL",
                "Battery is low and position is invalid. "
                "Apply the approved safety policy."
            )

        # 5. GPS and position check.
        if not position_ok:
            return Decision(
                HOLD_POSITION,
                "HIGH",
                "GPS or drone position is invalid."
            )

        # 6. Weather check.
        if environment.get("weather_safe") is False:
            return Decision(
                RETURN_TO_LAUNCH,
                "HIGH",
                "Weather conditions are unsafe."
            )

        # 7. Motion execution check.
        if str(motion.get("state", "")).upper() == "FAILED":
            return Decision(
                HOLD_POSITION,
                "HIGH",
                "Motion module reported execution failure."
            )

        # 8. Obstacle and route check.
        obstacle_detected = (
            environment.get("obstacle_detected") is True
        )

        replan_required = (
            navigation.get("replan_required") is True
        )

        path_valid = (
            navigation.get("path_valid") is True
        )

        if obstacle_detected or replan_required or not path_valid:
            return Decision(
                HOLD_POSITION,
                "HIGH",
                "Route may be blocked. Request Module C to replan.",
                request_replan=True,
            )

        # 9. Waypoint validation.
        waypoint = navigation.get("next_waypoint")

        if not validate_waypoint(waypoint):
            return Decision(
                HOLD_POSITION,
                "HIGH",
                "Navigation module has not supplied a valid waypoint."
            )

        # 10. Waypoint reached.
        if navigation.get("waypoint_reached") is True:
            return Decision(
                MOVE_TO_NEXT_WAYPOINT,
                "NORMAL",
                "Current waypoint reached.",
                target_waypoint=waypoint,
            )

        # 11. Normal decision.
        return Decision(
            CONTINUE_TO_WAYPOINT,
            "NORMAL",
            "Navigation is valid. Continue to waypoint.",
            target_waypoint=waypoint,
        )

"""Pure decision logic for Decision Module A."""

from typing import Any, Dict, Optional

from .decision_types import (
    Decision,
    CONTINUE_TO_WAYPOINT,
    MOVE_TO_NEXT_WAYPOINT,
    HOLD_POSITION,
    RETURN_TO_LAUNCH,
    LAND,
    EMERGENCY,
)

from .safety_manager import (
    battery_is_low,
    has_valid_position,
    validate_waypoint,
)


class DecisionEngine:

    def __init__(self, params: Optional[Dict[str, Any]] = None):
        self.params = params or {}

    def decide(
        self,
        drone: Dict[str, Any],
        navigation: Dict[str, Any],
        environment: Dict[str, Any],
        motion: Optional[Dict[str, Any]] = None,
    ) -> Decision:

        motion = motion or {}

        # 1. Active failsafe.
        if drone.get("failsafe_active") is True:
            return Decision(
                EMERGENCY,
                "CRITICAL",
                "PX4 reports an active failsafe."
            )

        # 2. Connection check.
        if drone.get("connected") is not True:
            return Decision(
                HOLD_POSITION,
                "CRITICAL",
                "Drone connection is not confirmed."
            )

        # 3. Position validation.
        position_ok = has_valid_position(drone, navigation)

        # 4. Battery check.
        if battery_is_low(drone, self.params):

            if position_ok:
                return Decision(
                    RETURN_TO_LAUNCH,
                    "HIGH",
                    "Battery is below the configured threshold."
                )

            return Decision(
                LAND,
                "CRITICAL",
                "Battery is low and position is invalid. "
                "Apply the approved safety policy."
            )

        # 5. GPS and position check.
        if not position_ok:
            return Decision(
                HOLD_POSITION,
                "HIGH",
                "GPS or drone position is invalid."
            )

        # 6. Weather check.
        if environment.get("weather_safe") is False:
            return Decision(
                RETURN_TO_LAUNCH,
                "HIGH",
                "Weather conditions are unsafe."
            )

        # 7. Motion execution check.
        if str(motion.get("state", "")).upper() == "FAILED":
            return Decision(
                HOLD_POSITION,
                "HIGH",
                "Motion module reported execution failure."
            )

        # 8. Obstacle and route check.
        obstacle_detected = (
            environment.get("obstacle_detected") is True
        )

        replan_required = (
            navigation.get("replan_required") is True
        )

        path_valid = (
            navigation.get("path_valid") is True
        )

        if obstacle_detected or replan_required or not path_valid:
            return Decision(
                HOLD_POSITION,
                "HIGH",
                "Route may be blocked. Request Module C to replan.",
                request_replan=True,
            )

        # 9. Waypoint validation.
        waypoint = navigation.get("next_waypoint")

        if not validate_waypoint(waypoint):
            return Decision(
                HOLD_POSITION,
                "HIGH",
                "Navigation module has not supplied a valid waypoint."
            )

        # 10. Waypoint reached.
        if navigation.get("waypoint_reached") is True:
            return Decision(
                MOVE_TO_NEXT_WAYPOINT,
                "NORMAL",
                "Current waypoint reached.",
                target_waypoint=waypoint,
            )

        # 11. Normal decision.
        return Decision(
            CONTINUE_TO_WAYPOINT,
            "NORMAL",
            "Navigation is valid. Continue to waypoint.",
            target_waypoint=waypoint,
        )

