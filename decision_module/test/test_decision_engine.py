import unittest

from decision_module.decision_engine import DecisionEngine


class TestDecisionEngine(unittest.TestCase):

    def setUp(self):
        self.engine = DecisionEngine({
            "battery_return_threshold_percent": 25.0
        })

        self.drone = {
            "connected": True,
            "armed": True,
            "battery_percent": 80.0,
            "position_valid": True,
            "failsafe_active": False,
        }

        self.navigation = {
            "gps_valid": True,
            "path_valid": True,
            "replan_required": False,
            "waypoint_reached": False,
            "next_waypoint": {
                "north_m": 10.0,
                "east_m": 8.0,
                "down_m": -3.0
            },
        }

        self.environment = {
            "obstacle_detected": False,
            "weather_safe": True,
        }

    def test_normal_flight(self):
        result = self.engine.decide(
            self.drone,
            self.navigation,
            self.environment
        )

        self.assertEqual(
            result.action,
            "CONTINUE_TO_WAYPOINT"
        )

    def test_low_battery(self):
        drone = dict(
            self.drone,
            battery_percent=15.0
        )

        result = self.engine.decide(
            drone,
            self.navigation,
            self.environment
        )

        self.assertEqual(
            result.action,
            "RETURN_TO_LAUNCH"
        )

    def test_invalid_gps(self):
        navigation = dict(
            self.navigation,
            gps_valid=False
        )

        result = self.engine.decide(
            self.drone,
            navigation,
            self.environment
        )

        self.assertEqual(
            result.action,
            "HOLD_POSITION"
        )

    def test_obstacle_requests_replan(self):
        environment = dict(
            self.environment,
            obstacle_detected=True
        )

        result = self.engine.decide(
            self.drone,
            self.navigation,
            environment
        )

        self.assertEqual(
            result.action,
            "HOLD_POSITION"
        )

        self.assertTrue(result.request_replan)

    def test_waypoint_reached(self):
        navigation = dict(
            self.navigation,
            waypoint_reached=True
        )

        result = self.engine.decide(
            self.drone,
            navigation,
            self.environment
        )

        self.assertEqual(
            result.action,
            "MOVE_TO_NEXT_WAYPOINT"
        )

    def test_failsafe(self):
        drone = dict(
            self.drone,
            failsafe_active=True
        )

        result = self.engine.decide(
            drone,
            self.navigation,
            self.environment
        )

        self.assertEqual(
            result.action,
            "EMERGENCY"
        )


if __name__ == "__main__":
    unittest.main()
