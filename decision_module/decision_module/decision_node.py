
"""ROS 2 node for Decision Module A. JSON uses std_msgs/String."""

import json
import time
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .decision_engine import DecisionEngine
from .decision_types import Decision, HOLD_POSITION


class DecisionNode(Node):

    def __init__(self):
        super().__init__("decision_node")

        self.declare_parameter(
            "battery_return_threshold_percent", 25.0
        )
        self.declare_parameter("input_timeout_sec", 3.0)
        self.declare_parameter("decision_rate_hz", 2.0)

        battery_threshold = float(
            self.get_parameter(
                "battery_return_threshold_percent"
            ).value
        )

        timeout = float(
            self.get_parameter("input_timeout_sec").value
        )

        rate = float(
            self.get_parameter("decision_rate_hz").value
        )

        if not 0.0 <= battery_threshold <= 100.0:
            raise ValueError("Battery threshold must be 0-100.")

        if timeout <= 0.0:
            raise ValueError("Input timeout must be positive.")

        if rate <= 0.0:
            raise ValueError("Decision rate must be positive.")

        self.input_timeout = timeout

        self.engine = DecisionEngine({
            "battery_return_threshold_percent": battery_threshold
        })

        self.data: Dict[str, Optional[Dict[str, Any]]] = {
            "drone": None,
            "navigation": None,
            "environment": None,
            "motion": None,
        }

        self.received_at = {
            key: None for key in self.data
        }

        self.decision_id = 0
        self.last_signature = None

        # Input subscriptions.
        self.create_subscription(
            String,
            "/drone/status",
            lambda msg: self._receive("drone", msg),
            10
        )

        self.create_subscription(
            String,
            "/navigation/status",
            lambda msg: self._receive("navigation", msg),
            10
        )

        self.create_subscription(
            String,
            "/environment/status",
            lambda msg: self._receive("environment", msg),
            10
        )

        self.create_subscription(
            String,
            "/motion/status",
            lambda msg: self._receive("motion", msg),
            10
        )

        # Output publishers.
        self.command_pub = self.create_publisher(
            String, "/decision/command", 10
        )

        self.replan_pub = self.create_publisher(
            String, "/navigation/replan_request", 10
        )

        self.status_pub = self.create_publisher(
            String, "/decision/status", 10
        )

        self.timer = self.create_timer(
            1.0 / rate,
            self._evaluate
        )

        self.get_logger().info("Decision Module A started.")

        self.get_logger().info(
            "Inputs: /drone/status, /navigation/status, "
            "/environment/status, /motion/status"
        )

        self.get_logger().info(
            "Outputs: /decision/command, "
            "/navigation/replan_request, /decision/status"
        )

    def _receive(self, key: str, msg: String) -> None:
        try:
            parsed = json.loads(msg.data)

            if not isinstance(parsed, dict):
                raise ValueError("JSON root must be an object.")

            self.data[key] = parsed
            self.received_at[key] = time.monotonic()

        except (json.JSONDecodeError, ValueError) as exc:
            self.get_logger().error(
                f"Invalid JSON on /{key}/status: {exc}"
            )

    def _is_fresh(self, key: str) -> bool:
        received = self.received_at.get(key)

        return (
            received is not None
            and time.monotonic() - received <= self.input_timeout
        )

    def _publish_json(self, publisher, payload: Dict[str, Any]):
        msg = String()
        msg.data = json.dumps(
            payload,
            ensure_ascii=False,
            allow_nan=False
        )
        publisher.publish(msg)

    def _evaluate(self) -> None:

        required = (
            "drone",
            "navigation",
            "environment",
        )

        stale = [
            key for key in required
            if not self._is_fresh(key)
        ]

        # Missing or stale required input.
        if stale:

            # Without a fresh drone status, do not create
            # a new decision based on unknown drone state.
            if not self._is_fresh("drone"):
                return

            decision = Decision(
                HOLD_POSITION,
                "HIGH",
                "Missing or stale input: " + ", ".join(stale)
            )

        else:
            drone = self.data["drone"] or {}
            navigation = self.data["navigation"] or {}
            environment = self.data["environment"] or {}
            motion = self.data["motion"] or {}

            decision = self.engine.decide(
                drone,
                navigation,
                environment,
                motion
            )

        signature = (
            decision.action,
            decision.priority,
            decision.reason,
            json.dumps(
                decision.target_waypoint,
                sort_keys=True,
                allow_nan=False
            ),
            decision.request_replan,
        )

        # Publish command and status periodically so that
        # late subscribers can still receive the current state.
        self.decision_id += 1

        payload = decision.to_dict(self.decision_id)

        self._publish_json(
            self.command_pub,
            payload
        )

        self._publish_json(
            self.status_pub,
            payload
        )

        # Publish replan request only when the decision changes.
        if (
            decision.request_replan
            and signature != self.last_signature
        ):
            navigation = self.data["navigation"] or {}

            request = {
                "request_id": self.decision_id,
                "reason": decision.reason,
                "current_position": navigation.get(
                    "current_position"
                ),
                "frame_id": navigation.get(
                    "frame_id",
                    "UNSPECIFIED"
                ),
            }

            self._publish_json(
                self.replan_pub,
                request
            )

        if signature != self.last_signature:
            self.get_logger().info(
                f"Decision #{self.decision_id}: "
                f"{decision.action} ({decision.priority})"
            )

        self.last_signature = signature


def main(args=None):
    rclpy.init(args=args)

    node = DecisionNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

"""ROS 2 node for Decision Module A. JSON uses std_msgs/String."""

import json
import time
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .decision_engine import DecisionEngine
from .decision_types import Decision, HOLD_POSITION


class DecisionNode(Node):

    def __init__(self):
        super().__init__("decision_node")

        self.declare_parameter(
            "battery_return_threshold_percent", 25.0
        )
        self.declare_parameter("input_timeout_sec", 3.0)
        self.declare_parameter("decision_rate_hz", 2.0)

        battery_threshold = float(
            self.get_parameter(
                "battery_return_threshold_percent"
            ).value
        )

        timeout = float(
            self.get_parameter("input_timeout_sec").value
        )

        rate = float(
            self.get_parameter("decision_rate_hz").value
        )

        if not 0.0 <= battery_threshold <= 100.0:
            raise ValueError("Battery threshold must be 0-100.")

        if timeout <= 0.0:
            raise ValueError("Input timeout must be positive.")

        if rate <= 0.0:
            raise ValueError("Decision rate must be positive.")

        self.input_timeout = timeout

        self.engine = DecisionEngine({
            "battery_return_threshold_percent": battery_threshold
        })

        self.data: Dict[str, Optional[Dict[str, Any]]] = {
            "drone": None,
            "navigation": None,
            "environment": None,
            "motion": None,
        }

        self.received_at = {
            key: None for key in self.data
        }

        self.decision_id = 0
        self.last_signature = None

        # Input subscriptions.
        self.create_subscription(
            String,
            "/drone/status",
            lambda msg: self._receive("drone", msg),
            10
        )

        self.create_subscription(
            String,
            "/navigation/status",
            lambda msg: self._receive("navigation", msg),
            10
        )

        self.create_subscription(
            String,
            "/environment/status",
            lambda msg: self._receive("environment", msg),
            10
        )

        self.create_subscription(
            String,
            "/motion/status",
            lambda msg: self._receive("motion", msg),
            10
        )

        # Output publishers.
        self.command_pub = self.create_publisher(
            String, "/decision/command", 10
        )

        self.replan_pub = self.create_publisher(
            String, "/navigation/replan_request", 10
        )

        self.status_pub = self.create_publisher(
            String, "/decision/status", 10
        )

        self.timer = self.create_timer(
            1.0 / rate,
            self._evaluate
        )

        self.get_logger().info("Decision Module A started.")

        self.get_logger().info(
            "Inputs: /drone/status, /navigation/status, "
            "/environment/status, /motion/status"
        )

        self.get_logger().info(
            "Outputs: /decision/command, "
            "/navigation/replan_request, /decision/status"
        )

    def _receive(self, key: str, msg: String) -> None:
        try:
            parsed = json.loads(msg.data)

            if not isinstance(parsed, dict):
                raise ValueError("JSON root must be an object.")

            self.data[key] = parsed
            self.received_at[key] = time.monotonic()

        except (json.JSONDecodeError, ValueError) as exc:
            self.get_logger().error(
                f"Invalid JSON on /{key}/status: {exc}"
            )

    def _is_fresh(self, key: str) -> bool:
        received = self.received_at.get(key)

        return (
            received is not None
            and time.monotonic() - received <= self.input_timeout
        )

    def _publish_json(self, publisher, payload: Dict[str, Any]):
        msg = String()
        msg.data = json.dumps(
            payload,
            ensure_ascii=False,
            allow_nan=False
        )
        publisher.publish(msg)

    def _evaluate(self) -> None:

        required = (
            "drone",
            "navigation",
            "environment",
        )

        stale = [
            key for key in required
            if not self._is_fresh(key)
        ]

        # Missing or stale required input.
        if stale:

            # Without a fresh drone status, do not create
            # a new decision based on unknown drone state.
            if not self._is_fresh("drone"):
                return

            decision = Decision(
                HOLD_POSITION,
                "HIGH",
                "Missing or stale input: " + ", ".join(stale)
            )

        else:
            drone = self.data["drone"] or {}
            navigation = self.data["navigation"] or {}
            environment = self.data["environment"] or {}
            motion = self.data["motion"] or {}

            decision = self.engine.decide(
                drone,
                navigation,
                environment,
                motion
            )

        signature = (
            decision.action,
            decision.priority,
            decision.reason,
            json.dumps(
                decision.target_waypoint,
                sort_keys=True,
                allow_nan=False
            ),
            decision.request_replan,
        )

        # Publish command and status periodically so that
        # late subscribers can still receive the current state.
        self.decision_id += 1

        payload = decision.to_dict(self.decision_id)

        self._publish_json(
            self.command_pub,
            payload
        )

        self._publish_json(
            self.status_pub,
            payload
        )

        # Publish replan request only when the decision changes.
        if (
            decision.request_replan
            and signature != self.last_signature
        ):
            navigation = self.data["navigation"] or {}

            request = {
                "request_id": self.decision_id,
                "reason": decision.reason,
                "current_position": navigation.get(
                    "current_position"
                ),
                "frame_id": navigation.get(
                    "frame_id",
                    "UNSPECIFIED"
                ),
            }

            self._publish_json(
                self.replan_pub,
                request
            )

        if signature != self.last_signature:
            self.get_logger().info(
                f"Decision #{self.decision_id}: "
                f"{decision.action} ({decision.priority})"
            )

        self.last_signature = signature


def main(args=None):
    rclpy.init(args=args)

    node = DecisionNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

