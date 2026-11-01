
"""Data models and action names for Decision Module A."""

from dataclasses import dataclass
from typing import Any, Dict, Optional


CONTINUE_TO_WAYPOINT = "CONTINUE_TO_WAYPOINT"
MOVE_TO_NEXT_WAYPOINT = "MOVE_TO_NEXT_WAYPOINT"
HOLD_POSITION = "HOLD_POSITION"
RETURN_TO_LAUNCH = "RETURN_TO_LAUNCH"
LAND = "LAND"
EMERGENCY = "EMERGENCY"


@dataclass
class Decision:
    action: str
    priority: str
    reason: str
    target_waypoint: Optional[Dict[str, float]] = None
    request_replan: bool = False

    def to_dict(self, decision_id: int) -> Dict[str, Any]:
        return {
            "decision_id": decision_id,
            "action": self.action,
            "priority": self.priority,
            "target_waypoint": self.target_waypoint,
            "reason": self.reason,
            "request_replan": self.request_replan,
        }

