"""Accept one return-to-launch request for a single flight execution."""

import asyncio
import json


class RtlReceiver:
    def __init__(self):
        self.decision_id = None
        self.calls = 0

    def accept(self, message):
        """Record the first valid RTL request. Return whether it was accepted."""
        try:
            payload = json.loads(message)
        except (TypeError, ValueError):
            return False
        if not isinstance(payload, dict) or payload.get("action") != "RETURN_TO_LAUNCH":
            return False
        decision_id = payload.get("decision_id")
        if type(decision_id) is not int or decision_id < 0 or self.decision_id is not None:
            return False
        self.decision_id = decision_id
        return True

    async def dispatch(self, action, timeout=8):
        """Call PX4 once. Keep the latch set even if the response times out."""
        if self.decision_id is None or self.calls:
            return False
        self.calls += 1
        await asyncio.wait_for(action.return_to_launch(), timeout)
        return True
