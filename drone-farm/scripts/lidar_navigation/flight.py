"""Talking to the drone: telemetry helpers, pose tracking, and the Offboard setpoint controller."""

import asyncio
import collections
import math
import time

from lidar_to_map import gps_to_offset
from mavsdk_grpc.offboard import PositionNedYaw

from .constants import PX4_XY_P, SETPOINT_RATE_HZ


# ----------------------------------------------------------------------
# Small helpers
# ----------------------------------------------------------------------
def angle_diff(a, b):
    """Signed smallest difference a - b in degrees, in [-180, 180).

    Handles wrap-around: angle_diff(350, 10) == -20, angle_diff(10, 350) == 20.
    """
    return (a - b + 180.0) % 360.0 - 180.0


def _wrap_180(deg):
    return (deg + 180.0) % 360.0 - 180.0


async def wait_for_health(drone, timeout):
    """Wait until PX4 has a valid global position and Home."""
    print("Waiting for valid position and Home...")

    async def check():
        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                return

    try:
        await asyncio.wait_for(check(), timeout)
    except asyncio.TimeoutError:
        raise RuntimeError(
            f"PX4 did not report a valid position and Home within {timeout:g} s."
        ) from None
    print("Position and Home are valid.")


async def set_telemetry_rates(drone, hz=20.0):
    """Ask PX4 for faster telemetry (better pose for fast flight). Optional."""
    for name in (
        "set_rate_position_velocity_ned",
        "set_rate_attitude_euler",
        "set_rate_position",
    ):
        setter = getattr(drone.telemetry, name, None)
        if setter is None:
            continue
        try:
            await setter(hz)
        except Exception as error:  # not fatal: PX4 defaults still work
            print(f"(Could not set {name[9:]} rate: {error})")


async def arm_and_takeoff(drone, altitude):
    """Send the arm and takeoff commands (does not wait for the climb)."""
    await drone.action.set_takeoff_altitude(float(altitude))
    await drone.action.arm()
    await drone.action.takeoff()


async def land_and_wait(drone, timeout=60.0):
    """Land and wait until PX4 reports the vehicle disarmed."""
    await drone.action.land()

    async def check():
        async for armed in drone.telemetry.armed():
            if not armed:
                return

    try:
        await asyncio.wait_for(check(), timeout)
    except asyncio.TimeoutError:
        raise RuntimeError(
            "Landing was not confirmed (still armed). Inspect PX4/Gazebo."
        ) from None
    print("Landed and disarmed.")


# ----------------------------------------------------------------------
# Pose tracking (with history, so scans can be placed accurately)
# ----------------------------------------------------------------------
class PoseTracker:
    """Keeps the latest position, heading and speed from PX4 in the background.

    Map frame = north/east metres from `origin` (PX4 home), the same frame as
    mission.yaml and the planner. PX4's local NED frame is normally identical
    at start-up; any small difference is measured once in wait_ready() and
    corrected, so setpoints and the map always agree.

    Attributes: heading (deg, 0 = north, 90 = east), speed (m/s, horizontal).
    pose_at(t) gives the interpolated pose at an earlier time.monotonic().
    """

    def __init__(self, drone, origin):
        self.drone = drone
        self.origin = origin
        self.heading = None
        self.roll = None
        self.pitch = None
        self.has_tilt = False
        self.speed = 0.0
        self.vel_north = 0.0
        self.vel_east = 0.0
        self.vel_down = 0.0
        self.flight_mode = "?"
        self.error = None
        self.last_update = None
        self.offset_north = 0.0
        self.offset_east = 0.0
        self._ned = None  # raw local NED (north, east, down)
        self._gps = None
        self._tasks = []
        # (time, raw north, raw east, heading, tilt) - newest last
        self.history = collections.deque(maxlen=2000)

    # ---- lifecycle ---------------------------------------------------
    def start(self):
        self._tasks = [
            asyncio.create_task(self._guard(self._ned_loop())),
            asyncio.create_task(self._guard(self._gps_loop())),
            asyncio.create_task(self._guard(self._heading_loop())),
            asyncio.create_task(self._guard(self._mode_loop())),
        ]

    async def stop(self):
        for task in self._tasks:
            task.cancel()
        await asyncio.gather(*self._tasks, return_exceptions=True)
        self._tasks = []

    async def wait_ready(self, timeout):
        deadline = time.monotonic() + timeout
        while self._ned is None or self._gps is None or self.heading is None:
            if self.error is not None:
                raise RuntimeError(f"Telemetry failed: {self.error!r}")
            if time.monotonic() > deadline:
                raise RuntimeError("Timed out waiting for position and heading.")
            await asyncio.sleep(0.05)

        # Measure how the map frame (GPS offset from origin) differs from
        # PX4's local NED frame, once, while still on the ground.
        gps_n, gps_e = gps_to_offset(
            self._gps[0], self._gps[1], self.origin[0], self.origin[1]
        )
        self.offset_north = gps_n - self._ned[0]
        self.offset_east = gps_e - self._ned[1]
        if math.hypot(self.offset_north, self.offset_east) > 0.5:
            print(
                f"Note: map frame is {self.offset_north:.2f} N / "
                f"{self.offset_east:.2f} E away from PX4's local frame "
                "(corrected)."
            )
        if not self.has_tilt:
            print("Note: no attitude stream, so tilted lidar scans cannot "
                  "be filtered out.")

    # ---- readings ----------------------------------------------------
    @property
    def north(self):
        return self._ned[0] + self.offset_north

    @property
    def east(self):
        return self._ned[1] + self.offset_east

    @property
    def down(self):
        return self._ned[2]

    def ned(self):
        return self._ned

    def pose(self):
        """(north_m, east_m, heading_deg) in the map frame."""
        return self.north, self.east, self.heading

    def map_to_ned(self, north, east):
        return north - self.offset_north, east - self.offset_east

    def pose_at(self, t):
        """(north, east, heading, tilt_deg) at time t, or None if unknown.

        Interpolated between the two nearest telemetry samples.
        """
        samples = list(self.history)
        if not samples or t < samples[0][0]:
            return None

        if t >= samples[-1][0]:
            _, n, e, h, tilt = samples[-1]
        else:
            for i in range(len(samples) - 1, 0, -1):
                a, b = samples[i - 1], samples[i]
                if a[0] <= t <= b[0]:
                    span = b[0] - a[0]
                    f = (t - a[0]) / span if span > 1e-6 else 1.0
                    n = a[1] + f * (b[1] - a[1])
                    e = a[2] + f * (b[2] - a[2])
                    h = (a[3] + f * angle_diff(b[3], a[3])) % 360.0
                    tilt = max(a[4], b[4])
                    break
            else:
                return None
        return n + self.offset_north, e + self.offset_east, h, tilt

    def check(self):
        """Raise if telemetry died or stopped updating."""
        if self.error is not None:
            raise RuntimeError(f"Telemetry failed: {self.error!r}")
        if self.last_update is None or time.monotonic() - self.last_update > 2.0:
            raise RuntimeError("Position telemetry stopped updating.")

    # ---- background loops --------------------------------------------
    async def _guard(self, coro):
        try:
            await coro
        except asyncio.CancelledError:
            raise
        except Exception as error:  # remembered, raised by check()
            self.error = error

    def _record(self):
        if self._ned is None or self.heading is None:
            return
        if self.roll is not None:
            tilt = max(abs(self.roll), abs(self.pitch))
        else:
            tilt = 0.0
        self.history.append(
            (time.monotonic(), self._ned[0], self._ned[1], self.heading, tilt)
        )

    async def _ned_loop(self):
        async for pv in self.drone.telemetry.position_velocity_ned():
            self._ned = (
                pv.position.north_m,
                pv.position.east_m,
                pv.position.down_m,
            )
            self.vel_north = pv.velocity.north_m_s
            self.vel_east = pv.velocity.east_m_s
            self.vel_down = pv.velocity.down_m_s
            self.speed = math.hypot(self.vel_north, self.vel_east)
            self.last_update = time.monotonic()
            self._record()

    async def _mode_loop(self):
        """Remember PX4's flight mode (to see if Offboard is ever dropped)."""
        try:
            stream = self.drone.telemetry.flight_mode()
        except AttributeError:
            return
        async for mode in stream:
            self.flight_mode = str(mode).split(".")[-1]

    async def _gps_loop(self):
        async for position in self.drone.telemetry.position():
            if math.isfinite(position.latitude_deg) and math.isfinite(
                position.longitude_deg
            ):
                self._gps = (position.latitude_deg, position.longitude_deg)

    async def _heading_loop(self):
        # Prefer the attitude stream: higher rate, and it gives roll/pitch.
        try:
            stream = self.drone.telemetry.attitude_euler()
        except AttributeError:
            stream = None

        if stream is not None:
            self.has_tilt = True
            async for euler in stream:
                if math.isfinite(euler.yaw_deg):
                    self.roll = euler.roll_deg
                    self.pitch = euler.pitch_deg
                    self.heading = euler.yaw_deg % 360.0
                    self._record()
        else:
            async for heading in self.drone.telemetry.heading():
                if math.isfinite(heading.heading_deg):
                    self.heading = heading.heading_deg % 360.0
                    self._record()


# ----------------------------------------------------------------------
# Offboard control
# ----------------------------------------------------------------------
class SetpointShaper:
    """Keeps the setpoint sent to PX4 from jumping.

    In position mode PX4 flies at a speed proportional to the "lead": how far
    the setpoint is from the drone. The navigation logic changes its goal in
    steps (the lead collapses to zero to stop, flips to the other side at a
    reversal, the yaw target jumps by up to 180 degrees at a corner), and PX4
    answers each step with its hardest possible acceleration or spin. Hard
    yaw spins and hard braking cost the motors thrust headroom, and the drone
    sinks.

    This limits how fast the yaw setpoint may turn and, if max_accel is
    given, how fast the lead may change (which limits acceleration to about
    max_accel). With max_accel=None only the yaw is limited and positions
    pass through unchanged. When the goal is steady the output equals the
    goal, so cruising is unchanged.

    Note: limiting acceleration makes the drone stop later (about
    v^2 / (2 * max_accel) metres), which can be more than the planner's
    safety margin. Account for that before using it.
    """

    def __init__(self, max_accel, max_yaw_rate):
        # m/s: how fast the lead may change (None = no limit)
        self.lead_rate = None if max_accel is None else max_accel / PX4_XY_P
        self.max_yaw_rate = max_yaw_rate
        self.lead_north = 0.0
        self.lead_east = 0.0
        self.yaw = None

    def reset(self, yaw):
        self.lead_north = self.lead_east = 0.0
        self.yaw = yaw % 360.0

    def step(self, goal_lead_north, goal_lead_east, goal_yaw, dt):
        """Move towards the goal; returns (lead_north, lead_east, yaw_deg)."""
        if self.yaw is None:
            self.yaw = goal_yaw % 360.0

        dn = goal_lead_north - self.lead_north
        de = goal_lead_east - self.lead_east
        dist = math.hypot(dn, de)
        limit = None if self.lead_rate is None else self.lead_rate * dt
        if limit is None or dist <= limit:
            self.lead_north, self.lead_east = goal_lead_north, goal_lead_east
        else:
            self.lead_north += dn / dist * limit
            self.lead_east += de / dist * limit

        turn = angle_diff(goal_yaw, self.yaw)
        max_turn = self.max_yaw_rate * dt
        self.yaw = (self.yaw + max(-max_turn, min(max_turn, turn))) % 360.0
        return self.lead_north, self.lead_east, self.yaw


class OffboardController:
    """Streams the current position+yaw setpoint to PX4 at 20 Hz.

    Callers only set the goal (set_target_map / hold); the stream never
    pauses, even while the program is planning (PX4 drops Offboard if it
    stops). With a shaper, the setpoint actually sent follows the goal
    smoothly instead of jumping to it.
    """

    def __init__(self, drone, shaper=None):
        self.drone = drone
        self.shaper = shaper
        self.down = 0.0
        self.error = None
        self._goal = None    # what the navigation logic asked for
        self._target = None  # what is sent: (north_ned, east_ned, down_ned, yaw_deg)
        self._tracker = None
        self._task = None
        self._active = False
        self.max_gap = 0.0       # longest pause between setpoints (diagnostic)
        self._last_send = None

    async def _send(self):
        now = time.monotonic()
        if self._last_send is not None:
            self.max_gap = max(self.max_gap, now - self._last_send)
        self._last_send = now
        n, e, d, yaw = self._target
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(n, e, d, float(_wrap_180(yaw)))
        )

    async def start(self, tracker, down=None, warmup=10):
        """down: NED 'down' value to hold (default: where the drone is now)."""
        n, e, d = tracker.ned()
        self.down = d if down is None else down
        self._tracker = tracker
        self._goal = self._target = (n, e, self.down, tracker.heading)
        if self.shaper is not None:
            self.shaper.reset(tracker.heading)

        # PX4 requires a setpoint stream before Offboard can start.
        for _ in range(warmup):
            await self._send()
            await asyncio.sleep(1.0 / SETPOINT_RATE_HZ)

        await self.drone.offboard.start()
        self._active = True
        self._task = asyncio.create_task(self._run())
        print("Offboard started.")

    def _shape(self, dt):
        """Update the setpoint to send: the goal, smoothed."""
        if self.shaper is None or self._tracker is None:
            self._target = self._goal
            return
        goal_n, goal_e, goal_d, goal_yaw = self._goal
        n, e, _ = self._tracker.ned()
        lead_n, lead_e, yaw = self.shaper.step(goal_n - n, goal_e - e, goal_yaw, dt)
        self._target = (n + lead_n, e + lead_e, goal_d, yaw)

    async def _run(self):
        try:
            last = time.monotonic()
            while True:
                now = time.monotonic()
                self._shape(min(0.25, now - last))
                last = now
                await self._send()
                await asyncio.sleep(1.0 / SETPOINT_RATE_HZ)
        except asyncio.CancelledError:
            raise
        except Exception as error:
            self.error = error

    async def stop(self):
        if self._task is not None:
            self._task.cancel()
            await asyncio.gather(self._task, return_exceptions=True)
            self._task = None
        if self._active:
            self._active = False
            try:
                await self.drone.offboard.stop()
                print("Offboard stopped.")
            except Exception as error:
                print(f"(Offboard stop failed: {error})")

    def check(self):
        if self.error is not None:
            raise RuntimeError(f"Offboard setpoint stream failed: {self.error!r}")

    def set_target_map(self, tracker, north, east, yaw):
        """Fly towards a map-frame (north, east) at the hold altitude."""
        n, e = tracker.map_to_ned(north, east)
        self._goal = (n, e, self.down, yaw)

    def hold(self, tracker, yaw=None):
        """Stay where the drone is (facing `yaw`, default: as it is now)."""
        n, e, _ = tracker.ned()
        self._goal = (n, e, self.down, tracker.heading if yaw is None else yaw)
