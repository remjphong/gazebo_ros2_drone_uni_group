"""Stage 4 (fast version): fly to targets using ONLY what the lidar sees,
without stopping to scan.

How it works now (compared with the first stop-and-go version):

  * LidarStream keeps ONE `gz topic -e` process running and always holds the
    newest scan, so reading a scan costs nothing (no new process per scan).
  * PoseTracker remembers the last ~20 s of poses. Every scan is placed on
    the map using the drone's pose AT THE MOMENT OF THE SCAN, so the drone
    can keep flying while it maps.
  * Mapper (background task) adds scans to the occupancy grid ~5 times a
    second. Scans taken while the drone is tilted are skipped: a tilted 2D
    lidar sees the floor as a fake wall.
  * A planner job runs about every 0.3 s. It keeps the current path while it
    is still collision-free (no jitter) and only replans when it is not.
  * The controller (10 Hz) follows the path with a "carrot" setpoint a few
    metres ahead, as far as the lidar has CONFIRMED the space to be free.
    In open space it flies at full speed; where the map is still unknown
    (behind a corner) it slows down and waits for the map to fill in.
  * Yaw is changed while flying; the drone only stops to turn for big turns.
  * Between targets it does not stop: it switches to the next target when
    within --corner-radius of the current one.
  * --explore needs no targets. The drone finds the edges of what it has
    seen (frontiers), flies to the nearest reachable one, and repeats until
    nothing reachable is unexplored, then returns to the start. Combined
    with the saved map, a second run only visits what is still unknown.
  * The map is saved to online_map.json at the end of every run (also when
    the flight fails) and loaded at the start of the next one, so repeat
    runs already know the obstacles and can fly the aisles at full speed.
    It is ignored if the start point or grid settings changed. Use
    --no-load-map for a fresh start, --no-save-map to leave the file alone.
  * A timing summary at the end shows where the time went.

Also provides the helpers yaw_test.py imports:
    PoseTracker, angle_diff, arm_and_takeoff, land_and_wait, wait_for_health

Run with the simulator up (gz_x500_lidar_2d) and no other MAVSDK script:
    python3 scripts/online_navigation.py --target 8,8
    python3 scripts/online_navigation.py            # targets from mission.yaml
"""

import argparse
import asyncio
import collections
import csv
import json
import math
import os
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path

from mavsdk_grpc.offboard import PositionNedYaw

from lidar_scan import LidarError, find_scan_topic, parse_scan
from lidar_to_map import connect, gps_to_offset, read_origin
from occupancy_grid import OCC_THRESHOLD, OccupancyGrid, render_ascii
from path_planning import Point2D
from plan_from_lidar import parse_target, scan_to_rays
from plan_route import (
    DEFAULT_CONFIG,
    ConfigError,
    build_planner,
    load_point,
    load_yaml,
)

SETPOINT_RATE_HZ = 10.0
CONTROL_PERIOD_S = 0.1
# PX4's horizontal position gain (MPC_XY_P, default 0.95 1/s): in position
# mode it flies at about this many m/s per metre of distance to the setpoint.
PX4_XY_P = 0.95
MAP_FILE_VERSION = 1
DEFAULT_MAP_FILE = Path(__file__).resolve().with_name("online_map.json")


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
# Lidar stream (one long-running `gz topic -e`, newest scan always ready)
# ----------------------------------------------------------------------
class LidarStream:
    def __init__(self, topic):
        self.topic = topic
        self.proc = None
        self.thread = None
        self.latest = None  # (arrival time.monotonic(), parsed scan)
        self.seq = 0
        self.bad = 0
        self.error = None
        self._stopping = False

    def start(self):
        if shutil.which("gz") is None:
            raise LidarError(
                "Could not find the 'gz' command. Is Gazebo installed, and is "
                "this terminal set up like the one running the simulator?"
            )
        cmd = ["gz", "topic", "-e", "-t", self.topic]
        if shutil.which("stdbuf"):  # ask for line-by-line output on a pipe
            cmd = ["stdbuf", "-oL"] + cmd
        self.proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _publish(self, lines):
        try:
            scan = parse_scan("\n".join(lines))
        except LidarError:
            self.bad += 1
            return
        self.latest = (time.monotonic(), scan)
        self.seq += 1

    def _reader(self):
        """Split the endless text output into one message per scan."""
        buf = []
        has_ranges = False
        try:
            for raw in self.proc.stdout:
                line = raw.rstrip("\n")
                blank = not line.strip()
                if has_ranges:
                    key = line.split(":", 1)[0].split("{", 1)[0].strip()
                    new_message = blank or (
                        not line[0].isspace()
                        and key not in ("ranges", "intensities")
                    )
                    if new_message:
                        self._publish(buf)
                        buf, has_ranges = [], False
                if blank:
                    continue
                buf.append(line)
                if line.lstrip().startswith("ranges:"):
                    has_ranges = True
            if has_ranges:
                self._publish(buf)
        except Exception as error:
            if not self._stopping:
                self.error = error
            return

        if not self._stopping:
            try:
                detail = self.proc.stderr.read().strip()
            except Exception:
                detail = ""
            self.error = LidarError(
                f"'gz topic -e -t {self.topic}' stopped unexpectedly. {detail}"
            )

    async def wait_first(self, timeout):
        deadline = time.monotonic() + timeout
        while self.seq == 0:
            if self.error is not None:
                raise self.error
            if time.monotonic() > deadline:
                raise LidarError(
                    f"No lidar scan arrived from {self.topic} within "
                    f"{timeout:g} s. Check what `gz topic -e -t {self.topic}` "
                    "prints in a terminal."
                )
            await asyncio.sleep(0.1)

    def stop(self):
        self._stopping = True
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.proc.kill()


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


# ----------------------------------------------------------------------
# Path geometry
# ----------------------------------------------------------------------
def path_length(pts):
    return sum(
        math.hypot(b.north_m - a.north_m, b.east_m - a.east_m)
        for a, b in zip(pts, pts[1:])
    )


def trim_path(path, north, east):
    """Path from the point on it closest to (north, east) to its end."""
    if len(path) < 2:
        return list(path)

    best_d2, best_i, best_p = math.inf, 0, path[0]
    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        dn, de = b.north_m - a.north_m, b.east_m - a.east_m
        seg2 = dn * dn + de * de
        if seg2 < 1e-12:
            t = 0.0
        else:
            t = ((north - a.north_m) * dn + (east - a.east_m) * de) / seg2
            t = max(0.0, min(1.0, t))
        pn, pe = a.north_m + t * dn, a.east_m + t * de
        d2 = (north - pn) ** 2 + (east - pe) ** 2
        if d2 < best_d2:
            best_d2, best_i, best_p = d2, i, Point2D(pn, pe)
    return [best_p] + list(path[best_i + 1:])


def point_along(pts, distance):
    """The point `distance` metres along the polyline (clamped to its end)."""
    for a, b in zip(pts, pts[1:]):
        seg = math.hypot(b.north_m - a.north_m, b.east_m - a.east_m)
        if seg > 1e-9 and distance <= seg:
            f = distance / seg
            return Point2D(
                a.north_m + f * (b.north_m - a.north_m),
                a.east_m + f * (b.east_m - a.east_m),
            )
        distance -= seg
    return pts[-1]


# ---- speed planning: slow down BEFORE turns --------------------------
def turn_angle_at(pts, s, window=0.8):
    """How sharply the path turns at distance s along it, in degrees.

    Measured between the direction over the `window` metres before and after
    that point, so a path made of many small steps is not mistaken for a
    series of sharp corners.
    """
    p0 = point_along(pts, max(0.0, s - window))
    p1 = point_along(pts, s)
    p2 = point_along(pts, s + window)
    a = (p1.north_m - p0.north_m, p1.east_m - p0.east_m)
    b = (p2.north_m - p1.north_m, p2.east_m - p1.east_m)
    if math.hypot(*a) < 0.2 or math.hypot(*b) < 0.2:
        return 0.0
    return abs(angle_diff(math.degrees(math.atan2(b[1], b[0])),
                          math.degrees(math.atan2(a[1], a[0]))))


def corner_speed_limit(theta, v_cruise, v_corner, v_reverse):
    """Speed allowed at a turn of theta degrees: full speed up to 15
    degrees, v_corner at 90, v_reverse from 150 (a reversal), in between
    interpolated."""
    v_corner = min(v_corner, v_cruise)
    v_reverse = min(v_reverse, v_corner)
    if theta <= 15.0:
        return v_cruise
    if theta <= 90.0:
        return v_cruise + (theta - 15.0) / 75.0 * (v_corner - v_cruise)
    if theta <= 150.0:
        return v_corner + (theta - 90.0) / 60.0 * (v_reverse - v_corner)
    return v_reverse


def path_end_direction(pts):
    """Compass direction (degrees) of the last non-trivial segment."""
    for a, b in zip(reversed(pts[:-1]), reversed(pts[1:])):
        if math.hypot(b.north_m - a.north_m, b.east_m - a.east_m) > 1e-6:
            return math.degrees(
                math.atan2(b.east_m - a.east_m, b.north_m - a.north_m)
            )
    return None


def leg_end_speed(pts, next_point, v_cruise, v_corner, v_reverse, v_unknown):
    """Speed wanted at the end of this leg, from how sharply the drone must
    turn onto the next one. next_point None = unknown (use v_unknown)."""
    if next_point is None:
        return min(v_unknown, v_cruise)
    d_in = path_end_direction(pts)
    if d_in is None:
        return min(v_unknown, v_cruise)
    end = pts[-1]
    d_out = math.degrees(
        math.atan2(next_point.east_m - end.east_m,
                   next_point.north_m - end.north_m)
    )
    return corner_speed_limit(abs(angle_diff(d_out, d_in)),
                              v_cruise, v_corner, v_reverse)


def speed_limit_ahead(pts, v_cruise, v_corner, v_reverse, decel,
                      end_speed=None, end_at=None, step=0.25):
    """Highest speed (m/s) the drone may fly at right now.

    For every turn within braking distance, the drone must be able to slow
    to that turn's speed at a gentle `decel` (m/s^2) by the time it gets
    there: v^2 <= v_turn^2 + 2 * decel * distance. end_speed is the speed
    wanted end_at metres ahead (where the leg is switched to the next one).
    Far from any turn this is just v_cruise.
    """
    remaining = path_length(pts)
    look = v_cruise * v_cruise / (2.0 * decel) + 1.0
    limit = v_cruise
    s = step
    while s <= min(remaining, look):
        v_turn = corner_speed_limit(turn_angle_at(pts, s),
                                    v_cruise, v_corner, v_reverse)
        if v_turn < v_cruise:
            limit = min(limit, math.sqrt(v_turn * v_turn + 2.0 * decel * s))
        s += step
    if end_speed is not None and end_at is not None and end_at <= look:
        limit = min(limit, math.sqrt(end_speed * end_speed
                                     + 2.0 * decel * max(0.0, end_at)))
    return limit


def cell_seen(grid, north, east):
    """True if the lidar has looked at this spot (free or occupied)."""
    row, col = grid.to_cell(north, east)
    return grid.in_grid(row, col) and grid.seen[row * grid.cols + col]


def free_extent(grid, pts, max_distance):
    """How far along the path the map is confirmed (seen) before a gap.

    A gap is two samples in a row (about 1 m) on cells the lidar never
    looked at. Returns the distance to the start of the gap, or math.inf if
    there is none within max_distance. A single unseen cell is ignored: the
    lidar's thin rays can leave tiny holes far away.
    """
    step = grid.resolution_m
    walked = 0.0
    next_sample = step
    run = 0
    first = None

    for a, b in zip(pts, pts[1:]):
        seg = math.hypot(b.north_m - a.north_m, b.east_m - a.east_m)
        if seg < 1e-9:
            continue
        while next_sample <= walked + seg:
            if next_sample > max_distance:
                return math.inf
            f = (next_sample - walked) / seg
            n = a.north_m + f * (b.north_m - a.north_m)
            e = a.east_m + f * (b.east_m - a.east_m)
            if cell_seen(grid, n, e):
                run, first = 0, None
            else:
                run += 1
                if first is None:
                    first = next_sample
                if run >= 2:
                    return first
            next_sample += step
        walked += seg
    return math.inf


# ----------------------------------------------------------------------
# Frontiers: where the known map meets the unknown
# ----------------------------------------------------------------------
def coverage(grid):
    """Fraction of the grid the lidar has looked at (0..1)."""
    return sum(grid.seen) / float(len(grid.seen))


def cell_to_point(grid, row, col):
    return (
        grid.min_north_m + row * grid.resolution_m,
        grid.min_east_m + col * grid.resolution_m,
    )


def find_frontiers(grid, is_free, min_cells=3, min_unknown=6, exclude=()):
    """Groups of 'frontier' cells, each a list of (row, col).

    A frontier cell is a cell the lidar saw to be free that touches a cell it
    never looked at, so flying there reveals something new. Three filters
    keep out false alarms:
      * min_unknown: at least this many unseen cells must lie within two
        cells (5 x 5 block). The lidar's thin far-away rays leave single
        unseen pinholes, which are not worth a trip.
      * is_free(north, east): the planner must be willing to fly there.
      * exclude: (north, east, radius_m) spots to ignore (places already
        tried and given up on).
    Groups smaller than min_cells are dropped.
    """
    rows, cols = grid.rows, grid.cols
    seen, odds = grid.seen, grid.log_odds

    def unseen(r, c):
        return 0 <= r < rows and 0 <= c < cols and not seen[r * cols + c]

    frontier = set()
    for r in range(rows):
        base = r * cols
        for c in range(cols):
            i = base + c
            if not seen[i] or odds[i] > OCC_THRESHOLD:
                continue
            if not (unseen(r - 1, c) or unseen(r + 1, c)
                    or unseen(r, c - 1) or unseen(r, c + 1)):
                continue
            unknown = 0
            for dr in range(-2, 3):
                for dc in range(-2, 3):
                    if unseen(r + dr, c + dc):
                        unknown += 1
            if unknown < min_unknown:
                continue
            north, east = cell_to_point(grid, r, c)
            if any(math.hypot(north - n, east - e) <= rad for n, e, rad in exclude):
                continue
            if not is_free(north, east):
                continue
            frontier.add((r, c))

    clusters = []
    todo = set(frontier)
    while todo:
        first = todo.pop()
        stack, group = [first], [first]
        while stack:
            r, c = stack.pop()
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    nb = (r + dr, c + dc)
                    if nb in todo:
                        todo.remove(nb)
                        stack.append(nb)
                        group.append(nb)
        if len(group) >= min_cells:
            clusters.append(group)
    return clusters


EXPLORE_SKIP_RADIUS_M = 2.0   # give-up radius around a frontier that failed
EXPLORE_DONE_RADIUS_M = 2.0   # a frontier "still exists" if one is this close
EXPLORE_MAX_FAILURES = 3      # unreachable frontiers in a row before giving up


# ----------------------------------------------------------------------
# Saving and loading the map between runs
# ----------------------------------------------------------------------
def _geometry(grid):
    return {
        "min_north_m": grid.min_north_m,
        "min_east_m": grid.min_east_m,
        "resolution_m": grid.resolution_m,
        "rows": grid.rows,
        "cols": grid.cols,
    }


def _same_geometry(saved, grid):
    now = _geometry(grid)
    try:
        return (
            saved["rows"] == now["rows"]
            and saved["cols"] == now["cols"]
            and all(
                abs(float(saved[k]) - now[k]) < 1e-6
                for k in ("min_north_m", "min_east_m", "resolution_m")
            )
        )
    except (KeyError, TypeError, ValueError):
        return False


def save_grid(grid, path, origin):
    """Write every cell the lidar has seen to `path` (JSON). Returns the count.

    Only seen cells are stored, as [index, log_odds]. The file is written to
    a temporary name and then renamed, so a crash never leaves half a map.
    """
    cells = [[i, round(grid.log_odds[i], 3)] for i, s in enumerate(grid.seen) if s]
    data = {
        "version": MAP_FILE_VERSION,
        "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "origin_lat_lon": [origin[0], origin[1]],
        "grid": _geometry(grid),
        "cells": cells,
    }
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(data, separators=(",", ":")))
    os.replace(tmp, path)
    return len(cells)


def load_grid(grid, path, origin, origin_tol_m, decay):
    """Load a saved map into an empty grid.

    Returns (loaded, message). Nothing is loaded (and the grid is left
    untouched) if the file is unreadable, was made with different grid
    settings, or was made from a different start point: an old map placed on
    the wrong spot would put obstacles where there are none.

    decay (0..1) scales the old log-odds, so fresh scans can overrule old
    ones quickly (1.0 = trust the old map completely).
    """
    path = Path(path)
    try:
        data = json.loads(path.read_text())
        if data.get("version") != MAP_FILE_VERSION:
            return False, f"{path.name} has an unknown format version."
        cells = data["cells"]
        saved_origin = data["origin_lat_lon"]
        saved_grid = data["grid"]
    except (OSError, ValueError, KeyError, AttributeError) as error:
        return False, f"Could not read {path.name} ({error}). Starting with an empty map."

    if not _same_geometry(saved_grid, grid):
        return False, (
            f"{path.name} was made with different grid settings (area or "
            "resolution in mission.yaml changed). Starting with an empty map."
        )

    d_north, d_east = gps_to_offset(
        float(saved_origin[0]), float(saved_origin[1]), origin[0], origin[1]
    )
    shift = math.hypot(d_north, d_east)
    if shift > origin_tol_m:
        return False, (
            f"{path.name} was made from a start point {shift:.1f} m away from "
            f"this one (limit {origin_tol_m:g} m). Starting with an empty map."
        )

    size = grid.rows * grid.cols
    try:
        parsed = [(int(i), float(v)) for i, v in cells]
    except (TypeError, ValueError):
        return False, f"{path.name} contains bad cell data. Starting with an empty map."
    if any(not 0 <= i < size for i, _ in parsed):
        return False, f"{path.name} contains out-of-range cells. Starting with an empty map."

    for i, v in parsed:
        grid.log_odds[i] = v * decay
        grid.seen[i] = True
    # The grid caches its inflated obstacle set; make it recompute.
    grid._version += 1
    grid._inflated_cache.clear()

    occupied = sum(1 for v in grid.log_odds if v > OCC_THRESHOLD)
    when = data.get("saved_at", "unknown time")
    return True, (
        f"Loaded map from {path.name} (saved {when}): {len(parsed)} known "
        f"cells, {occupied} occupied."
    )


def persist_map(grid, args, origin, lock=None):
    """Save the map if saving is on. Never raises (a failed save must not
    hide the flight result or stop the landing)."""
    if not args.save_map or origin is None:
        return False
    try:
        if lock is not None:
            with lock:
                count = save_grid(grid, args.map_file, origin)
        else:
            count = save_grid(grid, args.map_file, origin)
    except Exception as error:
        print(f"(Could not save the map to {args.map_file}: {error})")
        return False
    print(f"Saved map ({count} cells) to {args.map_file}")
    return True


class ResistantGrid(OccupancyGrid):
    """OccupancyGrid where cells already seen as occupied are harder to
    erase.

    Why: a beam that runs almost parallel to a wall (for example looking
    down an aisle from its end) passes through the wall's edge cells before
    it ends, and marks them free. That punched holes in the walls and left
    obstacles only 0.25 m from a planned path. Free evidence against an
    occupied cell now counts `occupied_resistance` times as much (1.0 gives
    the original behaviour). Real obstacles that are gone still clear, just
    a little slower.
    """

    def __init__(self, *args, occupied_resistance=0.5, **kwargs):
        super().__init__(*args, **kwargs)
        self.occupied_resistance = occupied_resistance

    def _add(self, row, col, delta):
        if delta < 0 and self.log_odds[self._idx(row, col)] > OCC_THRESHOLD:
            delta *= self.occupied_resistance
        super()._add(row, col, delta)


# ----------------------------------------------------------------------
# Map + planner, shared between threads
# ----------------------------------------------------------------------
class MapService:
    """The occupancy grid and planner behind one lock.

    Updating the grid and planning both take tens of milliseconds, so they
    run in worker threads; the lock keeps them from touching the grid at the
    same time.
    """

    def __init__(self, planner, grid):
        self.planner = planner
        self.grid = grid
        self.lock = threading.Lock()
        # Which heavy jobs are running right now (read by the flight log).
        self.busy = {"map": False, "plan": False, "frontier": False}

    def update(self, north, east, rays):
        with self.lock:
            self.busy["map"] = True
            try:
                self.grid.update(north, east, rays)
            finally:
                self.busy["map"] = False

    def _path_ok(self, pts):
        p = self.planner
        if len(pts) < 2 or not p.is_point_free(pts[-1]):
            return False
        return all(p.is_segment_free(a, b) for a, b in zip(pts, pts[1:]))

    def plan_job(self, start, target, old_path, force):
        """Returns (kind, route, seconds); kind is 'keep', 'new' or
        'target-blocked'. 'keep' means the old path is still collision-free."""
        t0 = time.monotonic()
        with self.lock:
            if not self.planner.is_point_free(target):
                return "target-blocked", None, time.monotonic() - t0
            if old_path is not None and not force and self._path_ok(old_path):
                return "keep", None, time.monotonic() - t0
            self.busy["plan"] = True
            try:
                route = self.planner.plan(start, target, allow_blocked_start=True)
            finally:
                self.busy["plan"] = False
        return "new", route, time.monotonic() - t0


class Stats:
    def __init__(self):
        self.t0 = time.monotonic()
        self.state_time = collections.defaultdict(float)
        self.plan_times = []
        self.plans_kept = 0
        self.integrated = 0
        self.skipped_rate = 0
        self.skipped_tilt = 0
        self.skipped_pose = 0

    def add_plan(self, kind, seconds):
        if kind == "keep":
            self.plans_kept += 1
        elif kind == "new":
            self.plan_times.append(seconds)

    def report(self, stream, flown):
        total = max(1e-6, time.monotonic() - self.t0)
        print("\n--- Timing summary ---")
        print(
            f"Navigation time {total:.1f} s, distance flown {flown:.1f} m, "
            f"average speed {flown / total:.2f} m/s"
        )
        print("Where the time went:")
        for state, secs in sorted(self.state_time.items(), key=lambda kv: -kv[1]):
            print(f"  {state:<9} {secs:6.1f} s  ({100 * secs / total:.0f}%)")
        print(
            f"Lidar: {stream.seq} scans received ({stream.seq / total:.1f}/s), "
            f"{self.integrated} added to the map; skipped: "
            f"{self.skipped_rate} (rate limit), {self.skipped_tilt} (drone "
            f"tilted), {self.skipped_pose} (no pose)"
        )
        if self.plan_times:
            mean = 1000 * sum(self.plan_times) / len(self.plan_times)
            worst = 1000 * max(self.plan_times)
            print(
                f"Planner: {len(self.plan_times)} replans (mean {mean:.0f} ms, "
                f"worst {worst:.0f} ms), path kept {self.plans_kept} times"
            )


# ----------------------------------------------------------------------
# Flight log (CSV), for finding out why the drone loses height
# ----------------------------------------------------------------------
LOG_COLUMNS = [
    "wall", "t", "nav_t", "state", "mode",
    "north", "east", "alt", "alt_target", "alt_err",
    "vn", "ve", "vd", "speed",
    "roll", "pitch", "heading", "yaw_sp", "yaw_err",
    "sp_dist", "horizon",
    "send_gap_ms", "pose_age_ms", "loop_lag_ms",
    "busy_map", "busy_plan", "busy_frontier", "tilt_skips",
    "goal_dist", "yaw_goal_err",
]


class FlightLogger:
    """Writes one CSV row per 1/rate_hz seconds while the drone navigates.

    alt_err is how far BELOW the target height the drone is (positive =
    sagging). send_gap_ms is the longest pause between setpoints sent to PX4
    since the last row, loop_lag_ms is how late this logger woke up (both
    jump when Python is too busy), and busy_* say which heavy job was running.
    Any error in here is swallowed: logging must never affect the flight.
    """

    def __init__(self, path, tracker, ctrl, svc, navigator, stats, rate_hz=10.0):
        self.path = Path(path)
        self.tracker, self.ctrl, self.svc = tracker, ctrl, svc
        self.nav, self.stats = navigator, stats
        self.period = 1.0 / rate_hz
        self.t0 = time.monotonic()
        self._file = None
        self._writer = None
        self._task = None
        self.rows = 0
        self.failed = False

    def start(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = open(self.path, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(LOG_COLUMNS)
        self._file.flush()
        self.t0 = time.monotonic()
        self._task = asyncio.create_task(self._run())

    async def stop(self):
        if self._task is not None:
            self._task.cancel()
            await asyncio.gather(self._task, return_exceptions=True)
            self._task = None
        if self._file is not None:
            try:
                self._file.close()
            except OSError:
                pass
            self._file = None
            print(f"Flight log: {self.rows} rows in {self.path}")

    async def _run(self):
        last = time.monotonic()
        while True:
            await asyncio.sleep(self.period)
            now = time.monotonic()
            lag_ms = max(0.0, (now - last - self.period) * 1000.0)
            last = now
            if self.failed:
                continue
            try:
                self._write(now, lag_ms)
            except Exception as error:  # never let the log hurt the flight
                self.failed = True
                print(f"(Flight log disabled after an error: {error!r})")

    def _write(self, now, lag_ms):
        tr, ctrl, nav = self.tracker, self.ctrl, self.nav
        if tr._ned is None or tr.heading is None or ctrl._target is None:
            return
        raw_n, raw_e, down = tr._ned
        sp_n, sp_e, sp_d, sp_yaw = ctrl._target
        goal_n, goal_e, _, goal_yaw = getattr(ctrl, "_goal", None) or ctrl._target
        gap_ms = ctrl.max_gap * 1000.0
        ctrl.max_gap = 0.0
        pose_age = (now - tr.last_update) * 1000.0 if tr.last_update else 0.0
        horizon = nav.horizon if math.isfinite(nav.horizon) else 99.0
        busy = self.svc.busy
        row = [
            time.strftime("%H:%M:%S"),
            now - self.t0,
            now - self.stats.t0,
            nav.state,
            tr.flight_mode,
            tr.north, tr.east, -down, -ctrl.down, down - ctrl.down,
            tr.vel_north, tr.vel_east, tr.vel_down, tr.speed,
            tr.roll if tr.roll is not None else 0.0,
            tr.pitch if tr.pitch is not None else 0.0,
            tr.heading, sp_yaw % 360.0, angle_diff(sp_yaw, tr.heading),
            math.hypot(sp_n - raw_n, sp_e - raw_e), horizon,
            gap_ms, pose_age, lag_ms,
            int(busy["map"]), int(busy["plan"]), int(busy["frontier"]),
            self.stats.skipped_tilt,
            math.hypot(goal_n - raw_n, goal_e - raw_e),
            angle_diff(goal_yaw, tr.heading),
        ]
        self._writer.writerow(
            [f"{v:.3f}" if isinstance(v, float) else v for v in row]
        )
        self._file.flush()
        self.rows += 1


# ----------------------------------------------------------------------
# Mapper: scans -> grid, in the background
# ----------------------------------------------------------------------
class Mapper:
    def __init__(self, stream, tracker, svc, stats, args, max_tilt):
        self.stream = stream
        self.tracker = tracker
        self.svc = svc
        self.stats = stats
        self.args = args
        self.max_tilt = max_tilt
        self.error = None
        self._task = None

    def start(self):
        self._task = asyncio.create_task(self._guard())

    async def stop(self):
        if self._task is not None:
            self._task.cancel()
            await asyncio.gather(self._task, return_exceptions=True)
            self._task = None

    def check(self):
        if self.error is not None:
            raise RuntimeError(f"Mapping failed: {self.error!r}")

    async def _guard(self):
        try:
            await self._run()
        except asyncio.CancelledError:
            raise
        except Exception as error:
            self.error = error

    async def _run(self):
        args = self.args
        min_gap = 1.0 / args.map_rate
        last_seq = 0
        last_used = 0.0

        while True:
            await asyncio.sleep(0.02)
            if self.stream.error is not None:
                raise self.stream.error

            item, seq = self.stream.latest, self.stream.seq
            if item is None or seq == last_seq:
                continue
            last_seq = seq

            if time.monotonic() - last_used < min_gap:
                self.stats.skipped_rate += 1
                continue

            t_scan, scan = item
            pose = self.tracker.pose_at(t_scan - args.scan_delay)
            if pose is None:
                self.stats.skipped_pose += 1
                continue

            north, east, heading, tilt = pose
            if tilt > self.max_tilt:
                self.stats.skipped_tilt += 1
                continue

            rays = scan_to_rays(scan, north, east, heading)[:: args.ray_stride]
            await asyncio.to_thread(self.svc.update, north, east, rays)
            last_used = time.monotonic()
            self.stats.integrated += 1


# ----------------------------------------------------------------------
# Navigator: plan in the background, follow the path continuously
# ----------------------------------------------------------------------
async def wait_still(tracker, timeout=4.0, speed_tol=0.2, hold_s=0.5):
    still_since = None
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        tracker.check()
        if tracker.speed < speed_tol:
            if still_since is None:
                still_since = time.monotonic()
            if time.monotonic() - still_since >= hold_s:
                return True
        else:
            still_since = None
        await asyncio.sleep(0.1)
    return False


class Navigator:
    def __init__(self, tracker, ctrl, svc, mapper, stream, stats, args):
        self.tracker = tracker
        self.ctrl = ctrl
        self.svc = svc
        self.grid = svc.grid
        self.mapper = mapper
        self.stream = stream
        self.stats = stats
        self.args = args
        self.yaw_target = tracker.heading
        self.state = "idle"        # what the controller is doing (for the log)
        self.horizon = math.inf    # how far ahead the map is known free
        self.flown = 0.0
        self.trail = [tracker.pose()[:2]]
        self._last_pos = tracker.pose()[:2]

    def _check(self):
        self.ctrl.check()
        self.tracker.check()
        self.mapper.check()

    def _account(self, north, east):
        self.flown += math.hypot(north - self._last_pos[0], east - self._last_pos[1])
        self._last_pos = (north, east)
        if math.hypot(north - self.trail[-1][0], east - self.trail[-1][1]) > 0.5:
            self.trail.append((north, east))

    async def fly_target(self, index, count, target, final, still_wanted=None,
                         next_target=None):
        """Returns 'reached', 'skipped' (target unusable), 'failed', or
        'obsolete' (still_wanted() became False: the target is no longer
        worth flying to).

        still_wanted: optional async function, asked about once a second.
        next_target: where the drone goes after this one, if known; it sets
        how much to slow down for the turn at this target.
        """
        args, tracker, ctrl = self.args, self.tracker, self.ctrl
        reach = args.tolerance if final else args.corner_radius
        lookahead = max(0.5, args.speed)

        t_start = time.monotonic()
        last_tick = t_start
        last_want = t_start
        last_status = 0.0
        last_plan_launch = -1e9
        best_dist, best_t = math.inf, t_start
        speed_cap = max(tracker.speed, 0.3)   # grows while the leg is flown

        path = None
        path_planned = 0.0
        plan_task = None
        plan_start = None
        no_path_since = None
        wait_map_since = None
        turning = False

        try:
            while True:
                await asyncio.sleep(CONTROL_PERIOD_S)
                now = time.monotonic()
                dt, last_tick = now - last_tick, now
                self._check()

                north, east, heading = tracker.pose()
                self._account(north, east)
                dist = math.hypot(target.north_m - north, target.east_m - east)

                if dist <= reach:
                    return "reached"
                if still_wanted is not None and now - last_want >= 1.0:
                    last_want = now
                    if not await still_wanted():
                        return "obsolete"
                if now - t_start > args.target_timeout:
                    print(f"Target {index}: timed out after {args.target_timeout:g} s.")
                    return "failed"
                if dist < best_dist - 0.5:
                    best_dist, best_t = dist, now
                elif now - best_t > args.stall_timeout:
                    print(f"Target {index}: no progress for {args.stall_timeout:g} s.")
                    return "failed"

                # ---- collect a finished planner job -------------------
                if plan_task is not None and plan_task.done():
                    kind, route, seconds = plan_task.result()
                    plan_task = None
                    self.stats.add_plan(kind, seconds)
                    if kind == "target-blocked":
                        print(
                            f"Target {index} is now inside the safety margin "
                            "of a detected obstacle (or outside the flight "
                            "area). Skipping it."
                        )
                        return "skipped"
                    if kind == "new":
                        if route is None:
                            path = None
                            if no_path_since is None:
                                no_path_since = now
                        else:
                            path = [plan_start] + list(route)
                            path_planned = now
                            no_path_since = None

                # ---- start a new planner job --------------------------
                if plan_task is None and now - last_plan_launch >= args.replan_period:
                    old = None if path is None else trim_path(path, north, east)
                    force = path is not None and (
                        now - path_planned > args.reoptimize_period
                    )
                    plan_start = Point2D(north, east)
                    plan_task = asyncio.create_task(
                        asyncio.to_thread(
                            self.svc.plan_job, plan_start, target, old, force
                        )
                    )
                    last_plan_launch = now

                # ---- decide the setpoint ------------------------------
                state = "cruise"
                horizon = math.inf

                if path is None:
                    if no_path_since is not None:
                        state = "no-path"
                        ctrl.hold(tracker, self.yaw_target)
                        if now - no_path_since > args.no_path_timeout:
                            print(f"Target {index}: no safe path with the "
                                  "current map.")
                            return "failed"
                    else:
                        # First plan not ready yet: keep the last setpoint
                        # (no braking at corners).
                        state = "planning"
                else:
                    pts = trim_path(path, north, east)
                    remaining = path_length(pts)

                    # How far ahead to aim (this sets the speed). Cruise
                    # uses the full lookahead; before turns it is shortened
                    # so the drone arrives slowly and cuts the corner less.
                    lead, slowing = lookahead, False
                    if args.governor:
                        if final:
                            end_speed = 0.2
                        else:
                            end_speed = leg_end_speed(
                                pts, next_target, args.speed,
                                args.corner_speed, args.turn_speed,
                                args.waypoint_speed)
                        v_limit = speed_limit_ahead(
                            pts, args.speed, args.corner_speed,
                            args.turn_speed, args.plan_decel,
                            end_speed=end_speed,
                            end_at=max(0.0, remaining - reach))
                        # Speed up again gently after a turn or a stop: the
                        # cap grows from the speed the leg started at.
                        speed_cap += args.plan_accel * dt
                        v_limit = min(v_limit, speed_cap)
                        ratio = min(1.0, v_limit / args.speed)
                        lead = max(min(0.4, lookahead), lookahead * ratio)
                        slowing = ratio < 0.9
                    carrot_d = min(lead, remaining)

                    # Only run ahead as far as the lidar has confirmed free.
                    horizon = free_extent(
                        self.grid, pts, carrot_d + args.stop_margin
                    )
                    if horizon < carrot_d + args.stop_margin:
                        carrot_d = max(0.0, horizon - args.stop_margin)

                    if carrot_d < 0.3:
                        state = "wait-map"
                        if wait_map_since is None:
                            wait_map_since = now
                        if now - wait_map_since > 3.0:
                            # The map is not filling in: creep forward slowly.
                            state = "creep"
                            carrot_d = min(0.6, remaining)
                        else:
                            carrot_d = 0.0
                    else:
                        wait_map_since = None
                        if slowing and state == "cruise":
                            state = "slowing"

                    carrot = point_along(pts, carrot_d)

                    if not args.hold_yaw:
                        to_carrot = math.hypot(
                            carrot.north_m - north, carrot.east_m - east
                        )
                        if to_carrot > 0.5:
                            bearing = math.degrees(
                                math.atan2(
                                    carrot.east_m - east, carrot.north_m - north
                                )
                            ) % 360.0
                            if abs(angle_diff(bearing, self.yaw_target)) > 10.0:
                                self.yaw_target = bearing

                        err = abs(angle_diff(self.yaw_target, heading))
                        if turning and err < 35.0:
                            turning = False
                        elif not turning and err > args.turn_in_place:
                            turning = True
                        if turning:
                            state = "turning"
                            carrot = Point2D(north, east)

                    ctrl.set_target_map(
                        tracker, carrot.north_m, carrot.east_m, self.yaw_target
                    )

                self.stats.state_time[state] += dt
                self.state, self.horizon = state, horizon

                if now - last_status >= 2.0:
                    last_status = now
                    extent = "all clear" if math.isinf(horizon) else f"{horizon:.1f} m"
                    print(
                        f"  t={now - self.stats.t0:5.1f}s  "
                        f"pos=({north:5.1f}, {east:5.1f})  {dist:5.1f} m to go  "
                        f"speed {tracker.speed:.1f} m/s  {state}  "
                        f"(known free ahead: {extent})"
                    )
        finally:
            if plan_task is not None and not plan_task.done():
                plan_task.cancel()

    # ---- exploration -------------------------------------------------
    def _frontier_clusters(self, exclude=()):
        # Callers hold svc.lock (the mapper changes the grid in a thread).
        planner = self.svc.planner
        return find_frontiers(
            self.grid,
            lambda n, e: planner.is_point_free(Point2D(n, e)),
            min_cells=self.args.min_frontier,
            exclude=exclude,
        )

    def pick_frontier(self, start, blacklist):
        """Choose where to explore next. Runs in a worker thread.

        Returns (point or None, path length, number of unexplored areas,
        points that turned out to be unreachable).
        """
        grid, planner = self.grid, self.svc.planner
        with self.svc.lock:
            self.svc.busy["frontier"] = True
            try:
                clusters = self._frontier_clusters(blacklist)
                cands = []
                for cluster in clusters:
                    # The cell of each area that is closest to the drone.
                    points = [cell_to_point(grid, r, c) for r, c in cluster]
                    n, e = min(
                        points,
                        key=lambda p: math.hypot(p[0] - start.north_m,
                                                 p[1] - start.east_m),
                    )
                    cands.append(
                        (math.hypot(n - start.north_m, e - start.east_m),
                         Point2D(n, e))
                    )
                cands.sort(key=lambda item: item[0])

                best, best_cost, unreachable = None, None, []
                # Check real path lengths for the few nearest, take the cheapest.
                for _, point in cands[:6]:
                    route = planner.plan(start, point, allow_blocked_start=True)
                    if route is None:
                        unreachable.append(point)
                        continue
                    cost = path_length([start] + list(route))
                    if best is None or cost < best_cost:
                        best, best_cost = point, cost
            finally:
                self.svc.busy["frontier"] = False
        return best, best_cost, len(clusters), unreachable

    def frontier_near(self, point, radius=EXPLORE_DONE_RADIUS_M):
        """True if some unexplored edge is still within `radius` of point."""
        with self.svc.lock:
            self.svc.busy["frontier"] = True
            try:
                clusters = self._frontier_clusters()
            finally:
                self.svc.busy["frontier"] = False
        for cluster in clusters:
            for r, c in cluster:
                n, e = cell_to_point(self.grid, r, c)
                if math.hypot(n - point.north_m, e - point.east_m) <= radius:
                    return True
        return False

    async def _frontier_alive(self, point):
        return await asyncio.to_thread(self.frontier_near, point)

    async def settle_at(self, point):
        """Come to rest at point (before looking around)."""
        self.state = "settle"
        self.ctrl.set_target_map(
            self.tracker, point.north_m, point.east_m, self.yaw_target
        )
        await wait_still(self.tracker, timeout=5.0)

    async def look_around(self):
        """Turn a full circle on the spot, for a lidar that cannot see all
        around or has blind spots. The mapper keeps adding scans meanwhile."""
        north, east, heading = self.tracker.pose()
        self.state = "look-around"
        print("  Unexplored edge still there: looking around.")
        for step in (1, 2, 3, 4):
            yaw = (heading + 90.0 * step) % 360.0
            self.yaw_target = yaw
            deadline = time.monotonic() + 4.0
            while time.monotonic() < deadline:
                self._check()
                self.ctrl.set_target_map(self.tracker, north, east, yaw)
                if abs(angle_diff(yaw, self.tracker.heading)) < 15.0:
                    break
                await asyncio.sleep(0.1)
            await asyncio.sleep(0.4)  # let a fresh scan arrive

    async def explore(self):
        """Explore until nothing reachable is unexplored, then go home.

        Returns True unless the drone got stuck.
        """
        args = self.args
        north, east, _ = self.tracker.pose()
        home = Point2D(north, east)
        t0 = time.monotonic()
        self.stats.t0 = t0
        blacklist = []   # (north, east, radius): places given up on
        visited = 0
        failures = 0
        ok = True

        def give_up_on(point):
            blacklist.append((point.north_m, point.east_m, EXPLORE_SKIP_RADIUS_M))

        while True:
            self._check()
            if time.monotonic() - t0 > args.explore_timeout:
                print(f"\nExploration time limit ({args.explore_timeout:g} s) "
                      "reached.")
                break

            north, east, _ = self.tracker.pose()
            self.state = "choosing"
            point, cost, left, unreachable = await asyncio.to_thread(
                self.pick_frontier, Point2D(north, east), blacklist
            )
            for p in unreachable:
                give_up_on(p)
            if point is None:
                if unreachable:
                    continue  # try the remaining areas
                print("\nExploration complete: no reachable unexplored area "
                      "is left.")
                break

            visited += 1
            print(
                f"\n=== Frontier {visited}: north={point.north_m:.1f}, "
                f"east={point.east_m:.1f} ({cost:.1f} m away, {left} "
                f"unexplored area(s), {100 * coverage(self.grid):.0f}% of "
                "the flight area seen) ==="
            )
            t_leg = time.monotonic()

            async def wanted(p=point):
                return await self._frontier_alive(p)

            result = await self.fly_target(visited, 0, point, False,
                                           still_wanted=wanted)
            print(f"Frontier {visited}: {result} ({time.monotonic() - t_leg:.1f} s)")

            if result == "failed":
                give_up_on(point)
                failures += 1
                self.ctrl.hold(self.tracker, self.yaw_target)
                if failures >= EXPLORE_MAX_FAILURES:
                    print(f"{failures} unreachable areas in a row. Giving up "
                          "on exploring.")
                    ok = False
                    break
                continue
            failures = 0

            if result == "skipped":
                give_up_on(point)
            elif result == "reached" and await self._frontier_alive(point):
                # Arrived, but the edge is still there: stop, look around.
                await self.settle_at(point)
                if await self._frontier_alive(point):
                    await self.look_around()
                if await self._frontier_alive(point):
                    print("  Still unknown from here; not worth more time.")
                    give_up_on(point)

            if args.show_map:
                cfg = self.svc.planner.config
                n, e, _ = self.tracker.pose()
                print(render_ascii(self.grid, cfg.safety_margin_m, None,
                                   (n, e), (point.north_m, point.east_m)))

        print(f"\nExplored {visited} area(s) in "
              f"{time.monotonic() - t0:.0f} s; "
              f"{100 * coverage(self.grid):.0f}% of the flight area seen.")

        if not args.no_return:
            print("\n=== Returning to the start ===")
            result = await self.fly_target(0, 0, home, True)
            print(f"Return: {result}")
            if result == "failed":
                ok = False
            elif result == "reached":
                self.ctrl.set_target_map(
                    self.tracker, home.north_m, home.east_m, self.yaw_target
                )
                await wait_still(self.tracker)
        return ok

    async def run(self, targets):
        """Visit every target. Returns True unless something failed."""
        self.stats.t0 = time.monotonic()
        skipped = 0

        for i, target in enumerate(targets, start=1):
            final = i == len(targets)
            print(
                f"\n=== Target {i}/{len(targets)}: "
                f"north={target.north_m:.1f}, east={target.east_m:.1f} ==="
            )
            t0 = time.monotonic()
            upcoming = targets[i] if i < len(targets) else None
            result = await self.fly_target(i, len(targets), target, final,
                                           next_target=upcoming)
            print(f"Target {i}: {result} ({time.monotonic() - t0:.1f} s)")

            if result == "failed":
                return False
            if result == "skipped":
                skipped += 1
            elif final:
                # Come to rest on the last target (corners are flown through).
                self.ctrl.set_target_map(
                    self.tracker, target.north_m, target.east_m, self.yaw_target
                )
                await wait_still(self.tracker)

            if self.args.show_map:
                cfg = self.svc.planner.config
                north, east, _ = self.tracker.pose()
                print(render_ascii(self.grid, cfg.safety_margin_m, None,
                                   (north, east),
                                   (target.north_m, target.east_m)))

        if skipped:
            print(f"\n{skipped} target(s) were skipped as unusable.")
        return True


# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------
async def wait_for_altitude(tracker, altitude, timeout=30.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        tracker.check()
        if -tracker.down >= altitude * 0.85:
            return
        await asyncio.sleep(0.1)
    raise TimeoutError(f"Did not reach {altitude:.1f} m within {timeout:g} s.")


async def wait_until(condition, timeout, what):
    deadline = time.monotonic() + timeout
    while not condition():
        if time.monotonic() > deadline:
            raise RuntimeError(f"Timed out waiting for {what}.")
        await asyncio.sleep(0.05)


async def run(args, planner, grid, targets):
    cfg = planner.config
    drone = await connect(args.address, args.timeout)
    tracker = ctrl = stream = mapper = svc = origin = logger = None
    airborne = False
    saved = False

    try:
        await wait_for_health(drone, 60)
        await set_telemetry_rates(drone)

        if args.origin:
            try:
                origin = tuple(float(v) for v in args.origin.split(","))
                if len(origin) != 2:
                    raise ValueError
            except ValueError:
                raise LidarError("--origin must look like LAT,LON") from None
        else:
            try:
                origin = await read_origin(drone, args.timeout)
            except AttributeError:
                raise LidarError(
                    "This MAVSDK version has no home() telemetry. "
                    "Pass the start point with --origin LAT,LON."
                ) from None

        if args.load_map:
            if args.map_file.exists():
                _, message = load_grid(grid, args.map_file, origin,
                                       args.map_origin_tol, args.map_decay)
                print(message)
            else:
                print(f"No saved map at {args.map_file}: starting with an "
                      "empty map (one will be saved at the end).")

        topic = args.topic or await asyncio.to_thread(find_scan_topic, args.timeout)
        print(f"Lidar topic: {topic}")
        stream = LidarStream(topic)
        stream.start()
        await stream.wait_first(args.timeout)

        tracker = PoseTracker(drone, origin)
        tracker.start()
        await tracker.wait_ready(args.timeout)
        ground_down = tracker.down

        print(f"Taking off to {args.altitude:.1f} m...")
        await arm_and_takeoff(drone, args.altitude)
        airborne = True
        await wait_for_altitude(tracker, args.altitude)

        shaper = None if args.no_shaping else SetpointShaper(
            args.accel, args.yaw_rate
        )
        ctrl = OffboardController(drone, shaper)
        if shaper is None:
            print("Setpoint smoothing is OFF (--no-shaping).")
        else:
            accel = ("not limited" if args.accel is None
                     else f"limited to about {args.accel:g} m/s^2")
            print(f"Setpoint smoothing: turning at most {args.yaw_rate:g} "
                  f"deg/s, acceleration {accel}.")
        await ctrl.start(tracker, down=ground_down - args.altitude)

        # A level 2D lidar at this height never sees the floor within range;
        # tilted past this angle it would, and report fake walls.
        max_tilt = args.max_tilt
        if max_tilt is None:
            max_tilt = 0.8 * math.degrees(
                math.atan2(args.altitude, args.max_range)
            )
        print(f"Lidar scans are skipped while tilted more than {max_tilt:.1f} deg.")

        stats = Stats()
        svc = MapService(planner, grid)
        mapper = Mapper(stream, tracker, svc, stats, args, max_tilt)
        mapper.start()
        await wait_until(lambda: stats.integrated >= 1 or mapper.error,
                         8.0, "the first lidar scan to reach the map")
        mapper.check()

        nav = Navigator(tracker, ctrl, svc, mapper, stream, stats, args)
        if args.log:
            log_path = args.log_file or (
                args.log_dir / time.strftime("flight_%Y%m%d_%H%M%S.csv")
            )
            logger = FlightLogger(log_path, tracker, ctrl, svc, nav, stats,
                                  args.log_rate)
            try:
                logger.start()
                print(f"Logging flight data to {log_path}")
            except OSError as error:
                print(f"(Flight log not started: {error})")
                logger = None
        if args.explore:
            success = await nav.explore()
        else:
            success = await nav.run(targets)
        if logger is not None:
            await logger.stop()   # before landing, so the descent is not logged
            logger = None
        print("\nNavigation finished." if success else "\nNavigation FAILED.")
        stats.report(stream, nav.flown)

        await mapper.stop()
        mapper = None
        saved = persist_map(grid, args, origin, svc.lock)

        final = tracker.pose()
        print("\n# = seen obstacle, + = safety margin, . = seen free, "
              "blank = unknown")
        print("D = drone, T = last target, * = path flown\n")
        if args.explore:
            print(f"(Exploration covered {100 * coverage(grid):.0f}% of the "
                  "flight area; the rest is out of reach or hidden.)\n")
        print(render_ascii(grid, cfg.safety_margin_m, nav.trail,
                           (final[0], final[1]),
                           (targets[-1].north_m, targets[-1].east_m)
                           if targets else None))

        print("\nLanding...")
        await ctrl.stop()
        await land_and_wait(drone)
        airborne = False

    finally:
        # Keep what was learned even if the flight ended early (a quick
        # write; the lock stops a half-finished scan update from being saved).
        if logger is not None:
            await logger.stop()
            logger = None
        if mapper is not None:
            await mapper.stop()
            mapper = None
        if not saved and svc is not None:
            persist_map(grid, args, origin, svc.lock)

        if airborne:
            print("\nCleanup: landing...")
            try:
                if ctrl is not None:
                    await ctrl.stop()
                await land_and_wait(drone)
            except Exception as error:
                print(
                    f"CRITICAL: could not confirm landing: {error}\n"
                    "Inspect the vehicle in Gazebo/PX4 before continuing."
                )
        if stream is not None:
            stream.stop()
        if tracker is not None:
            await tracker.stop()


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--explore", action="store_true",
                        help="no targets needed: fly to the edge of the "
                             "known map again and again until nothing "
                             "reachable is unexplored, then return to the "
                             "start. The map is saved for later runs")
    parser.add_argument("--explore-timeout", type=float, default=900.0,
                        help="stop exploring after this many seconds and go "
                             "home (default 900)")
    parser.add_argument("--min-frontier", type=int, default=3,
                        help="ignore unexplored gaps smaller than this many "
                             "map cells (default 3)")
    parser.add_argument("--no-return", action="store_true",
                        help="after exploring, land where the drone is "
                             "instead of flying back to the start")
    parser.add_argument("--target", action="append",
                        help="NORTH,EAST in metres; repeat for several. "
                             "Default: `targets` from mission.yaml")
    parser.add_argument("--altitude", type=float, default=1.5,
                        help="flight altitude in metres (default 1.5)")
    parser.add_argument("--speed", type=float, default=2.0,
                        help="approximate cruise speed in m/s (default 2.0)")
    parser.add_argument("--tolerance", type=float, default=0.6,
                        help="distance (m) that counts as reaching the LAST "
                             "target (default 0.6)")
    parser.add_argument("--corner-radius", type=float, default=1.5,
                        help="distance (m) at which earlier targets count as "
                             "reached, so the drone flies through them "
                             "instead of stopping (default 1.5)")
    parser.add_argument("--stop-margin", type=float, default=0.5,
                        help="stay this far (m) short of unmapped space "
                             "(default 0.5)")
    parser.add_argument("--turn-in-place", type=float, default=100.0,
                        help="stop and turn first if the heading is off by "
                             "more than this many degrees (default 100)")
    parser.add_argument("--no-governor", dest="governor", action="store_false",
                        help="do not slow down before turns (always aim a "
                             "full lookahead ahead, as before)")
    parser.add_argument("--corner-speed", type=float, default=1.0,
                        help="speed (m/s) to slow to at a 90 degree turn "
                             "(default 1.0)")
    parser.add_argument("--turn-speed", type=float, default=0.5,
                        help="speed (m/s) to slow to where the path "
                             "reverses, 150 degrees or more (default 0.5)")
    parser.add_argument("--waypoint-speed", type=float, default=1.0,
                        help="speed (m/s) when reaching a target whose next "
                             "direction is not known, as in --explore "
                             "(default 1.0)")
    parser.add_argument("--plan-decel", type=float, default=1.2,
                        help="how gently to plan slowing down, in m/s^2 "
                             "(default 1.2; PX4 itself can brake harder)")
    parser.add_argument("--plan-accel", type=float, default=1.5,
                        help="how quickly to speed up again after a turn or "
                             "a stop, in m/s^2 (default 1.5)")
    parser.add_argument("--hold-yaw", action="store_true",
                        help="never turn to face the direction of travel "
                             "(only for a 360-degree lidar)")
    parser.add_argument("--max-range", type=float, default=10.0,
                        help="ignore lidar hits beyond this many metres")
    parser.add_argument("--map-rate", type=float, default=5.0,
                        help="scans added to the map per second (default 5)")
    parser.add_argument("--ray-stride", type=int, default=3,
                        help="use every Nth lidar ray (default 3)")
    parser.add_argument("--max-tilt", type=float, default=None,
                        help="skip scans when roll/pitch exceeds this many "
                             "degrees (default: computed from altitude and "
                             "max range)")
    parser.add_argument("--scan-delay", type=float, default=0.05,
                        help="seconds between a scan being taken and it "
                             "arriving here (default 0.05)")
    parser.add_argument("--replan-period", type=float, default=0.3,
                        help="seconds between path checks (default 0.3)")
    parser.add_argument("--reoptimize-period", type=float, default=4.0,
                        help="replan from scratch at least this often, even "
                             "if the current path is still free (default 4)")
    parser.add_argument("--target-timeout", type=float, default=120.0,
                        help="give up on a target after this many seconds")
    parser.add_argument("--stall-timeout", type=float, default=15.0,
                        help="give up if no progress for this many seconds")
    parser.add_argument("--no-path-timeout", type=float, default=6.0,
                        help="give up if no safe path exists for this long")
    parser.add_argument("--accel", type=float, default=None,
                        help="limit how quickly the drone may speed up or "
                             "slow down, in m/s^2 (default: no limit). Warning: "
                             "a low limit makes it stop later, possibly "
                             "inside the safety margin")
    parser.add_argument("--yaw-rate", type=float, default=70.0,
                        help="fastest the heading may turn, in degrees per "
                             "second (default 70)")
    parser.add_argument("--no-shaping", action="store_true",
                        help="send raw setpoints without smoothing (the "
                             "previous behaviour, to compare flight logs)")
    parser.add_argument("--no-log", dest="log", action="store_false",
                        help="do not write the flight log CSV")
    parser.add_argument("--log-dir", type=Path,
                        default=Path(__file__).resolve().with_name("flight_logs"),
                        help="folder for flight logs (default flight_logs/ "
                             "next to this script)")
    parser.add_argument("--log-file", type=Path,
                        help="exact log file name (default: a new file in "
                             "--log-dir named by date and time)")
    parser.add_argument("--log-rate", type=float, default=10.0,
                        help="flight log rows per second (default 10)")
    parser.add_argument("--occupied-resistance", type=float, default=0.5,
                        help="how strongly a wall resists being erased by "
                             "beams that graze past it, 0-1 (default 0.5; "
                             "1 = original behaviour, lower = walls erase "
                             "more slowly)")
    parser.add_argument("--map-file", type=Path, default=DEFAULT_MAP_FILE,
                        help="where the map is saved between runs "
                             f"(default {DEFAULT_MAP_FILE.name} next to this "
                             "script)")
    parser.add_argument("--no-load-map", dest="load_map", action="store_false",
                        help="start with an empty map even if a saved one "
                             "exists (a new one is still saved at the end)")
    parser.add_argument("--no-save-map", dest="save_map", action="store_false",
                        help="do not write the map file at the end")
    parser.add_argument("--map-decay", type=float, default=0.6,
                        help="how much to trust a loaded map, 0-1 (default "
                             "0.6). Lower lets fresh scans overrule it "
                             "faster; 1 trusts it fully")
    parser.add_argument("--map-origin-tol", type=float, default=1.0,
                        help="ignore the saved map if this run's start point "
                             "is more than this many metres from the one it "
                             "was made at (default 1.0)")
    parser.add_argument("--use-file-obstacles", action="store_true",
                        help="also use the obstacles listed in mission.yaml")
    parser.add_argument("--show-map", action="store_true",
                        help="print the map after every target")
    parser.add_argument("--address", help="PX4 address (default: from mission.yaml)")
    parser.add_argument("--topic", help="Gazebo scan topic (default: auto-detect)")
    parser.add_argument("--origin", help="start point as LAT,LON (default: PX4 home)")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--timeout", type=float, default=10.0)
    return parser


def main():
    args = build_parser().parse_args()

    if min(args.altitude, args.speed, args.map_rate, args.replan_period,
           args.log_rate, args.yaw_rate) <= 0 or (
               args.accel is not None and args.accel <= 0):
        sys.exit("ERROR: --altitude, --speed, --map-rate, --replan-period, "
                 "--log-rate, --yaw-rate and --accel must be positive.")
    if args.ray_stride < 1:
        sys.exit("ERROR: --ray-stride must be at least 1.")
    if not 0.0 < args.map_decay <= 1.0:
        sys.exit("ERROR: --map-decay must be above 0 and at most 1.")
    if not 0.0 < args.occupied_resistance <= 1.0:
        sys.exit("ERROR: --occupied-resistance must be above 0 and at most 1.")
    if min(args.corner_speed, args.turn_speed, args.waypoint_speed,
           args.plan_decel, args.plan_accel) <= 0:
        sys.exit("ERROR: --corner-speed, --turn-speed, --waypoint-speed, "
                 "--plan-decel and --plan-accel must be positive.")
    if args.explore and args.target:
        sys.exit("ERROR: use either --explore or --target, not both.")
    if args.min_frontier < 1 or args.explore_timeout <= 0:
        sys.exit("ERROR: --min-frontier must be at least 1 and "
                 "--explore-timeout positive.")

    try:
        config = load_yaml(args.config)
        planner = build_planner(config)
        if not args.use_file_obstacles:
            planner.clear_obstacles()
        if not args.address:
            args.address = (config.get("connection") or {}).get("address")

        if args.explore:
            targets = []  # chosen by the drone itself
        elif args.target:
            targets = [parse_target(t) for t in args.target]
        else:
            raw = config.get("targets")
            if not raw:
                raise ConfigError(
                    "No --target given and mission.yaml has no `targets`."
                )
            targets = [load_point(e, f"Target {i}") for i, e in enumerate(raw, 1)]
    except (ValueError, ConfigError) as error:
        sys.exit(f"ERROR: {error}")
    args.address = args.address or "udpin://0.0.0.0:14540"

    for i, t in enumerate(targets, start=1):
        if not planner.is_point_free(t):
            sys.exit(f"ERROR: target {i} ({t.north_m}, {t.east_m}) is outside "
                     "the flight area or too close to a known obstacle.")

    cfg = planner.config
    grid = ResistantGrid(
        cfg.min_north_m, cfg.max_north_m, cfg.min_east_m, cfg.max_east_m,
        resolution_m=cfg.resolution_m, max_range_m=args.max_range,
        occupied_resistance=args.occupied_resistance,
    )
    planner.set_occupancy_grid(grid)

    try:
        asyncio.run(run(args, planner, grid, targets))
    except (LidarError, RuntimeError, TimeoutError) as error:
        sys.exit(f"ERROR: {error}")
    except KeyboardInterrupt:
        print("\nInterrupted.")


if __name__ == "__main__":
    main()
