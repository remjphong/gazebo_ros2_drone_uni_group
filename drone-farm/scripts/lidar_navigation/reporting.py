"""The timing summary and the CSV flight log."""

import asyncio
import collections
import csv
import math
import time
from pathlib import Path

from .flight import angle_diff


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
