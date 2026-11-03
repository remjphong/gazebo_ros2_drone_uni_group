"""The Navigator: flying to targets and exploring, using the map."""

import asyncio
import math
import time

from occupancy_grid import render_ascii
from path_planning import Point2D

from .constants import CONTROL_PERIOD_S
from .exploration import (
    EXPLORE_DONE_RADIUS_M,
    EXPLORE_MAX_FAILURES,
    EXPLORE_SKIP_RADIUS_M,
    cell_to_point,
    coverage,
    find_frontiers,
)
from .flight import angle_diff
from .paths import (
    free_extent,
    leg_end_speed,
    path_length,
    point_along,
    speed_limit_ahead,
    trim_path,
)


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
