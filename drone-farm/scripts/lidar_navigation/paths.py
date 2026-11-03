"""Path geometry, and planning the speed along a path (slow down before turns)."""

import math

from path_planning import Point2D

from .flight import angle_diff


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
