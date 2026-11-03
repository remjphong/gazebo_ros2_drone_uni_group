"""Frontier exploration: finding the edges of what the lidar has seen."""

import math

from occupancy_grid import OCC_THRESHOLD


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
