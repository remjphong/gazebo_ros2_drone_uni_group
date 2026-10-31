"""Occupancy grid built from lidar rays, in the planner's north/east frame.

Each cell keeps a "log-odds" score. A lidar hit pushes its cell towards
occupied; every cell the beam passed through on the way pushes towards free.
Repeated scans from different spots therefore correct each other.

Cells never seen are UNKNOWN, and the planner treats unknown space as free
(it can only avoid what it has actually seen).
"""

import math

L_OCC = 0.85          # added to a cell where the beam ended on something
L_FREE = 0.4          # subtracted from cells the beam passed through
L_MIN, L_MAX = -4.0, 4.0
OCC_THRESHOLD = 0.5   # above this a cell counts as occupied


def _bresenham(r0, c0, r1, c1):
    """All (row, col) cells on the line from (r0, c0) to (r1, c1)."""
    cells = []
    dr, dc = abs(r1 - r0), abs(c1 - c0)
    sr = 1 if r1 > r0 else -1
    sc = 1 if c1 > c0 else -1
    err = dc - dr
    r, c = r0, c0
    while True:
        cells.append((r, c))
        if r == r1 and c == c1:
            return cells
        e2 = 2 * err
        if e2 > -dr:
            err -= dr
            c += sc
        if e2 < dc:
            err += dc
            r += sr


class OccupancyGrid:
    def __init__(
        self,
        min_north_m,
        max_north_m,
        min_east_m,
        max_east_m,
        resolution_m=0.5,
        max_range_m=10.0,
    ):
        if resolution_m <= 0 or max_range_m <= 0:
            raise ValueError("resolution_m and max_range_m must be positive.")
        if max_north_m <= min_north_m or max_east_m <= min_east_m:
            raise ValueError("Grid bounds are invalid.")

        self.min_north_m = min_north_m
        self.min_east_m = min_east_m
        self.resolution_m = resolution_m
        self.max_range_m = max_range_m
        self.rows = int(math.floor((max_north_m - min_north_m) / resolution_m)) + 1
        self.cols = int(math.floor((max_east_m - min_east_m) / resolution_m)) + 1

        self.log_odds = [0.0] * (self.rows * self.cols)
        self.seen = [False] * (self.rows * self.cols)
        self._version = 0
        self._inflated_cache = {}

    # ---- coordinates -------------------------------------------------
    def to_cell(self, north_m, east_m):
        return (
            round((north_m - self.min_north_m) / self.resolution_m),
            round((east_m - self.min_east_m) / self.resolution_m),
        )

    def in_grid(self, row, col):
        return 0 <= row < self.rows and 0 <= col < self.cols

    def _idx(self, row, col):
        return row * self.cols + col

    # ---- updating ----------------------------------------------------
    def _add(self, row, col, delta):
        i = self._idx(row, col)
        self.log_odds[i] = max(L_MIN, min(L_MAX, self.log_odds[i] + delta))
        self.seen[i] = True

    def update(self, origin_north_m, origin_east_m, rays):
        """rays: iterable of (end_north_m, end_east_m, is_hit).

        Beams longer than max_range_m are cut at max_range_m and treated as
        "nothing seen": far readings are less accurate, so they only clear
        free space and never mark obstacles.
        """
        o_row, o_col = self.to_cell(origin_north_m, origin_east_m)

        for end_n, end_e, is_hit in rays:
            dn, de = end_n - origin_north_m, end_e - origin_east_m
            dist = math.hypot(dn, de)
            if dist > self.max_range_m:
                scale = self.max_range_m / dist
                end_n = origin_north_m + dn * scale
                end_e = origin_east_m + de * scale
                is_hit = False

            e_row, e_col = self.to_cell(end_n, end_e)
            cells = _bresenham(o_row, o_col, e_row, e_col)

            for row, col in cells[:-1]:
                if self.in_grid(row, col):
                    self._add(row, col, -L_FREE)
            if self.in_grid(e_row, e_col):
                self._add(e_row, e_col, L_OCC if is_hit else -L_FREE)

        self._version += 1
        self._inflated_cache.clear()

    # ---- queries -----------------------------------------------------
    def is_occupied(self, row, col):
        return self.in_grid(row, col) and self.log_odds[self._idx(row, col)] > OCC_THRESHOLD

    def occupied_cells(self):
        return [
            (i // self.cols, i % self.cols)
            for i, v in enumerate(self.log_odds)
            if v > OCC_THRESHOLD
        ]

    def inflated(self, margin_m):
        """Set of cells within margin_m of an occupied cell (cached)."""
        key = round(margin_m, 6)
        if key in self._inflated_cache:
            return self._inflated_cache[key]

        reach = margin_m + self.resolution_m / 2.0
        k = int(math.ceil(reach / self.resolution_m))
        blocked = set()
        for row, col in self.occupied_cells():
            for dr in range(-k, k + 1):
                for dc in range(-k, k + 1):
                    if math.hypot(dr, dc) * self.resolution_m <= reach:
                        blocked.add((row + dr, col + dc))

        self._inflated_cache[key] = blocked
        return blocked

    def is_blocked(self, north_m, east_m, margin_m):
        """True if the point is within margin_m of something the lidar saw."""
        return self.to_cell(north_m, east_m) in self.inflated(margin_m)


# ----------------------------------------------------------------------
# Text picture (no plotting library needed)
# ----------------------------------------------------------------------
def render_ascii(grid, margin_m, path=None, drone=None, target=None):
    """1 character = 1 m.  # seen obstacle, + safety margin, . seen free,
    (blank) unknown, * planned path, D drone, T target. North is up."""
    block = max(1, round(1.0 / grid.resolution_m))
    inflated = grid.inflated(margin_m)
    n_rows = (grid.rows + block - 1) // block
    n_cols = (grid.cols + block - 1) // block
    canvas = [[" "] * n_cols for _ in range(n_rows)]

    for br in range(n_rows):
        for bc in range(n_cols):
            has_occ = has_inf = has_seen = False
            for r in range(br * block, min((br + 1) * block, grid.rows)):
                for c in range(bc * block, min((bc + 1) * block, grid.cols)):
                    i = grid._idx(r, c)
                    if grid.log_odds[i] > OCC_THRESHOLD:
                        has_occ = True
                    elif (r, c) in inflated:
                        has_inf = True
                    elif grid.seen[i]:
                        has_seen = True
            canvas[br][bc] = "#" if has_occ else "+" if has_inf else "." if has_seen else " "

    def mark(north, east, ch):
        r, c = grid.to_cell(north, east)
        if grid.in_grid(r, c):
            canvas[r // block][c // block] = ch

    if path:
        for a, b in zip(path, path[1:]):
            length = math.hypot(b[0] - a[0], b[1] - a[1])
            steps = max(1, int(length / 0.25))
            for s in range(steps + 1):
                t = s / steps
                mark(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), "*")
    if drone:
        mark(drone[0], drone[1], "D")
    if target:
        mark(target[0], target[1], "T")

    lines = []
    for br in range(n_rows - 1, -1, -1):
        north = grid.min_north_m + br * block * grid.resolution_m
        lines.append(f"{north:5.0f} |" + "".join(canvas[br]))
    axis = [" "] * n_cols
    labels = []
    for bc in range(0, n_cols, 5):
        east = grid.min_east_m + bc * block * grid.resolution_m
        text = f"{east:.0f}"
        for j, ch in enumerate(text):
            if bc + j < n_cols:
                axis[bc + j] = ch
    lines.append("      +" + "-" * n_cols)
    lines.append("       " + "".join(axis) + "   (east, m)")
    return "\n".join(lines)
