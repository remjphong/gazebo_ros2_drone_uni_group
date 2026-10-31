"""Stage 1: read ONE 2D lidar scan from Gazebo and look at it.

Start the simulator with the lidar drone first, for example:
    PX4_GZ_WORLD=obstacle_course make px4_sitl gz_x500_lidar_2d

Then, from the project root:
    python3 scripts/lidar_scan.py

What it does:
  1. finds the lidar's Gazebo topic (or use --topic),
  2. grabs one scan with the `gz` command line tool (no extra Python packages),
  3. prints a summary (number of readings, nearest obstacle, ...),
  4. saves a picture (lidar_scan.png) of the points the lidar hit.

The picture is in the DRONE frame: x = straight ahead, y = to the left.
At spawn the x500 faces east, so x = east and y = north. If PX4's default
spawn (world origin) is used, the obstacles from config/mission.yaml are
drawn on top so you can check the scan against them.
"""

import argparse
import math
import re
import shutil
import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = PROJECT_ROOT / "config" / "mission.yaml"


class LidarError(Exception):
    pass


# ----------------------------------------------------------------------
# Talking to Gazebo
# ----------------------------------------------------------------------
def run_gz(args, timeout):
    if shutil.which("gz") is None:
        raise LidarError(
            "Could not find the 'gz' command. Is Gazebo installed, and is "
            "this terminal set up the same way as the one running the "
            "simulator?"
        )
    try:
        result = subprocess.run(
            ["gz", *args], capture_output=True, text=True, timeout=timeout
        )
    except subprocess.TimeoutExpired:
        raise LidarError(
            f"Timed out after {timeout} s waiting for: gz {' '.join(args)}\n"
            "Is the simulation running (not paused)?"
        ) from None

    if result.returncode != 0:
        raise LidarError(
            f"'gz {' '.join(args)}' failed:\n{result.stderr.strip()}"
        )
    return result.stdout


def find_scan_topic(timeout):
    topics = run_gz(["topic", "-l"], timeout).split()
    candidates = [t for t in topics if t.endswith("/scan")]
    lidar = [t for t in candidates if "lidar" in t] or candidates

    if not lidar:
        shown = "\n  ".join(topics[:30]) or "(none)"
        raise LidarError(
            "No lidar scan topic found. Did you start the simulator with "
            "gz_x500_lidar_2d?\nTopics seen:\n  " + shown
        )
    if len(lidar) > 1:
        print(f"Several scan topics found, using the first: {lidar[0]}")
    return lidar[0]


def grab_scan_text(topic, timeout):
    return run_gz(["topic", "-e", "-t", topic, "-n", "1"], timeout)


# ----------------------------------------------------------------------
# Parsing and conversion
# ----------------------------------------------------------------------
def _field(text, name, default=None):
    match = re.search(rf"^{name}:\s*(\S+)", text, re.M)
    return float(match.group(1)) if match else default


def parse_scan(text):
    """Parse the text form of a gz.msgs.LaserScan message."""
    ranges = [
        float(v) for v in re.findall(r"^\s*ranges:\s*(\S+)", text, re.M)
    ]
    if not ranges:
        preview = "\n".join(text.splitlines()[:25])
        raise LidarError(
            "The message had no 'ranges' values. First lines received:\n"
            + preview
        )

    angle_min = _field(text, "angle_min")
    angle_max = _field(text, "angle_max")
    if angle_min is None or angle_max is None:
        raise LidarError("The message had no angle_min / angle_max.")

    step = _field(text, "angle_step")
    if step is None and len(ranges) > 1:
        step = (angle_max - angle_min) / (len(ranges) - 1)

    return {
        "angle_min": angle_min,
        "angle_max": angle_max,
        "angle_step": step,
        "range_min": _field(text, "range_min", 0.0),
        "range_max": _field(text, "range_max", math.inf),
        "ranges": ranges,
    }


def scan_to_hits(scan):
    """Return (x_forward, y_left, range, bearing) for every real hit."""
    hits = []
    for i, r in enumerate(scan["ranges"]):
        if not math.isfinite(r):
            continue
        if r < scan["range_min"] or r >= scan["range_max"] - 1e-6:
            continue  # too close, or "nothing seen" (reported as max range)
        a = scan["angle_min"] + i * scan["angle_step"]
        hits.append((r * math.cos(a), r * math.sin(a), r, a))
    return hits


def print_summary(scan, hits, topic):
    n = len(scan["ranges"])
    print(f"Topic: {topic}")
    print(
        f"Readings: {n}, angle {math.degrees(scan['angle_min']):.0f} to "
        f"{math.degrees(scan['angle_max']):.0f} deg, "
        f"range {scan['range_min']:.1f} to {scan['range_max']:.1f} m"
    )
    print(f"Readings that hit something: {len(hits)} ({100 * len(hits) / n:.0f}%)")

    if not hits:
        print("Nothing in range. (Normal if the drone is far from everything.)")
        return

    nearest = min(hits, key=lambda h: h[2])
    print(
        f"Nearest hit: {nearest[2]:.2f} m at {math.degrees(nearest[3]):+.0f} deg "
        "(0 = straight ahead, + = left)"
    )

    sectors = {
        "ahead  (-45..45)": (-45, 45),
        "left   (45..135)": (45, 135),
        "right  (-135..-45)": (-135, -45),
    }
    for name, (lo, hi) in sectors.items():
        in_sector = [
            h[2] for h in hits if lo <= math.degrees(h[3]) < hi
        ]
        text = f"{min(in_sector):.2f} m" if in_sector else "clear"
        print(f"  closest {name}: {text}")


# ----------------------------------------------------------------------
# Picture
# ----------------------------------------------------------------------
def load_overlay(config_path):
    try:
        import yaml

        with Path(config_path).open("r", encoding="utf-8") as file:
            config = yaml.safe_load(file)
        return config.get("obstacles") or []
    except Exception as error:  # overlay is optional
        print(f"(No overlay: could not read obstacles: {error})")
        return []


def save_picture(hits, scan, output, overlay, show):
    try:
        import matplotlib

        if not show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle, Rectangle
    except ImportError:
        print("matplotlib is not installed, so no picture was made. "
              "Install it with: pip install matplotlib")
        return

    fig, ax = plt.subplots(figsize=(8, 7))

    # Known obstacles, assuming drone at the world origin facing east:
    # drone x (forward) = east_m, drone y (left) = north_m.
    for o in overlay:
        try:
            x, y = float(o["east_m"]), float(o["north_m"])
            if "radius_m" in o:
                ax.add_patch(Circle((x, y), float(o["radius_m"]),
                                    fc="#f4b8a8", ec="#cc3311", alpha=0.6))
            else:
                w, l = float(o["size_east_m"]), float(o["size_north_m"])
                ax.add_patch(Rectangle((x - w / 2, y - l / 2), w, l,
                                       fc="#b8dcb8", ec="#1a6b1a", alpha=0.6))
        except (KeyError, TypeError, ValueError):
            continue

    if hits:
        ax.scatter([h[0] for h in hits], [h[1] for h in hits],
                   s=6, color="#1f5fbf", label="lidar hits", zorder=3)

    ax.plot(0, 0, "ks", zorder=4)
    ax.annotate("drone", (0, 0), textcoords="offset points", xytext=(6, -12))
    ax.annotate("", xy=(2, 0), xytext=(0, 0),
                arrowprops=dict(arrowstyle="->", lw=2), zorder=4)

    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x: straight ahead (m)   [east at spawn]")
    ax.set_ylabel("y: to the left (m)   [north at spawn]")
    ax.set_title("One lidar scan (blue) vs known obstacles (shaded)")
    if hits:
        ax.legend(loc="upper right")
    fig.tight_layout()

    fig.savefig(output, dpi=110)
    print(f"Saved picture: {output}")
    if show:
        plt.show()


# ----------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--topic", help="Gazebo scan topic (default: auto-detect)")
    parser.add_argument("--timeout", type=float, default=10.0,
                        help="seconds to wait for Gazebo (default 10)")
    parser.add_argument("--output", default="lidar_scan.png",
                        help="picture file (default lidar_scan.png)")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG),
                        help="mission.yaml used for the obstacle overlay")
    parser.add_argument("--no-overlay", action="store_true")
    parser.add_argument("--no-plot", action="store_true")
    parser.add_argument("--show", action="store_true",
                        help="also open the picture in a window")
    args = parser.parse_args()

    try:
        topic = args.topic or find_scan_topic(args.timeout)
        print(f"Reading one scan from {topic} ...")
        scan = parse_scan(grab_scan_text(topic, args.timeout))
    except LidarError as error:
        sys.exit(f"ERROR: {error}")

    hits = scan_to_hits(scan)
    print_summary(scan, hits, topic)

    if not args.no_plot:
        overlay = [] if args.no_overlay else load_overlay(args.config)
        save_picture(hits, scan, args.output, overlay, args.show)


if __name__ == "__main__":
    main()
