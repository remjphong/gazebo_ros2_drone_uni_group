#!/usr/bin/env python3
"""Find out why the drone loses height, from a flight log written by
online_navigation.py.

    python3 analyze_flight_log.py flight_logs/flight_20261031_213000.csv
    python3 analyze_flight_log.py LOG.csv --threshold 0.2 --plot sag.png

A "sag" is a stretch where the drone is more than --threshold metres below
its target height. For every sag the script looks at the 2.5 s before it and
checks which conditions were present (hard braking, strong tilt, fast
turning, Python stalling, PX4 leaving Offboard mode, ...). A condition only
counts as a likely cause if it shows up before the sags much more often than
it does during the rest of the flight ("lift"): something that is always
true cannot explain a sag that happens only sometimes.

Only the Python standard library is needed (matplotlib is optional, for
--plot).
"""

import argparse
import csv
import math
import sys
from pathlib import Path

LEAD_S = 2.5        # how far before a sag to look
AFTER_S = 0.5       # and how far past its start
MERGE_GAP_S = 1.0   # dips closer together than this are one sag
SMOOTH_S = 0.3      # time span used for accelerations and turn rates
STOP_STATES = {"braking", "turning", "wait-map", "creep", "settle",
               "look-around", "no-path", "alt-hold"}
TEXT_COLUMNS = {"wall", "state", "mode"}


def angle_diff(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


# ----------------------------------------------------------------------
# Loading
# ----------------------------------------------------------------------
def load(path):
    rows, skipped = [], 0
    with open(path, newline="") as handle:
        for raw in csv.DictReader(handle):
            try:
                row = {k: (v if k in TEXT_COLUMNS else float(v))
                       for k, v in raw.items() if k is not None}
            except (TypeError, ValueError):
                skipped += 1
                continue
            rows.append(row)
    if skipped:
        print(f"(Skipped {skipped} damaged row(s).)")
    return rows


def add_derived(rows):
    """Adds tilt, accel, brake and yaw_rate to every row (in place)."""
    j = 0
    for i, row in enumerate(rows):
        row["tilt"] = max(abs(row["roll"]), abs(row["pitch"]))
        while j < i and row["t"] - rows[j]["t"] > SMOOTH_S * 1.5:
            j += 1
        dt = row["t"] - rows[j]["t"]
        if i == j or dt < SMOOTH_S * 0.5:
            row["accel"] = row["brake"] = row["yaw_rate"] = 0.0
            continue
        old = rows[j]
        row["accel"] = math.hypot(row["vn"] - old["vn"], row["ve"] - old["ve"]) / dt
        row["brake"] = max(0.0, (old["speed"] - row["speed"]) / dt)
        row["yaw_rate"] = abs(angle_diff(row["heading"], old["heading"])) / dt


# ----------------------------------------------------------------------
# Sag events
# ----------------------------------------------------------------------
def find_sags(rows, threshold):
    events, current = [], None
    for i, row in enumerate(rows):
        if row["alt_err"] > threshold:
            if current is not None and row["t"] - rows[current["end"]]["t"] <= MERGE_GAP_S:
                current["end"] = i
            else:
                if current is not None:
                    events.append(current)
                current = {"start": i, "end": i}
        elif current is not None and row["t"] - rows[current["end"]]["t"] > MERGE_GAP_S:
            events.append(current)
            current = None
    if current is not None:
        events.append(current)

    for ev in events:
        span = rows[ev["start"]: ev["end"] + 1]
        peak = max(span, key=lambda r: r["alt_err"])
        ev["onset_t"] = rows[ev["start"]]["t"]
        ev["end_t"] = rows[ev["end"]]["t"]
        ev["peak"] = peak["alt_err"]
        ev["peak_t"] = peak["t"]
    return events


def window_features(rows, t_from, t_to):
    """Summary of the conditions inside [t_from, t_to]."""
    w = [r for r in rows if t_from <= r["t"] <= t_to]
    if not w:
        return None
    modes = {r["mode"] for r in w}
    states = {r["state"] for r in w}
    return {
        "n": len(w),
        "tilt": max(r["tilt"] for r in w),
        "speed": max(r["speed"] for r in w),
        "brake": max(r["brake"] for r in w),
        "accel": max(r["accel"] for r in w),
        "yaw_rate": max(r["yaw_rate"] for r in w),
        "descent": max(r["vd"] for r in w),
        "gap": max(r["send_gap_ms"] for r in w),
        "lag": max(r["loop_lag_ms"] for r in w),
        "pose_age": max(r["pose_age_ms"] for r in w),
        "left_offboard": any(m not in ("OFFBOARD", "?") for m in modes),
        "busy": any(r["busy_map"] or r["busy_plan"] or r["busy_frontier"] for r in w),
        "states": states,
        "state_now": w[-1]["state"],
    }


CONDITIONS = [
    # key, test, description
    ("brake", lambda f: f["brake"] > 1.5,
     "hard braking (slowing down faster than 1.5 m/s^2)"),
    ("tilt", lambda f: f["tilt"] > 12.0,
     "strong tilt (roll or pitch above 12 degrees)"),
    ("turn", lambda f: f["yaw_rate"] > 40.0,
     "fast turning (above 40 degrees per second)"),
    ("stopstate", lambda f: bool(f["states"] & STOP_STATES),
     "a stop/turn state (braking, turning, wait-map, creep, settle, look-around)"),
    ("fast", lambda f: f["speed"] > 1.5,
     "flying fast (above 1.5 m/s)"),
    ("stall", lambda f: f["lag"] > 80.0,
     "Python stalling (the program woke up more than 80 ms late)"),
    ("gap", lambda f: f["gap"] > 250.0,
     "setpoint stream gap (more than 250 ms between setpoints)"),
    ("stale", lambda f: f["pose_age"] > 300.0,
     "stale position data (more than 300 ms old)"),
    ("offboard", lambda f: f["left_offboard"],
     "PX4 left Offboard mode"),
    ("busy", lambda f: f["busy"],
     "a map/planner/frontier job was running"),
]

EXPLANATIONS = {
    "offboard": (
        "PX4 dropped out of Offboard mode (its failsafe takes over and the "
        "drone sinks). The setpoint stream was interrupted for too long.",
        "Make sure setpoints keep flowing: move the heavy work (planning, "
        "frontier search) out of the process, or reduce how often it runs."),
    "gap": (
        "The setpoint stream to PX4 stalls while Python is busy, so PX4 "
        "briefly flies on stale commands.",
        "Send setpoints from a separate process or thread, and cut the CPU "
        "load of the planner and frontier search."),
    "stall": (
        "The program freezes for a moment (the GIL is held by the planner "
        "or the map update), which delays control.",
        "Move planning/mapping to a separate process, or lower --map-rate "
        "and --ray-stride."),
    "stale": (
        "Position telemetry arrives late, so the controller reacts to old "
        "data.",
        "Raise the telemetry rate or reduce load on the program."),
    "brake": (
        "The drone is slowed abruptly (setpoint snapped back to its own "
        "position at speed), PX4 pitches back hard, and some lift is lost.",
        "Brake gradually: ramp the setpoint lead down, or use velocity "
        "setpoints with an acceleration limit. Lowering PX4's MPC_ACC_HOR "
        "and MPC_TILTMAX_AIR also helps."),
    "tilt": (
        "The drone leans hard (fast acceleration or braking), which costs "
        "vertical thrust.",
        "Limit horizontal acceleration: a lower --speed, a gradual speed "
        "ramp, or lower MPC_ACC_HOR / MPC_TILTMAX_AIR in PX4."),
    "turn": (
        "Fast yawing coincides with the sag.",
        "Turn more slowly (limit the yaw rate, MPC_YAWRAUTO_MAX) or only "
        "turn while hovering."),
    "stopstate": (
        "The sag follows the drone stopping or turning in place.",
        "Check which of these states it is (see the table) and smooth that "
        "transition."),
    "fast": (
        "Sags only happen at higher speed.",
        "Reduce --speed or add a speed ramp."),
    "busy": (
        "A planner/map job runs at the same moment. Probably a side "
        "effect of Python load rather than the cause itself.",
        "See the stall/gap rows; if they are also high, reduce the load."),
}


# ----------------------------------------------------------------------
# Analysis
# ----------------------------------------------------------------------
def baseline_windows(rows, events, step=1.0):
    """Windows of the same length away from every sag."""
    t_start, t_end = rows[0]["t"], rows[-1]["t"]
    # A window overlaps a sag's causes if it starts anywhere from LEAD_S
    # before the sag to LEAD_S after it ends; keep all of those out.
    bad = [(ev["onset_t"] - LEAD_S - 1.0, ev["end_t"] + LEAD_S + 1.0)
           for ev in events]
    out, t = [], t_start + LEAD_S
    while t < t_end - AFTER_S:
        if not any(a <= t <= b for a, b in bad):
            f = window_features(rows, t - LEAD_S, t + AFTER_S)
            if f:
                out.append(f)
        t += step
    return out


def analyse(rows, threshold):
    events = find_sags(rows, threshold)
    for ev in events:
        ev["features"] = window_features(
            rows, ev["onset_t"] - LEAD_S, ev["onset_t"] + AFTER_S
        )
    base = baseline_windows(rows, events)

    results = []
    for key, test, text in CONDITIONS:
        hits = sum(1 for ev in events if ev["features"] and test(ev["features"]))
        n_ev = sum(1 for ev in events if ev["features"])
        n_base = len(base)
        base_hits = sum(1 for f in base if test(f))
        ev_rate = hits / n_ev if n_ev else 0.0
        base_rate = base_hits / n_base if n_base else 0.0
        lift = (ev_rate / base_rate) if base_rate > 0 else (math.inf if ev_rate > 0 else 0.0)
        results.append({"key": key, "text": text, "hits": hits, "n_ev": n_ev,
                        "ev_rate": ev_rate, "base_rate": base_rate,
                        "lift": lift, "n_base": n_base})
    return events, results


def likely_causes(results):
    """Conditions that precede most sags and are unusual otherwise."""
    out = []
    for r in results:
        if r["n_ev"] == 0:
            continue
        strong = r["ev_rate"] >= 0.6 and (r["lift"] >= 1.5 or r["base_rate"] <= 0.1)
        if strong and r["key"] in EXPLANATIONS:
            out.append(r)
    # Most specific evidence first: rarest condition, then highest rate; on a
    # tie the more fundamental cause (the order of EXPLANATIONS) comes first.
    order = {key: i for i, key in enumerate(EXPLANATIONS)}
    out.sort(key=lambda r: (round(r["base_rate"], 2), -round(r["ev_rate"], 2),
                            order[r["key"]]))
    return out


# ----------------------------------------------------------------------
# Output
# ----------------------------------------------------------------------
def fmt_lift(r):
    if r["lift"] == math.inf:
        return "never otherwise"
    return f"x{r['lift']:.1f}"


def report(rows, events, results, threshold):
    duration = rows[-1]["t"] - rows[0]["t"]
    worst = max(r["alt_err"] for r in rows)
    below = sum(1 for r in rows if r["alt_err"] > threshold) / len(rows)
    print(f"Flight log: {len(rows)} rows, {duration:.0f} s, "
          f"states seen: {', '.join(sorted({r['state'] for r in rows}))}")
    modes = sorted({r["mode"] for r in rows})
    print(f"PX4 flight modes seen: {', '.join(modes)}")
    print(f"Worst sag below the target height: {worst:.2f} m; "
          f"{100 * below:.1f}% of the time below it by more than {threshold:g} m")

    if not events:
        print(f"\nNo sag deeper than {threshold:g} m in this log. If you saw "
              "one in Gazebo, lower --threshold, or the drop is not in PX4's "
              "own height estimate (see the note at the end).")
        return

    print(f"\n{len(events)} sag(s) deeper than {threshold:g} m:\n")
    head = (f"{'#':>2} {'at':>7} {'len':>5} {'peak':>5} {'state':<11} "
            f"{'speed':>5} {'brake':>5} {'tilt':>4} {'yaw/s':>5} {'down':>5} "
            f"{'lag':>4} {'gap':>4}  mode")
    print(head)
    print("-" * len(head))
    for i, ev in enumerate(events, 1):
        f = ev["features"]
        if not f:
            continue
        mode = next((r["mode"] for r in rows if r["t"] >= ev["onset_t"]), "?")
        print(f"{i:>2} {ev['onset_t']:>6.1f}s {ev['end_t'] - ev['onset_t']:>4.1f}s "
              f"{ev['peak']:>4.2f}m {f['state_now']:<11} {f['speed']:>5.1f} "
              f"{f['brake']:>5.1f} {f['tilt']:>4.0f} {f['yaw_rate']:>5.0f} "
              f"{f['descent']:>5.1f} {f['lag']:>4.0f} {f['gap']:>4.0f}  {mode}")
    print("\n(at = seconds from the start of the log; speed m/s, brake m/s^2, "
          "tilt deg, down = fastest descent m/s,\n lag and gap in ms, all "
          f"within {LEAD_S:g} s before the sag)\n")

    print("What was present before the sags, compared with the rest of the flight:\n")
    print(f"{'condition':<72} {'before sags':>12} {'otherwise':>10}  lift")
    for r in sorted(results, key=lambda r: (-r["ev_rate"], r["base_rate"])):
        if r["n_ev"] == 0:
            continue
        print(f"{r['text']:<72} {r['hits']:>5}/{r['n_ev']:<5} "
              f"{100 * r['base_rate']:>8.0f}%  {fmt_lift(r)}")

    causes = likely_causes(results)
    print("\nVerdict:")
    if not causes:
        print("  Inconclusive: no condition clearly separates the sags from "
              "normal flight.\n  Either the sags are too few (try a longer "
              "flight or --threshold 0.15), or the drop is\n  not real in "
              "PX4's height estimate (see the note below).")
    else:
        for n, r in enumerate(causes[:3], 1):
            why, fix = EXPLANATIONS[r["key"]]
            tag = "most likely" if n == 1 else "also present"
            print(f"  {n}. [{tag}] {r['text']}: before {r['hits']} of "
                  f"{r['n_ev']} sags, {100 * r['base_rate']:.0f}% of the time "
                  f"otherwise.\n     {why}\n     Fix: {fix}")
        if len(causes) > 3:
            print(f"  (+{len(causes) - 3} weaker match(es) not shown)")

    print("\nNote: this log holds PX4's own estimate of height. If Gazebo shows "
          "a drop that does not appear here\n(alt_err stays near 0), the "
          "problem is the height estimate, not the control. Compare the time "
          "of a visible drop\n(the `wall` column is the clock time) with the "
          "rows around it.")


def plot(rows, events, path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("(matplotlib is not installed; skipping --plot.)")
        return
    t = [r["nav_t"] for r in rows]
    fig, ax = plt.subplots(5, 1, figsize=(12, 11), sharex=True)
    ax[0].plot(t, [r["alt_err"] for r in rows], color="C3")
    ax[0].set_ylabel("below target (m)")
    ax[1].plot(t, [r["speed"] for r in rows], label="speed")
    ax[1].plot(t, [r["vd"] for r in rows], label="descent rate")
    ax[1].set_ylabel("m/s")
    ax[1].legend(loc="upper right")
    ax[2].plot(t, [r["tilt"] for r in rows], color="C2")
    ax[2].set_ylabel("tilt (deg)")
    ax[3].plot(t, [r["yaw_rate"] for r in rows], color="C4")
    ax[3].set_ylabel("yaw rate (deg/s)")
    ax[4].plot(t, [r["loop_lag_ms"] for r in rows], label="loop lag")
    ax[4].plot(t, [r["send_gap_ms"] for r in rows], label="setpoint gap")
    ax[4].set_ylabel("ms")
    ax[4].legend(loc="upper right")
    ax[4].set_xlabel("time since navigation started (s)")
    offset = rows[0]["nav_t"] - rows[0]["t"]
    for ev in events:
        for a in ax:
            a.axvspan(ev["onset_t"] + offset, ev["end_t"] + offset,
                      color="red", alpha=0.15)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print(f"Plot saved to {path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("log", type=Path, help="flight log CSV")
    parser.add_argument("--threshold", type=float, default=0.25,
                        help="how far below the target height counts as a "
                             "sag, in metres (default 0.25)")
    parser.add_argument("--plot", type=Path, help="also save a plot (PNG)")
    args = parser.parse_args()

    try:
        rows = load(args.log)
    except OSError as error:
        sys.exit(f"ERROR: cannot read {args.log}: {error}")
    if len(rows) < 20:
        sys.exit(f"ERROR: only {len(rows)} usable rows in {args.log}; "
                 "that is too short to analyse.")
    missing = [c for c in ("t", "nav_t", "alt_err", "vn", "ve", "vd", "speed",
                           "roll", "pitch", "heading", "state", "mode",
                           "send_gap_ms", "pose_age_ms", "loop_lag_ms",
                           "busy_map", "busy_plan", "busy_frontier")
               if c not in rows[0]]
    if missing:
        sys.exit("ERROR: the log is missing columns: " + ", ".join(missing))

    add_derived(rows)
    events, results = analyse(rows, args.threshold)
    report(rows, events, results, args.threshold)
    if args.plot:
        plot(rows, events, args.plot)


if __name__ == "__main__":
    main()
