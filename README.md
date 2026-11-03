# Drone Farm Simulation

A PX4 SITL + Gazebo crop-field drone that flies using only its 2D lidar. It
builds an obstacle map as it flies, plans paths around obstacles, remembers the
map between runs, and can explore an unknown field by itself.

The scanning of crops (insects, diseases) with the RGB camera is a separate
piece of work; see "For the inspection / coverage work" below.

## Requirements

- Ubuntu
- PX4-Autopilot
- Gazebo
- Python 3 and the packages in `requirements.txt` (MAVSDK gRPC, PyYAML)

## Setup

Create and activate a Python virtual environment, then install the
dependencies from `requirements.txt`.

## Simulation

The project uses a custom PX4/Gazebo drone equipped with a 2D LiDAR
and an RGB camera for obstacle detection and crop inspection.

For details on the custom model, PX4 airframe registration, sensor
topics, and camera verification, see
[Custom PX4/Gazebo Drone: 2D LiDAR + RGB Camera](docs/custom_px4_gazebo_drone.md).

Start PX4 SITL and Gazebo with the project's world:

```bash
cd ~/PX4-Autopilot
PX4_GZ_WORLD=obstacle_course make px4_sitl gz_x500_lidar_rgb     # lidar + camera
PX4_GZ_WORLD=obstacle_course make px4_sitl gz_x500_lidar_2d      # lidar only
```

## Project structure

```
config/mission.yaml            flight area, safety margin, targets, connection address
docs/                          notes, e.g. the custom drone model
scripts/
  online_navigation.py         entry point: run this
  lidar_navigation/            the navigation code (table below)
  analyze_flight_log.py        explains height sags from the flight log
  lidar_scan.py                reads and parses Gazebo lidar scans
  lidar_to_map.py              connects to PX4, GPS <-> local north/east offsets
  occupancy_grid.py            the occupancy grid (obstacle map)
  path_planning.py             A* planner on the grid
  plan_route.py                loads mission.yaml and builds the planner
  plan_from_lidar.py           helpers that turn scans into rays and targets
  yaw_test.py                  manual test of turning in flight
  test_connection.py           checks the connection to PX4
requirements.txt               Python dependencies
```

`scripts/lidar_navigation/`:

| Module | What it does |
|---|---|
| `flight.py` | Drone helpers, pose tracking, the setpoint controller |
| `lidar.py` | The Gazebo lidar stream |
| `mapping.py` | Map saving and loading, wall-erosion grid, background mapper |
| `paths.py` | Path geometry and speed planning (slow down before turns) |
| `exploration.py` | Finding the edges of what has been seen (frontiers) |
| `navigator.py` | Flying to targets and exploring (`Navigator`) |
| `mission.py` | Takeoff, running a flight, landing |
| `reporting.py` | The timing summary and the CSV flight log |
| `cli.py` | Command-line options |

## Run

With the simulator up and no other MAVSDK script running:

```bash
python3 scripts/online_navigation.py --explore       # explore the field, then return home
python3 scripts/online_navigation.py --target 8,8    # fly to one point (north,east in metres)
python3 scripts/online_navigation.py                 # fly the targets in config/mission.yaml
```

`python3 scripts/online_navigation.py --help` lists every option. The defaults
are meant to be used as they are; most runs need no extra flags.

## Configuration

The navigation reads these parts of `config/mission.yaml`:

| Key | Used for |
|---|---|
| `connection.address` | where to connect to PX4 |
| `targets` | the points to fly to when no `--target` or `--explore` is given |
| `planner` | flight area (`min/max_north_m`, `min/max_east_m`), grid `resolution_m`, `safety_margin_m` |
| `obstacles` | only with `--use-file-obstacles` (normally the lidar finds obstacles itself) |

All positions are north/east offsets in metres from the drone's start point.
The flight area must match the `flight_zone` box in `obstacle_course.sdf`.
Flight speed and altitude are set with `--speed` and `--altitude`, not in the
file. The `mission:` and `safety:` sections are left over from the old waypoint
flow and are no longer used.

## Files written by a flight

- `scripts/online_map.json`: the map, saved at the end of every run and loaded
  at the start of the next. It is ignored if the start point or grid settings
  changed. Use `--no-load-map` for a fresh start.
- `scripts/flight_logs/last_flight.csv`: the flight log of the latest flight.
  It is one file, overwritten by every flight, so logs do not pile up. Copy it
  elsewhere to keep one.

Analyze the latest flight:

```bash
python3 scripts/analyze_flight_log.py
```

It lists sags below the target height and what was happening before them.

## Expected result

The drone connects, takes off, flies to the targets (or explores) while mapping
with the lidar, returns to the start, lands, saves the map, and prints a timing
summary.

## For the inspection / coverage work

The lidar navigation is a safety layer: it flies to any point without hitting
anything, using only what the lidar has confirmed to be free. A coverage planner
can sit on top of it:

- `path_planning.py` and `occupancy_grid.py` are the planner and the map.
- `Navigator.fly_target(index, count, target, final, still_wanted=None,
  next_target=None)` in `lidar_navigation/navigator.py` flies to one point
  (a `Point2D`: north and east in metres) with obstacle avoidance. It returns
  `"reached"`, `"skipped"` (target unusable), `"failed"` or `"obsolete"`
  (`still_wanted()` turned False). Pass `next_target` so the drone slows down
  correctly for the turn.
- By default the drone turns to face its direction of travel. `--hold-yaw`
  keeps the heading fixed, which is a starting point for a sideways-looking
  camera, but proper control of the heading along a lane is not built yet.

Not built yet (the coverage work will need them):

- pausing in place and resuming, for example to hover for a photo
- a position, heading and time stamp for each photo
- generating lanes between the crop rows from the saved map

## Known issues

- **Height sags.** The drone sometimes loses height (up to about 0.8 m) when it
  turns while moving fast. It happens in about 3 to 4% of a flight. The
  analyzer shows when.
- **High hover thrust on the camera model.** PX4 reports a hover thrust of about
  0.8 on `x500_lidar_rgb` (the usual value is about 0.5), so the drone has little
  thrust to spare. `MPC_THR_HOVER 0.8` is set in the airframe
  `4022_gz_x500_lidar_rgb` as a stopgap. The cause has not been found; the next
  step is to read `hover_thrust_estimate` on the lidar-only model for a baseline.
- **Exploration differs from run to run.** Targets depend on what the lidar has
  already seen, so two flights are never exactly the same. Judge a change over
  at least three flights.
- **Leftover code.** `plan_route.py` and `plan_from_lidar.py` still contain old
  command-line tools that nothing uses.
- **No automated tests are in the repository yet.**
