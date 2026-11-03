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

# Everything now lives in the lidar_navigation package (scripts/lidar_navigation/). The
# names below are re-exported so that `import online_navigation` (for example
# yaw_test.py) keeps working exactly as before.
from lidar_navigation.constants import (
    SETPOINT_RATE_HZ,
    CONTROL_PERIOD_S,
    PX4_XY_P,
    MAP_FILE_VERSION,
    DEFAULT_MAP_FILE,
    SCRIPT_DIR,
)
from lidar_navigation.flight import (
    angle_diff,
    _wrap_180,
    wait_for_health,
    set_telemetry_rates,
    arm_and_takeoff,
    land_and_wait,
    PoseTracker,
    SetpointShaper,
    OffboardController,
)
from lidar_navigation.lidar import LidarStream
from lidar_navigation.paths import (
    path_length,
    trim_path,
    point_along,
    turn_angle_at,
    corner_speed_limit,
    path_end_direction,
    leg_end_speed,
    speed_limit_ahead,
    cell_seen,
    free_extent,
)
from lidar_navigation.exploration import (
    coverage,
    cell_to_point,
    find_frontiers,
    EXPLORE_SKIP_RADIUS_M,
    EXPLORE_DONE_RADIUS_M,
    EXPLORE_MAX_FAILURES,
)
from lidar_navigation.mapping import (
    _geometry,
    _same_geometry,
    save_grid,
    load_grid,
    persist_map,
    ResistantGrid,
    MapService,
    Mapper,
)
from lidar_navigation.reporting import Stats, LOG_COLUMNS, FlightLogger
from lidar_navigation.navigator import wait_still, Navigator
from lidar_navigation.mission import wait_for_altitude, wait_until, run
from lidar_navigation.cli import build_parser, main


if __name__ == "__main__":
    main()
