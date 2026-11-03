"""Settings shared by the other modules."""

from pathlib import Path


SETPOINT_RATE_HZ = 10.0


CONTROL_PERIOD_S = 0.1


# PX4's horizontal position gain (MPC_XY_P, default 0.95 1/s): in position
# mode it flies at about this many m/s per metre of distance to the setpoint.
PX4_XY_P = 0.95


MAP_FILE_VERSION = 1


SCRIPT_DIR = Path(__file__).resolve().parent.parent   # the scripts/ folder (this file is in scripts/lidar_navigation/)
DEFAULT_MAP_FILE = SCRIPT_DIR / "online_map.json"
