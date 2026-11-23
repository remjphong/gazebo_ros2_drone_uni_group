# Drone Farm Simulation

Mô phỏng điều khiển bay PX4 SITL trong world nông trại 5 ha. Hướng dẫn dưới đây dùng nhánh `B-phong`.

## 1. Clone hoặc cập nhật

Lần đầu:

```bash
git clone --branch B-phong https://github.com/remjphong/gazebo_ros2_drone_uni_group.git
cd gazebo_ros2_drone_uni_group
```

Nếu đã clone, vào thư mục repo rồi chạy:

```bash
git fetch origin B-phong
git switch B-phong
git merge --ff-only origin/B-phong
```

## 2. Terminal A — khởi động mô phỏng

Cần Ubuntu, Python có `venv`, Gazebo Sim và PX4 SITL đã cài. Máy Phong đã chạy với Gazebo Sim 10.5.0 và PX4 `d57d7f3b11c044b86195a12cfb933d17fac85b34`. Script tìm PX4 tại `~/PX4-Autopilot`. Nếu PX4 nằm chỗ khác, đặt `PX4_DIR` trước khi chạy.

Từ thư mục gốc repo, đóng phiên PX4/Gazebo cũ rồi chạy:

```bash
bash simulation/farm_5ha_project/scripts/run_world.sh x500 main
```

Chờ Gazebo mở đúng world `farm_5ha_project` và PX4 báo sẵn sàng cất cánh. Giữ Terminal A chạy.

## 3. Terminal B — chạy thử 4 waypoint

Mở terminal thứ hai ở thư mục gốc của **cùng checkout**. Ca này dùng cấu hình `drone-farm/config/mission.yaml` và không cần ROS 2. Khối lệnh tạo môi trường Python và lưu log trong thư mục tạm được in ra ở `RUN_DIR`.

```bash
bash <<'SITL'
set -Eeuo pipefail
checkout="$PWD"
test -f "$checkout/simulation/farm_5ha_project/worlds/farm_5ha_project.sdf"
git rev-parse HEAD
run="$(mktemp -d)"
printf 'RUN_DIR=%s\n' "$run"
python3 -m venv "$run/.venv"
python="$run/.venv/bin/python"
"$python" -m pip install mavsdk-grpc==4.0.5 PyYAML==6.0.3
if [[ -n "$(ss -H -ltn 'sport = :50051')" || -n "$(ss -H -lun 'sport = :14540')" ]]; then
    printf 'BLOCKED: đóng client/server MAVSDK cũ rồi thử lại.\n'
    exit 1
fi
timeout 8s gz topic -l >"$run/topics.txt"
grep -Fx '/world/farm_5ha_project/clock' "$run/topics.txt"
server="$("$python" -c 'from pathlib import Path; import mavsdk_grpc; print(Path(mavsdk_grpc.__file__).resolve().parent / "bin/mavsdk_server")')"
"$server" -p 50051 --sysid 245 --compid 190 udpin://0.0.0.0:14540 >"$run/mavsdk-server.log" 2>&1 &
server_pid=$!
runner=""
cleanup() {
    if [[ -n "$runner" ]]; then rm -f -- "$runner"; fi
    kill -TERM "$server_pid" 2>/dev/null || true
    for _ in {1..5}; do
        kill -0 "$server_pid" 2>/dev/null || break
        sleep 1
    done
    kill -KILL "$server_pid" 2>/dev/null || true
    wait "$server_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
runner="$("$python" - "$checkout" <<'PY'
import sys
from pathlib import Path

root = Path(sys.argv[1])
script = root / "drone-farm/scripts/waypoint_mission.py"
source = script.read_text(encoding="utf-8")
old = "    drone = System()"
if source.count(old) != 1:
    raise SystemExit("BLOCKED: controller không khớp phiên bản trong hướng dẫn.")
source = source.replace(old, '    drone = System(mavsdk_server_address="127.0.0.1", port=50051)', 1)
import tempfile

with tempfile.NamedTemporaryFile(dir=script.parent, prefix=".waypoint-handoff-", suffix=".py", delete=False) as temp:
    copy = Path(temp.name)
compile(source, str(copy), "exec")
copy.write_text(source, encoding="utf-8")
print(copy)
PY
)"
for ((i=0; i<120; i++)); do
    [[ -n "$(ss -H -ltn 'sport = :50051')" ]] && break
    kill -0 "$server_pid" 2>/dev/null || break
    sleep 0.25
done
if [[ -z "$(ss -H -ltn 'sport = :50051')" ]]; then
    cat "$run/mavsdk-server.log" >&2
    exit 1
fi
timeout --kill-after=5s 300s "$python" -u "$runner" 2>&1 | tee "$run/mission.log"
grep -Fx 'MISSION_PROGRESS=4/4' "$run/mission.log"
grep -Fx 'LANDING_CONFIRMED=1' "$run/mission.log"
grep -Fx 'DISARM_CONFIRMED=1' "$run/mission.log"
grep -Fx 'WAYPOINT_MISSION=PASS' "$run/mission.log"
printf 'HANDOFF_LOG=%s\n' "$run/mission.log"
SITL
```

Khi thành công, log có `MISSION_PROGRESS=4/4`, xác nhận hạ cánh và disarm, rồi in `WAYPOINT_MISSION=PASS`. Gửi cho nhóm SHA commit hiện ở đầu lệnh, môi trường chạy, `mission.log` và lỗi đầu tiên nếu có.

Xem [hướng dẫn world và kiểm tra LiDAR](simulation/farm_5ha_project/README.md) và [kết quả kiểm thử của Phong](docs/phong-sitl-handoff.md).
