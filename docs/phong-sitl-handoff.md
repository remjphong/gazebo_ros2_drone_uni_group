# Bàn giao B — điều khiển bay PX4 SITL

## Chạy lại chuyến chuẩn Baylands/X500

Môi trường đã chạy trên Ubuntu 26.04.1, Python 3.14.4, ROS 2 Lyrical, Gazebo Sim 10.5.0 và PX4 commit `d57d7f3b11c044b86195a12cfb933d17fac85b34`. PX4 và Gazebo cần được cài trước. Ca waypoint dùng MAVSDK qua UDP 14540, không cần node ROS 2 để gửi waypoint.

Terminal A, từ bản PX4 đã cài:

```bash
cd "$HOME/PX4-Autopilot" &&
PX4_GZ_WORLD=baylands make px4_sitl gz_x500
```

World Baylands tích hợp trong PX4 đã được đối chiếu SHA-256 với `baylands.sdf` của repo (`18290387d36083ec948e69ce4f30b87cd08e947fc49f2363f3003f74d119c80c`).

Terminal B, dùng checkout riêng để chạy lại:

```bash
bash <<'SITL'
set -Eeuo pipefail
checkout="$(mktemp -d)"
git clone --branch B-phong --single-branch https://github.com/remjphong/gazebo_ros2_drone_uni_group.git "$checkout"
printf 'CHECKOUT=%s\n' "$checkout"
git -C "$checkout" rev-parse HEAD
python3 -m venv "$checkout/.venv"
python="$checkout/.venv/bin/python"
"$python" -m pip install mavsdk-grpc==4.0.5 PyYAML==6.0.3
if ss -H -lntu | awk '$5 ~ /:(14540|50051)$/ {print; used=1} END {exit !used}'; then
    printf 'BLOCKED: đóng client/server MAVSDK cũ rồi thử lại.\n'
    exit 1
fi
timeout 8s gz topic -l >"$checkout/topics.txt"
grep -Fx '/world/baylands/clock' "$checkout/topics.txt"
timeout --kill-after=5s 300s "$python" -u "$checkout/drone-farm/scripts/waypoint_mission.py" 2>&1 | tee "$checkout/mission.log"
grep -Fx 'MISSION_PROGRESS=4/4' "$checkout/mission.log"
grep -Fx 'LANDING_CONFIRMED=1' "$checkout/mission.log"
grep -Fx 'DISARM_CONFIRMED=1' "$checkout/mission.log"
grep -Fx 'WAYPOINT_MISSION=PASS' "$checkout/mission.log"
printf 'HANDOFF_LOG=%s\n' "$checkout/mission.log"
SITL
```

Ca chuẩn dùng `drone-farm/config/mission.yaml`. Script hiện đọc độ lệch Bắc/Đông, đơn vị mét, từ vị trí GPS ban đầu. Cao độ mục tiêu là độ cao tương đối, tính bằng mét. Trong ca phủ và tiếp cận dùng mock, phép đổi ENU là x → Đông, y → Bắc, z → cao độ và bản thử dùng PX4 home làm mốc. Chốt mốc với nhóm trước khi nhận tuyến mới.

## Bằng chứng đã ghi

| Ca thử | Kết quả | Phạm vi |
| --- | --- | --- |
| Waypoint chuẩn | 4/4, hạ cánh/disarm, sai số home 0,038 m | Controller trên nhánh B |
| Tuyến phủ và tiếp cận | 4/4 và 2/2, sai số home 0,240 m và 0,054 m | Tuyến và mục tiêu mock |
| Quỹ đạo tránh vật cản | 8/8, cận dưới 1,121 m, sai số home 0,040 m | SDF + LiDAR, vị trí ước lượng PX4, vùng đệm lập tuyến 1,40 m. Lượt đầu 0,889 m cận dưới nên FAIL ngưỡng 1 m |
| HOLD | Dừng ổn định sau 7,380 s, lệch tối đa 0,056 m trong 3 s, sau đó RTL | Quyết định NO_PATH mock. Lượt đo cũ lệch 0,589 m |
| RTL từ bên điều phối | RTL chỉ một lần, hạ cánh/disarm, sai số home 0,415 m | Node A thật nhận đầu vào giả lập, bộ nhận thử dùng `rtl_receiver.py` theo vòng đời một chuyến bay |
| Timeout và mất kết nối | Timeout 1,001 s, 0 lần upload/arm. Mất server gRPC trả UNAVAILABLE | Timeout dùng MAVSDK mock. Mất gRPC thử ở mặt đất |
| Sai số theo sự kiện waypoint | WP1 0,512 m, WP2 0,681 m, WP3 0,605 m, WP4 0,666 m. Tuổi mẫu lớn nhất 14,4 ms | Chuyến phủ bằng mock, đo lệch ngang bằng mẫu vị trí mới nhất nhận trước tiến độ. Chờ ngưỡng nghiệm thu của nhóm |

Log gốc và ULog nằm trên máy chạy SITL. Hồ sơ cuối cần lưu các log/video chính và dự phòng ở nơi nhóm truy cập được. Các ca SDF + LiDAR dùng world `obstacle_course` từ commit `8bdc64ad76fc170768cf97ddadbebd01f4646965` và model `x500_lidar_2d`.

## Mục bàn giao còn mở

- Chạy lại từ checkout sạch trên máy thành viên khác và ghi commit, log, người chạy.
- Nối bộ nhận RTL, telemetry và quy tắc chặn upload vào tiến trình B dùng chung. Kiểm tra với đầu vào giao theo [Issue #1](https://github.com/remjphong/gazebo_ros2_drone_uni_group/issues/1).
- Chốt và lưu log, video chính/dự phòng cùng Khánh. Camera cần kiểm thử nếu ca chung dùng nguồn ảnh.