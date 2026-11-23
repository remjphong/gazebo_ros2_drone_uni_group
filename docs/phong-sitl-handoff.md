# Bàn giao B — điều khiển bay PX4 SITL

[Xem hướng dẫn chạy thử từ bản clone](../README.md). Các kết quả Baylands dưới đây dùng world tích hợp sẵn trong PX4.

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

## Kiểm thử trên world nông trại 5 ha

World dùng `farm_5ha_project` từ gói `farm_5ha_project.zip`. Các kết quả dưới đây ghi nhận trên máy SITL của Phong.

| Ca | Kết quả |
| --- | --- |
| X500 ở bãi chính | 4/4 waypoint, hạ cánh và disarm. Sai số home 0,035 m. |
| Cảm biến tham chiếu | LiDAR 361 tia, 35 giá trị dương hữu hạn. RGB 640 × 480 có dữ liệu. |
| X500 LiDAR ở bãi phụ | Topic `/world/farm_5ha_project/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan` phát 1.080 tia. Mặt cản gần nhất phía trước 22,878 m. |
| Tuyến vòng tham chiếu, lượt đầu | Controller timeout ở 6/8 với giới hạn 240 giây thực. ULog `11_58_01.ulg` xác nhận PX4 hoàn tất và disarm. Cận dưới khoảng hở ngang theo quỹ đạo ước lượng là 4,138 m. |
| Cùng tuyến, lượt chạy lại | Controller theo dõi đủ 8/8, hạ cánh/disarm, mã thoát 0, sai số home 0,045 m. Thời lượng 314 giây thực, timeout thử 480 giây. ULog `12_10_32.ulg`. |

Hai lượt dùng tuyến vòng tham chiếu của world. Lượt 480 giây dùng cấu hình thử riêng. Ca lập tuyến từ scan live trên world nông trại và việc tái lập trên máy thành viên khác vẫn cần bằng chứng riêng.

Lượt tái lập world từ bản clone `097093f` trên máy Phong dùng bộ điều khiển và cấu hình tuyến thử đã lưu tại máy. Hoàn thành 8/8, hạ cánh/disarm, sai số home 0,048 m. ULog `12_48_35.ulg` ghi khoảng cách ngang nhỏ nhất tại mẫu đến hộp bao vật cản cao là 4,186 m. Ca chạy thử 4 waypoint trong README dùng controller và cấu hình có sẵn trong repo.
