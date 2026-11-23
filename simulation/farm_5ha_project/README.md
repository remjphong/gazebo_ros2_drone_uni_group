# Nông trại 5 ha cho Gazebo Sim và PX4 SITL

World dùng tên riêng `farm_5ha_project`. Giữ gốc địa lý, thửa 200 × 250 m, 500 ô cây, hạ tầng tưới, nhà thiết bị, bồn nước, đường, cầu, cây ven ruộng và bãi đáp chính từ bản SDF giao Khánh. Bổ sung sân kiểm thử ở phía nam ngoài thửa canh tác, bãi đáp phụ, vật cản cao và trạm cảm biến tham chiếu.

## Mở world

Sau khi giải nén gói vào thư mục riêng, chạy từ thư mục chứa README:

```bash
bash scripts/run_world.sh view
```

Đóng lượt Gazebo xem trước rồi chạy một trong hai cấu hình PX4. Cần cài sẵn Gazebo Sim và PX4 SITL tại `~/PX4-Autopilot`. Đặt biến `PX4_DIR` nếu cài PX4 ở đường dẫn khác.

```bash
bash scripts/run_world.sh x500 main
```

```bash
bash scripts/run_world.sh lidar east
```

Bản LiDAR dùng target PX4 `gz_x500_lidar_2d`. Script đặt link tạm cho world riêng và kiểm tra trước khi dùng đường dẫn đã tồn tại. Script chỉ khởi động mô phỏng. Sau khi khởi động hãy xác nhận tên world, vị trí spawn và dữ liệu cảm biến.

## Tọa độ để tích hợp

Dùng tọa độ tuyệt đối trong world theo ENU, mét, (x = Đông, y = Bắc, z = lên). Gốc world và home của PX4 là hai mốc khác nhau. Khi đổi waypoint thành offset so với PX4 home, trừ tọa độ home thực tế tương ứng. Nếu test dùng độ cao tương đối, cũng cần trừ cao độ home. Không đưa trực tiếp tọa độ world vào YAML Bắc/Đông tương đối.

- **Quỳnh** dùng `config/scenarios.json` cho điểm xuất phát, vùng phủ mẫu, điểm quan sát và hai ca vật cản. `collision_geometry.csv` là các hộp bao theo trục world của collision. Lọc theo cao độ lập tuyến và dùng trường `model` để loại mặt đất khi dựng bản đồ ngang.
- **Ngô** dùng `crop_ground_truth.csv` đối chiếu ID, lớp và tọa độ tâm 500 ô cây. Có 366 ô khỏe, 94 ô bệnh nhẹ, 40 ô bệnh nặng. Các lớp là nhãn mô phỏng từ world gốc. Camera cố định nhìn xuống `crop_severe_r03_c16`. LiDAR cố định nhìn về phía vật cản cao trong sân thử.
- **Khánh** dùng cùng ID ca cho tuyến phủ, tiếp cận, không có đường, không có mục tiêu và năng lượng thấp. Ca không có đường có nghĩa trong vùng lập tuyến giới hạn, vì vậy cần dùng đúng ROI ghi kèm. Ca pin thấp dùng mock ở tầng điều phối. Ghi nhận vùng tưới, điều kiện phun và chính sách năng lượng thuộc bộ điều phối.
- **Phong** dùng bãi đáp chính `(-119, -151)` hoặc bãi đáp phụ `(54, -165)`. Trước khi bay, đối chiếu pose thực tế và mốc home với world. Waypoint trong file dữ liệu là điểm tham chiếu cho lập tuyến và chưa được chạy tự động khi mở world. Tính và kiểm tra tuyến đi, tuyến về cùng vùng đệm cho từng ca.

## Kiểm thử topic cảm biến

Trong terminal khác khi Gazebo đang chạy:

```bash
gz topic -l | grep -E '/farm_5ha/reference/(scan|image)|/world/farm_5ha_project/.*/scan'
timeout 10s gz topic -i -t /farm_5ha/reference/scan
timeout 10s gz topic -i -t /farm_5ha/reference/image
timeout 10s gz topic -e -n 1 -t /farm_5ha/reference/scan > /tmp/farm_5ha_reference_scan.txt
timeout 10s gz topic -e -n 1 -t /farm_5ha/reference/image > /tmp/farm_5ha_reference_image.txt
```

Hai topic reference là cảm biến đặt cố định trong world, dùng để kiểm tra đường dữ liệu và nhãn mô phỏng. Nếu ca bay cần camera gắn drone, chọn model PX4 có camera và kiểm tra topic của model đó. Lượt xem world và khám phá topic vẫn dùng được khi chưa có model camera gắn drone.

## Phạm vi xác minh

Đã phân tích XML, kiểm tra 500 ID cây, không có URI tài nguyên ngoài trong world, xuất 2.607 hình học collision và đối chiếu các đoạn tuyến phủ/tuyến vòng mẫu theo mặt cắt cao độ và hộp bao. Ca đường thẳng cắt vật cản cao, vách chặn nối hai biên của ROI và các đầu mút trong ROI đã được đối chiếu hình học.

Ảnh `preview.png` là sơ đồ tạo từ dữ liệu, không phải ảnh chụp Gazebo. Runtime Gazebo/PX4, bản tin sensor, quỹ đạo thực thi và tích hợp thành viên cần chạy trên máy SITL.

Tham chiếu biến môi trường PX4: [Gazebo Simulation](https://docs.px4.io/main/en/sim_gazebo_gz/index).
