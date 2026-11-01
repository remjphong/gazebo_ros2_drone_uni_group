# Decision Module A - Drone Farm

Module A đảm nhận chức năng Decision Making.

## 1. Trách nhiệm

- Nhận trạng thái drone.
- Nhận dữ liệu định vị và waypoint từ C.
- Nhận cảnh báo cảm biến từ D.
- Phân tích tình trạng và chọn hành động.
- Gửi quyết định cho B.
- Yêu cầu C lập kế hoạch lại khi đường đi không hợp lệ.

A không trực tiếp điều khiển motor, không gửi PX4 setpoint
và không tự tính toán đường đi A*.

## 2. ROS 2 topics

### Input

- /drone/status
- /navigation/status
- /environment/status
- /motion/status

### Output

- /decision/command
- /navigation/replan_request
- /decision/status

Các topic hiện dùng std_msgs/msg/String chứa JSON.

## 3. Build

```bash
cd ~/drone_ws

source /opt/ros/jazzy/setup.bash

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install

source install/setup.bash
