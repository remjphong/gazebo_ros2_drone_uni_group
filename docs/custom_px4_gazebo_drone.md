## Custom PX4/Gazebo Drone: 2D LiDAR + RGB Camera

### Overview

A custom PX4/Gazebo drone model, `x500_lidar_rgb`, was created by combining the existing PX4 X500 drone with a 2D LiDAR sensor and an RGB camera.

The original `x500_lidar_2d` model remains unchanged and can still be launched separately.

The custom model is intended to support:
- **LiDAR-based navigation and obstacle detection**
- **RGB camera-based crop inspection**
- Future integration of camera and LiDAR data into the crop-drone mission pipeline

### 1. Custom Gazebo Model

The custom model is located in the PX4 Gazebo models directory:

```text
PX4-Autopilot/
└── Tools/
    └── simulation/
        └── gz/
            └── models/
                └── x500_lidar_rgb/
                    ├── model.config
                    └── model.sdf
```

The model combines:
- The PX4 X500 base drone
- The existing `lidar_2d_v2` sensor
- An OakD-Lite camera model with an RGB camera sensor

The camera is attached to the drone through a camera link and joint. The LiDAR remains attached through its existing LiDAR link and joint.

The model configuration identifies the custom model as `x500_lidar_rgb`.

### 2. PX4 Airframe Registration

A new PX4 airframe startup file was added:

```text
ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_x500_lidar_rgb
```

It was based on the existing `4013_gz_x500_lidar_2d` airframe, with the simulation model changed to:

```sh
PX4_SIM_MODEL=${PX4_SIM_MODEL:=x500_lidar_rgb}
```

The new airframe was also registered in:

```text
ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
```

PX4 was rebuilt so that the new airframe is included in the build output.

### 3. Launching the Custom Drone

From the PX4-Autopilot directory:

```bash
cd ~/PX4-Autopilot
PX4_GZ_WORLD=obstacle_course make px4_sitl gz_x500_lidar_rgb
```

This launches the custom drone in the `obstacle_course` Gazebo world.

To launch the original LiDAR-only drone instead:

```bash
PX4_GZ_WORLD=obstacle_course make px4_sitl gz_x500_lidar_2d
```

The model target can be changed without replacing the original model.

### 4. Gazebo Sensor Topics

The custom model was launched successfully, and its sensor topics were checked using Gazebo Transport.

List relevant topics:

```bash
gz topic -l | grep -Ei 'camera|image|depth|lidar|scan'
```

#### RGB Camera

RGB image topic:

```text
/world/obstacle_course/model/x500_lidar_rgb_0/link/camera_link/sensor/IMX214/image
```

Camera info topic:

```text
/world/obstacle_course/model/x500_lidar_rgb_0/link/camera_link/sensor/IMX214/camera_info
```

The RGB image topic uses the message type:

```text
gz.msgs.Image
```

#### 2D LiDAR

LiDAR scan topic:

```text
/world/obstacle_course/model/x500_lidar_rgb_0/link/link/sensor/lidar_2d_v2/scan
```

LiDAR point cloud topic:

```text
/world/obstacle_course/model/x500_lidar_rgb_0/link/link/sensor/lidar_2d_v2/scan/points
```

The camera and LiDAR topics are available while the simulation is running.

### 5. Checking the RGB Camera with Python

The Gazebo Transport Python binding was installed:

```bash
sudo apt install python3-gz-transport13
```

The binding can be checked with:

```bash
python3 -c "from gz.transport13 import Node; print('Gazebo Transport Python is available')"
```

The import succeeded. Protobuf duplicate-descriptor warnings were printed, but the final confirmation message appeared.

A Python subscriber was created to receive an image from the camera topic. The subscriber confirmed:

```text
Image: 1920 x 1080
Pixel format: 3
Data bytes: 6220800
```

The pixel format enum was checked and identified as:

```text
RGB_INT8
```

This confirms that the camera is publishing 1920 × 1080 RGB image data.

### 6. Camera Snapshot

A camera snapshot script, `camera_snapshot.py`, was prepared to receive a frame and save it as a PNG.

The intended capture directory is:

```text
drone-farm/
└── captures/
    └── camera_snapshot.png
```

Create the directory with:

```bash
mkdir -p ~/gazebo_ros2_drone_uni_group/drone-farm/captures
```

The snapshot script should save the received RGB frame to that directory. The PNG capture and visual inspection are the next steps.

## Camera Performance Optimization

The OakD-Lite RGB camera initially runs at **1920 × 1080 and 30 Hz**, which can significantly reduce Gazebo's real-time factor (RTF).

For a lighter simulation, reduce the RGB camera resolution and update rate in:

```bash
~/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite/model.sdf
```

Find the `IMX214` camera sensor and change its settings to:

```xml
<width>640</width>
<height>480</height>
<update_rate>10</update_rate>
```

This changes the RGB camera from **1920 × 1080 at 30 Hz** to **640 × 480 at 10 Hz**. Leave the `StereoOV7251` depth camera unchanged initially, so the effect of the RGB camera adjustment can be tested independently.

After saving the file, restart the simulation:

```bash
cd ~/PX4-Autopilot
PX4_GZ_WORLD=obstacle_course make px4_sitl gz_x500_lidar_rgb
```

Verify the new image dimensions by running the camera subscriber:

```bash
python3 camera_check.py
```

Expected output should include:

```text
Image: 640 x 480
```

Compare the Gazebo RTF before and after the change under similar simulation conditions. If performance is still poor, consider reducing the depth camera's update rate separately.

### Current Status

Completed:
- Created the custom `x500_lidar_rgb` Gazebo model.
- Combined the X500 drone, 2D LiDAR, and RGB camera.
- Registered the custom model as a PX4 airframe.
- Rebuilt PX4 with the new airframe.
- Launched the custom drone in the `obstacle_course` world.
- Verified that the RGB camera and LiDAR topics are available.
- Subscribed to the RGB camera topic using Python.
- Confirmed the camera resolution and `RGB_INT8` pixel format.

Next:
- Save and visually inspect a camera snapshot.
- Use the RGB camera stream for crop inspection.
- Integrate camera and LiDAR observations into the crop-drone mission workflow.
