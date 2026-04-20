# Startup Sequence

## Prerequisites
- NVIDIA Jetson Orin Nano Super with Jetpack 6.2.1 (rev.1) installed (https://www.youtube.com/watch?v=BaRdpSXU6EM)
- Intel RealSense D435 plugged in to Jetson via an USB-A to USB-C 3.0 cable
- U2D2 with all 6 Dynamixel XC430 motors connected and daisy chained, plugged in via USB
- MPU6050 wired to Jetson GPIO header: ()
  - VCC → Pin 1 (3.3V)
  - GND → Pin 6 (GND)
  - SDA → Pin 3
  - SCL → Pin 5

## Terminal 1 - Camera
```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  enable_sync:=true \
  enable_gyro:=true \
  enable_accel:=true \
  unite_imu_method:=1 \
  depth_module.depth_profile:=640x480x60 \
  rgb_camera.color_profile:=640x480x60
```
Wait for: `RealSense Node Is Up!`

## Terminal 2 - Enable Point Cloud
```bash
source /opt/ros/humble/setup.bash
ros2 param set /camera/camera pointcloud__neon_.enable true
```
Wait for: `Set parameter successful`

## Terminal 3 - Motor Driver
```bash
source /opt/ros/humble/setup.bash
python3 ~/dynamixel_driver.py
```
Wait for: `Robot ready!`

## Terminal 4 - Static Transform
```bash
source /opt/ros/humble/setup.bash
ros2 run tf2_ros static_transform_publisher 0.0 0.0 0.2 0.0 0.0 0.0 base_link camera_link
```

## Terminal 5 - RTAB-Map
```bash
source /opt/ros/humble/setup.bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/depth/image_rect_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  frame_id:=base_link \
  odom_frame_id:=odom \
  approx_sync:=false \
  visual_odometry:=false \
  icp_odometry:=false \
  Odom/MinInliers:=8 \
  rviz:=true
```

RViz2 opens automatically. Set it up before continuing:
- Fixed Frame → `map`
- Add → By topic → `/map` → Map → OK
- Add → By topic → `/camera/camera/depth/color/points` → PointCloud2 → OK

## Terminal 6 - Nav2
```bash
source /opt/ros/humble/setup.bash
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml \
  use_sim_time:=false
```
Wait for: `Managed nodes are active`

## Terminal 7 - Autonomous Exploration
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch explore_lite explore.launch.py
```
