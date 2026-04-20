# Startup Sequence

## Prerequisites

### Software
All non-optional software must be installed before running the startup sequence.
See [installation guide](installation.md) for full instructions.

### Hardware
- D435 plugged in via USB-A on the Jetson
- U2D2 with all 6 motors connected via USB-A (/dev/ttyUSB0)
- MPU6050 wired to Jetson GPIO I2C bus 7:
  - VCC → Pin 1 (3.3V)
  - GND → Pin 6 (GND)
  - SDA → Pin 3
  - SCL → Pin 5

## Step 1 - Stereocamera D435
Open a new terminal and run:
```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_sync:=true \
  depth_module.depth_profile:=640x480x60 \
  rgb_camera.color_profile:=640x480x60
```
Wait for: `RealSense Node Is Up!`

**Node: realsense2_camera_node**
Drives the Intel RealSense D435 camera. Publishes RGB image, depth image and
point cloud. The point cloud must be enabled at runtime as done
in Step 2. For more details see the [troubleshooting guide](troubleshooting.md).

## Step 2 - Enable Point Cloud
Open a new terminal and run:
```bash
ros2 param set /camera/camera pointcloud__neon_.enable true
```
Wait for: `Set parameter successful`

## Step 3 - Motor Driver
Open a new terminal and run:
```bash
python3 ~/dynamixel_driver.py
```
Wait for: `Robot ready!`

**Node: [dynamixel_driver](../code/dynamixel_driver.py)**
Custom node that links Nav2 and the dynamixel motors. Subscribes to `/cmd_vel`
and converts linear/angular velocity commands into left/right motor speeds using
the Dynamixel SDK. Also reads motor encoder positions and publishes `/odom` so
Nav2 knows where the robot is. Broadcasts the `odom -> base_link` TF transform.

## (Optional) Manual Control
Requires `ros-humble-teleop-twist-keyboard` — see [installation guide](installation.md).

To drive the rover manually, complete steps 1-3, connect a keyboard to the Jetson and do the following.
Open a new terminal and run:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Controls:
- `i` — forward
- `,` — backward
- `j` — turn left
- `l` — turn right
- `k` — stop
- `q/z` — increase/decrease speed

Skip to Step 4 for full autonomous navigation.

## Step 4 - Static Transform
Open a new terminal and run:
```bash
ros2 run tf2_ros static_transform_publisher 0.0 0.0 0.2 0.0 0.0 0.0 base_link camera_link
```

**Node: static_transform_publisher**
Tells ROS2 where the camera is physically mounted on the robot by publishing
a fixed `base_link -> camera_link` transform. Without this RTAB-Map cannot
relate what the camera sees to where the robot is.
- `0.0 0.0 0.2` — (x, y, z) camera position in meters
- `0.0 0.0 0.0` — (roll, pitch, yaw) camera rotation in radians
  - all zeros means the camera faces the same direction as the robot
- `base_link` — the parent frame (robot base)
- `camera_link` — the child frame (camera)

Change z to the actual height of your camera above the robot base in meters.
See [robot parameters](robot_parameters.md) for more details.

## Step 5 - RTAB-Map
Open a new terminal and run:
```bash
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

**Node: rtabmap**
The main SLAM node. Takes RGB images, depth images and odometry as input and
builds a 3D map of the environment. Also localizes the robot within that map.
Publishes the map on `/map` which Nav2 uses for path planning.

**Node: rtabmap_viz**
Visualizes the RTAB-Map output in RViz2. Shows the current map, point cloud,
camera trajectory and loop closure detections.

RViz2 opens automatically. Set it up before continuing:
- Fixed Frame → `map`
- Add → By topic → `/map` → Map → OK
- Add → By topic → `/camera/camera/depth/color/points` → PointCloud2 → OK

## Step 6 - Nav2
Open a new terminal and run:
```bash
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml \
  use_sim_time:=false
```
Wait for: `Managed nodes are active`

**Node: nav2 (multiple nodes)**
A collection of nodes that together handle autonomous navigation:
- **controller_server** — sends velocity commands to `/cmd_vel`
- **planner_server** — calculates the optimal path from A to B
- **behavior_server** — handles recovery behaviors when the robot gets stuck
- **bt_navigator** — coordinates all Nav2 nodes using a behavior tree
- **costmap** — builds a grid showing where obstacles are
- **velocity_smoother** — smooths out jerky velocity commands
