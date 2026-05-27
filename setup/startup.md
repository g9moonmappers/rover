# Startup Sequence

## Prerequisites

### Software
All non-optional software must be installed before running the startup sequence.
See [installation guide](installation.md) for full instructions.

### Hardware
- D435 plugged into a **USB 3.0 port** on the Jetson — check for `Device USB type: 3.2` in Step 1 output.
  If you get `2.1` the JetsonHacks kernel modules need to be reinstalled — see [troubleshooting guide](troubleshooting.md).
- U2D2 with all 6 motors connected via USB-A (`/dev/ttyUSB0`)
- All 6 Dynamixel motors powered on
- Arduino Mega connected via USB (`/dev/ttyACM0`) with `ekf.ino` flashed
  - MPU6050 wired to Arduino I2C (SDA/SCL)
  - BU04 UWB modules connected to Arduino Serial2 (pins 16/17)

## Step 1 - Camera
Open a new terminal and run:
```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_sync:=true \
  depth_module.depth_profile:=640x480x60 \
  rgb_camera.color_profile:=640x480x60
```
Wait for: `RealSense Node Is Up!` and `Device USB type: 3.2`

**Node: realsense2_camera_node**
Drives the Intel RealSense D435 camera. Publishes RGB image, depth image and
point cloud. The point cloud must be enabled at runtime as done in Step 2.
For more details see the [troubleshooting guide](troubleshooting.md).

## Step 2 - Enable Point Cloud
Open a new terminal and run:
```bash
ros2 param set /camera/camera pointcloud__neon_.enable true
```
Wait for: `Set parameter successful`

## Step 3 - Motor Driver and EKF
Open a new terminal and run:
```bash
python3 ~/robot.py
```
Wait for: `Robot ready!`

**Node: robot**
Reads wheel encoder positions from all 6 Dynamixel motors and computes wheel
velocities. Sends velocities to the Arduino over USB serial (`/dev/ttyACM0`).
The Arduino runs an Extended Kalman Filter fusing:
- Wheel odometry from the Dynamixel encoders
- IMU gyroscope from the MPU6050
- UWB position from BU04 trilateration (when anchors are configured)

The filtered position is sent back to the Jetson and published as `/odom`
for RTAB-Map and Nav2. Also broadcasts the `odom -> base_link` TF transform.

## (Optional) Manual Control
Requires `ros-humble-teleop-twist-keyboard` — see [installation guide](installation.md).

To drive the rover manually, complete Steps 1-3, connect a keyboard to the Jetson and run:
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

## Step 4 - Static Transforms
Open a new terminal and run:
```bash
ros2 run tf2_ros static_transform_publisher 0.0 0.0 0.27 0.0 0.0 0.0 base_link camera_link &
ros2 run tf2_ros static_transform_publisher 0 0 0 -1.5707963 0 -1.5707963 camera_link camera_color_optical_frame &
ros2 run tf2_ros static_transform_publisher 0 0 0 -1.5707963 0 -1.5707963 camera_link camera_depth_optical_frame &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 camera_link camera_color_frame &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 camera_link camera_depth_frame &
```

**Node: static_transform_publisher**
Tells ROS2 where the camera is physically mounted on the robot by publishing
a fixed `base_link -> camera_link` transform. Without this RTAB-Map cannot
relate what the camera sees to where the robot is.
- `0.0 0.0 0.27` — (x, y, z) camera position in meters — measure the actual
  height from the bottom of the robot to the camera lens and update this value
- `0.0 0.0 0.0` — (roll, pitch, yaw) camera rotation in radians
- `base_link` — the parent frame (robot base)
- `camera_link` — the child frame (camera)

## Step 5 - RTAB-Map
Open a new terminal and run:
```bash
rm ~/.ros/rtabmap.db
ros2 launch rtabmap_launch rtabmap.launch.py \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/depth/image_rect_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  frame_id:=base_link \
  odom_frame_id:=odom \
  approx_sync:=true \
  visual_odometry:=false \
  icp_odometry:=false \
  Odom/MinInliers:=3 \
  Vis/MinInliers:=3 \
  Odom/ResetCountdown:=1 \
  Rtabmap/LoopThr:=0.5 \
  Mem/NotLinkedNodesKept:=false \
  map_always_update:=true \
  rviz:=true
```

RViz2 opens automatically. Set it up before continuing:
- Fixed Frame → `map`
- Add → By topic → `/rtabmap/map` → Map → OK
- Add → By topic → `/camera/camera/depth/color/points` → PointCloud2 → OK

**Node: rtabmap**
The main SLAM node. Takes RGB images, depth images and wheel odometry as input
and builds a 3D map of the environment. Also localizes the robot within that map.
Publishes the map on `/rtabmap/map` which Nav2 uses for path planning.

**Node: rtabmap_viz**
Visualizes the RTAB-Map output in RViz2. Shows the current map, point cloud,
camera trajectory and loop closure detections.

## Step 6 - Map Relay
Open a new terminal and run:
```bash
ros2 run topic_tools relay /rtabmap/map /map
```

Forwards the RTAB-Map occupancy grid from `/rtabmap/map` to `/map` which
Nav2 and the frontier explorer listen to.

## Step 7 - Nav2
Open a new terminal and run:
```bash
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml \
  use_sim_time:=false
```
Wait for: `Managed nodes are active`

**Node: nav2 (multiple nodes)**
A collection of nodes that together handle autonomous navigation:
- **controller_server** — calculates velocity commands and sends them to `/cmd_vel` which `robot.py` listens to and executes on the Dynamixel motors
- **planner_server** — calculates the optimal path from A to B
- **behavior_server** — handles recovery behaviors when the robot gets stuck
- **bt_navigator** — coordinates all Nav2 nodes using a behavior tree
- **costmap** — builds a grid showing where obstacles are using the point cloud from the camera
- **velocity_smoother** — smooths out jerky velocity commands before they reach the motors

## Step 8 - Initial Spin
Open a new terminal and run:
```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.3}}" &
sleep 22
kill %1
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

Spins the robot 360° to build an initial map in all directions before
autonomous exploration starts. Wait for the full spin to complete before
moving to Step 9. Adjust the sleep value based on how long a full rotation
takes on your robot.

## Step 9 - Autonomous Exploration
Open a new terminal and run:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch frontier_exploration_ros2 frontier_explorer.launch.py \
  params_file:=$(ros2 pkg prefix frontier_exploration_ros2)/share/frontier_exploration_ros2/config/params.yaml
```

**Node: frontier_explorer**
Autonomous exploration node. Looks at the current map from RTAB-Map, finds
unexplored frontier areas and sends navigation goals to Nav2 to explore them.
Uses MRTSP ordering to plan the most efficient exploration path. The robot
drives itself around the room until the full map is built.
