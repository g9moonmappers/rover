# Rover

Documentation for all software used by bachelor group 9 (MoonMappers) at USN Kongsberg.

## Hardware
- **Compute:** NVIDIA Jetson Orin Nano Super Developer Kit (https://no.rs-online.com/web/p/processor-development-tools/2647384)
- **Camera:** Intel RealSense D435 (https://no.rs-online.com/web/p/depth-cameras/1720981)
- **Motors:** 6x Dynamixel XC430-W150-T (https://robotis.us/dynamixel-xc430-w150-t/)
- **Motor Controller:** Dynamixel Starter Set (https://www.generationrobots.com/en/403718-official-dynamixel-starter-set-eu.html)
- **IMU:** MPU6050 (on I2C bus 7)
- **Motor layout:**
  - Left side: IDs 1, 3, 5
  - Right side: IDs 2, 4, 6

## Software
- **OS:** JetPack 6.2.1 (rev.1) / Ubuntu 22.04
- **ROS2:** Humble
- **SLAM:** RTAB-Map + RViz2
- **Navigation:** Nav2
- **Camera driver:** realsense2_camera v4.57.7
- **Motor driver:** [dynamixel_driver.py](code/dynamixel_driver.py)
- **IMU driver:** ros2_mpu6050_driver (modified for I2C bus 7)

## ROS2 Nodes
### realsense2_camera_node
Drives the Intel RealSense D435 camera. Publishes RGB image, depth image,
point cloud and IMU data.

### dynamixel_driver ([code/dynamixel_driver.py](code/dynamixel_driver.py))
Custom made node that links Nav2 and the dynamixel motors. Subscribes to `/cmd_vel`
and converts linear/angular velocity commands into left/right motor speeds using
the Dynamixel SDK. Also reads motor encoder positions and publishes `/odom` so
Nav2 knows where the robot is. Broadcasts the `odom -> base_link` TF transform.

### static_transform_publisher
Tells ROS2 where the camera is physically mounted on the robot by publishing
a fixed `base_link -> camera_link` transform. Without this RTAB-Map cannot
relate what the camera sees to where the robot is.

### rtabmap
The main SLAM node. Takes RGB images, depth images and odometry as input and
builds a 3D map of the environment. Also localizes the robot within that map.
Publishes the map on `/map` which Nav2 uses for path planning.

### rtabmap_viz
Visualizes the RTAB-Map output in RViz2. Shows the current map, point cloud,
camera trajectory and loop closure detections.

### nav2 (multiple nodes)
A collection of nodes that together handle autonomous navigation:
- **controller_server** — sends velocity commands to `/cmd_vel`
- **planner_server** — calculates the optimal path from A to B
- **behavior_server** — handles recovery behaviors when the robot gets stuck
- **bt_navigator** — coordinates all Nav2 nodes using a behavior tree
- **costmap** — builds a grid showing where obstacles are
- **velocity_smoother** — smooths out jerky velocity commands

### mpu6050driver
Reads accelerometer and gyroscope data from the MPU6050 IMU connected
to I2C bus 7 on the Jetson GPIO header. Publishes to `/imu`.
Currently not fused with wheel odometry but can be added later using
the robot_localization package.

## Repository Structure
- `README.md` — this file
- `code/dynamixel_driver.py` — motor driver node
- `setup/startup.md` — full startup sequence
- `setup/troubleshooting.md` — common issues and fixes
- `simulation/README.md` — guide for recreating in simulation

## Quick Start
See [setup/startup.md](setup/startup.md)
