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

## Software Stack
- **OS:** Jetpack 6.2.1 (rev.1) / Ubuntu 22.04
- **ROS2:** Humble
- **SLAM:** RTAB-Map + Rviz
- **Navigation:** Nav2
- **Camera driver:** realsense2_camera v4.57.7
- **Motor driver:** dynamixel_sdk
- **IMU driver:** ros2_mpu6050_driver

## Repository Structure
- `README.md` — this file
- `code/dynamixel_driver.py` — motor driver node
- `setup/startup.md` — full startup sequence
- `setup/troubleshooting.md` — common issues and fixes
- `simulation/README.md` — guide for recreating in simulation

## Quick Start
See [setup/startup.md](setup/startup.md)
