# Simulation Guide

## Goal
The physical hardware in Steps 1-4 of the [startup guide](../setup/startup.md)
needs to be replaced with a simulated equivalent. Once that is done,
Steps 5-6 (RTAB-Map and Nav2) run identically without any changes.

## What needs to be simulated

### Motors (replaces Step 3)
The simulation needs a 6-wheel skid steer robot that:
- Subscribes to `/cmd_vel` and drives the simulated wheels accordingly
- Publishes wheel odometry to `/odom`
- Broadcasts the `odom -> base_link` TF transform

### Camera (replaces Steps 1-2)
The simulation needs a depth camera that publishes:
- `/camera/camera/color/image_raw` — RGB image
- `/camera/camera/depth/image_rect_raw` — depth image
- `/camera/camera/color/camera_info` — camera calibration info
- `/camera/camera/depth/color/points` — point cloud

### TF Frames (replaces Step 4)
The simulation must publish these exact TF frame names:
```
odom
└── base_link
    └── camera_link
        ├── camera_color_optical_frame
        └── camera_depth_optical_frame
```
## Robot Parameters
- Wheel radius: see [dynamixel_driver.py](../code/dynamixel_driver.py)
- Wheel separation: see [dynamixel_driver.py](../code/dynamixel_driver.py)
- Camera height above base: 0.2 meters (placeholder — measure the actual height on your robot)
- Drive type: skid steer (left side: IDs 1,3,5 — right side: IDs 2,4,6)

## Camera Parameters
- Resolution: 640x480
- Frame rate: 60 Hz
- Color and depth streams must be synchronized

## Once simulation is running
Run Steps 5-6 from the [startup guide](../setup/startup.md) exactly as written:
- Step 5 — RTAB-Map builds the map from the simulated camera
- Step 6 — Nav2 plans paths and sends velocity commands to the simulated motors
