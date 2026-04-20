# Simulation Guide

##
Recreate the rover in simulation so the full ROS2 navigation stack
can be tested without physical hardware. Steps 5-7 in the
[startup guide](../setup/startup.md) are identical in simulation —
only Steps 1-4 need to be replaced with a simulated equivalent.

## Required ROS2 Topics
| Topic | Type | Direction |
|-------|------|-----------|
| `/cmd_vel` | geometry_msgs/Twist | Subscribe |
| `/odom` | nav_msgs/Odometry | Publish |
| `/camera/camera/color/image_raw` | sensor_msgs/Image | Publish |
| `/camera/camera/depth/image_rect_raw` | sensor_msgs/Image | Publish |
| `/camera/camera/color/camera_info` | sensor_msgs/CameraInfo | Publish |
| `/camera/camera/depth/color/points` | sensor_msgs/PointCloud2 | Publish |
| `/imu` | sensor_msgs/Imu | Publish |

## Required TF Frames
```
map
└── odom
    └── base_link
        └── camera_link
            ├── camera_color_optical_frame
            └── camera_depth_optical_frame
```
