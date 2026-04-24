# Camera Odometry Setup

Provides position data from the D435 camera using visual odometry.
Used as input for a Kalman filter or other position estimation systems.

## Prerequisites
- D435 plugged in via USB-A on the Jetson
- ROS2 Humble installed
- realsense2_camera installed
- rtabmap_ros installed

## Step 1 - Testing camera connection
Open a new terminal and run:
```bash
realsense-viewer
```
In the upper left corner it should say: `Intel RealSense D435 3.2` and `If it says 2.x, reconnect the camera until it says 3.2`

When you see `Intel RealSense D435 3.2` in the upper left corner, you can close realsense-viewer 

## Step 2 - Camera
Open a new terminal and run:
```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_sync:=true \
  depth_module.depth_profile:=640x480x60 \
  rgb_camera.color_profile:=640x480x60
```
Wait for: `RealSense Node Is Up!` and `Device USB type: 3.2`

## Step 3 - Enable Point Cloud
Open a new terminal and run:
```bash
ros2 param set /camera/camera pointcloud__neon_.enable true
```
Wait for: `Set parameter successful`

## Step 4 - RTAB-Map
Open a new terminal and run:
```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/depth/image_rect_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  frame_id:=camera_link \
  approx_sync:=false \
  visual_odometry:=true \
  icp_odometry:=false \
  Odom/MinInliers:=3 \
  Vis/MinInliers:=3 \
  Odom/ResetCountdown:=1
```

## Step 5 - Odometry Reader
Open a new terminal and run:
```bash
python3 ~/camera_odom.py
```
To edit the code, type: `sudo nano camera_odom.py and save by doing ctrl+o --> enter --> ctrl+x`

Move the camera slowly and position data will print continuously.
When tracking is lost it prints a warning and skips that frame.

## Output
The data is published on `/rtabmap/odom` as `nav_msgs/msg/Odometry`.
Position is in meters relative to where the camera started:
- x — forward/backward
- y — left/right
- z — up/down

## Code
```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        self.create_subscription(
            Odometry,
            '/rtabmap/odom',
            self.odom_callback,
            10)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # Check covariance - high value means tracking lost
        covariance = msg.pose.covariance[0]
        if covariance > 9000:
            print("WARNING: Tracking lost - skipping this measurement")
            return

        print(f"Position -> x: {x:.3f}  y: {y:.3f}  z: {z:.3f}")
        print(f"Rotation -> qx: {qx:.3f}  qy: {qy:.3f}  qz: {qz:.3f}  qw: {qw:.3f}")
        print("---")

def main():
    rclpy.init()
    node = OdomSubscriber()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```
