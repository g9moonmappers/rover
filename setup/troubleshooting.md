# Troubleshooting

## Camera Issues

### No RealSense devices found
- Check USB connection, must be a USB 3.x cable connected to one of the Jetson's USB-A ports 
- Try unplugging and replugging the camera
- Check: `ls /dev/video*`

### Point cloud not showing up
The realsense2_camera v4.57.7 has a bug where pointcloud.enable
must be set at runtime as pointcloud__neon_.enable:
```bash
ros2 param set /camera/camera pointcloud__neon_.enable true
```

### RGB and depth resolution mismatch
RTAB-Map requires matching aspect ratios between color and depth.
Always launch with matching profiles, eg:
- depth_module.depth_profile:=640x480x60
- rgb_camera.color_profile:=640x480x60

## Motor Issues

### Dynamixel motors not found
```bash
sudo chmod 777 /dev/ttyUSB0
```

### Wrong port
Check which port the U2D2 is on:
```bash
ls /dev/ttyUSB*
```
Update PORT in dynamixel_driver.py if different from /dev/ttyUSB0

### Motors spinning wrong direction
Edit dynamixel_driver.py and flip the negation in cmd_vel:
```python
self.send_velocity(-left_vel, -right_vel)
```

## IMU Issues

### MPU6050 not detected
The MPU6050 is on I2C bus 7 on the Jetson Orin Nano Super.
Scan all buses to confirm:
```bash
for i in 0 1 2 4 5 7 9; do echo "=== Bus $i ==="; sudo i2cdetect -y -r $i 2>/dev/null; done
```
Should show 68 on bus 7.

### IMU driver using wrong bus
The ros2_mpu6050_driver was modified to hardcode bus 7.
If rebuilding from source edit line 9 in src/mpu6050driver.cpp:
```cpp
: Node("mpu6050publisher"), mpu6050_{std::make_unique<MPU6050Sensor>(7)}
```

## RTAB-Map Issues

### TF of received image is not set
Add static transform between base_link and camera_link:
```bash
ros2 run tf2_ros static_transform_publisher 0.0 0.0 0.2 0.0 0.0 0.0 base_link camera_link
```

### Map turns red / tracking lost
- Move the camera/robot more slowly
- Make sure scene has visual features (not plain white walls)
- Lower the minimum inliers: Odom/MinInliers:=6

## Nav2 Issues

### base_link to odom transform not found
Make sure motor driver is running before launching Nav2.
The motor driver publishes the odom→base_link transform.

### Robot out of bounds of costmap
Normal warning during initial mapping — ignore until map is built.

### No map received
RTAB-Map must be running and have built some map before Nav2
can use it. Drive the robot around first to build the map.
