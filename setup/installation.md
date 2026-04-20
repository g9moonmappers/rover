# Installation Guide

## 1. JetPack
“Follow this guide (https://www.youtube.com/watch?v=BaRdpSXU6EM
) and install JetPack 6.2.1 (rev. 1) on the Jetson via either an NVMe SSD (size 2280 or 2230) or an SD card. An SSD is strongly recommended. JetPack should come pre-installed with CUDA, cuDNN, TensorRT, and Ubuntu 22.04. Verify that everything is installed before continuing.

## 2. ROS2 Humble

```bash
# Set up locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS2 repo
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions

# Add to bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 3. Intel RealSense D435 Driver

```bash
# Register Intel key
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

# Fix key if needed (key changed to FB0B24895113F120)
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key FB0B24895113F120

# Add repo
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

# Install
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev

# Install kernel modules (pre-built for JetPack 6)
# Clone jetsonhacks repo
git clone https://github.com/jetsonhacks/jetson-orin-librealsense
cd jetson-orin-librealsense

# Verify checksum
sha256sum -c install-modules.tar.gz.sha256

# Extract and install
tar -xzf install-modules.tar.gz
cd install-modules
sudo ./install-realsense-modules.sh

# Install ROS2 wrapper
sudo apt install ros-humble-realsense2-camera
```

## 4. RTAB-Map

```bash
sudo apt install ros-humble-rtabmap-ros
```

## 5. Nav2

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

## 6. Dynamixel SDK

```bash
sudo apt install ros-humble-dynamixel-sdk
pip3 install pyserial --break-system-packages
```

## 7. MPU6050 Driver
The standard driver does not support selecting the I2C bus so it
was built from source and modified to use bus 7.

```bash
# Install dependencies
sudo apt install libi2c-dev i2c-tools

# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone and modify
git clone https://github.com/hiwad-aziz/ros2_mpu6050_driver.git
```

Edit line 9 in `src/mpu6050driver.cpp`:
```cpp
// Change from:
: Node("mpu6050publisher"), mpu6050_{std::make_unique<MPU6050Sensor>()}
// To:
: Node("mpu6050publisher"), mpu6050_{std::make_unique<MPU6050Sensor>(7)}
```

```bash
# Build
cd ~/ros2_ws
colcon build
source ~/ros2_ws/install/setup.bash
```
