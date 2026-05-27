#!/usr/bin/env bash
# Installer ROS 2 Jazzy og MoonMapper systemavhengigheter (Ubuntu 24.04).
# Kjør: bash scripts/install_ros_deps.sh
set -euo pipefail

if [[ "$(id -u)" -ne 0 ]]; then
  echo "Kjør med sudo: sudo bash scripts/install_ros_deps.sh" >&2
  exit 1
fi

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y curl gnupg lsb-release software-properties-common

# ROS 2 Jazzy repository
if [[ ! -f /etc/apt/sources.list.d/ros2.list ]]; then
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "${UBUNTU_CODENAME:-noble}") main" \
    > /etc/apt/sources.list.d/ros2.list
fi

apt-get update
apt-get install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-nav2-bringup \
  ros-jazzy-robot-localization \
  ros-jazzy-rtabmap-ros \
  ros-jazzy-realsense2-camera \
  ros-jazzy-xacro \
  ros-jazzy-teleop-twist-keyboard \
  python3-rosdep \
  python3-colcon-common-extensions \
  libgz-sim8-dev \
  libgz-plugin2-dev

if ! rosdep init 2>/dev/null; then true; fi
rosdep update

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WS_ROOT}"

rosdep install --from-paths src --ignore-src -r -y || true

echo ""
echo "Ferdig. Bygg workspace:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  cd ${WS_ROOT} && colcon build --symlink-install"
echo "  source install/setup.bash"
