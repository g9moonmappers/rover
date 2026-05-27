#!/usr/bin/env bash
# Source ROS 2 Jazzy + dette workspace. Kjør: source scripts/source_workspace.sh
_WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy ikke funnet. Installer med: sudo bash scripts/install_ros_deps.sh" >&2
  return 1 2>/dev/null || exit 1
fi

# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash

if [[ ! -f "${_WS_ROOT}/install/setup.bash" ]]; then
  echo "Workspace ikke bygget. Kjør fra ${_WS_ROOT}:" >&2
  echo "  colcon build --symlink-install" >&2
  return 1 2>/dev/null || exit 1
fi

# shellcheck source=/dev/null
source "${_WS_ROOT}/install/setup.bash"
export MOONMAPPER_WS="${_WS_ROOT}"
