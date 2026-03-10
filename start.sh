#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "错误：请用 source 方式加载环境，否则不会影响当前终端。"
  echo "用法：source \"${SCRIPT_DIR}/start.sh\""
  exit 1
fi

source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/install/local_setup.bash"
echo "✅ ROS2 Humble 环境已激活"
echo "✅ maxwell_ros2 工作空间已加载"

