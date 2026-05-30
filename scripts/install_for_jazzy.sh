#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PACKAGE_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

sudo apt update -y
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-slam-toolbox \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-map-server \
  ros-jazzy-moveit \
  ros-jazzy-moveit-ros \
  ros-jazzy-rviz2 \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-pcl-ros \
  ros-jazzy-xacro \
  ros-dev-tools \
  python3-pip \
  python3-colcon-common-extensions \
  python3-argcomplete \
  pcl-tools

cat <<EOF

Jazzy dependencies installed.

Next steps:
  1. Build the workspace with colcon.
  2. Source install/setup.bash before launching.

The migrated launch files use ros_gz_sim package exports to populate
GZ_SIM_RESOURCE_PATH, so there is no need to copy models into ~/.gazebo.

Package root:
  ${PACKAGE_ROOT}
EOF
