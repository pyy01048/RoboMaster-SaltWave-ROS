#!/usr/bin/env bash
set -euo pipefail

sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-ament-cmake \
  ros-humble-joint-state-publisher \
  ros-humble-nav2-bringup \
  ros-humble-navigation2 \
  ros-humble-robot-localization \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2 \
  ros-humble-slam-toolbox \
  ros-humble-tf-transformations \
  ros-humble-xacro

if ! rosdep db 2>/dev/null | grep -q "humble"; then
  sudo rosdep init || true
  rosdep update
fi

rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
