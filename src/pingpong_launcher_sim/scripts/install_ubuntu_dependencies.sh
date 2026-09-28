#!/usr/bin/env bash
set -euo pipefail
source /etc/os-release
if [[ "${ID}" != "ubuntu" || "${VERSION_ID}" != "22.04" ]]; then
  echo 'This package targets Ubuntu 22.04. Install ROS 2 Humble on that system first.' >&2
  exit 1
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'Install ROS 2 Humble first: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html' >&2
  exit 1
fi
sudo apt-get update
sudo apt-get install -y build-essential cmake python3-colcon-common-extensions python3-pytest \
  ros-humble-ros-gz ros-humble-ament-cmake-python ros-humble-ament-cmake-pytest \
  libignition-gazebo6-dev
