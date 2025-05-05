#!/bin/bash
set -e
export PYTHONUNBUFFERED=1

echo "Building the workspace..."

if [ ! -d install ]; then
  echo "Building the workspace..."
  colcon build --symlink-install
fi

echo "Sourcing ROS setup files..."
source /opt/ros/humble/setup.bash
source /home/devuser/workspace/install/setup.bash

echo "Starting ROS 2 node: rmv_chore"

exec python3 -m rmv_chore
