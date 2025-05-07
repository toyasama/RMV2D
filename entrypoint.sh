#!/bin/bash
set -e
export PYTHONUNBUFFERED=1

source /opt/ros/humble/setup.bash
source /home/devuser/workspace/install/setup.bash
exec "$@"