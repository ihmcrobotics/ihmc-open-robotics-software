#!/bin/bash
# Immediately exit on any errors.
set -e

# This script is used to build the messages while running docker build

source "/opt/ros/$ROS_DISTRO/setup.bash"

# Print commands as they are run.
set -o xtrace

cd colcon_ws
colcon build
