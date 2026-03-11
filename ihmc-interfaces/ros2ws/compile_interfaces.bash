#!/bin/bash

# We assume the ROS2 installation has been sourced. Example: source /opt/ros/iron/setup.sh

rm -rf src
mkdir src
cp -r ../src/main/messages/ihmc_interfaces src/
colcon build --packages-skip vllm_extensions
