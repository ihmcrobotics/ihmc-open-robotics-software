#!/bin/bash

echo "Setting up workspace..."

if [ -z "${ROS_DOMAIN_ID}" ]; then
  echo "ROS_DOMAIN_ID is not defined"
  exit 1
fi

echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"

cp /root/centerpose-ros2/zed2/*.py /root/centerpose-install/src/
cd /root/centerpose-install

# Source the IHMC ROS2 workspace to bring in all the custom IHMC interfaces
source /root/ihmc_ros2_ws/install/setup.bash

python3 src/ZED2CenterposeNode.py
