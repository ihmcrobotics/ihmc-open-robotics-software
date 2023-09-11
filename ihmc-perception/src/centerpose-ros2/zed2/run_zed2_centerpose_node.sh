#!/bin/bash

echo "Setting up workspace..."

# Change this for your specific setup
export ROS_DOMAIN_ID=80
echo "Setting ROS_DOMAIN_ID to $ROS_DOMAIN_ID"

cp /root/centerpose-ros2/zed2/zed2_centerpose_node.py /root/centerpose-install/src/
cd /root/centerpose-install

# Source the IHMC ROS2 workspace to bring in all the custom IHMC interfaces
source /root/ihmc_ros2_ws/install/setup.bash

python3 src/zed2_centerpose_node.py
