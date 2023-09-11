#!/bin/bash

echo "Setting up workspace..."

# Change this for your specific setup
export ROS_DOMAIN_ID=80
echo "Setting ROS_DOMAIN_ID to $ROS_DOMAIN_ID"

cp /home/CenterPose_ws/zed2/ZEDImage2CenterPose.py /home/CenterPose/src/
cd /home/CenterPose

source /home/ros2ws/install/setup.bash

python3 src/ZEDImage2CenterPose.py
