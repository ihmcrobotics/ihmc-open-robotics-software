#!/bin/bash

echo "Setting up Workspace..."

# Build DCV_v2 and installing CenterPose
cd $CenterPose_ROOT/src/lib/models/networks/DCNv2 || exit
python3 setup.py build develop

cp /home/CenterPose_ws/ZEDImage2CenterPose.py /home/CenterPose/src/
cd /home/CenterPose || exit
source /home/ros2ws/install/setup.bash
python3 src/ZEDImage2CenterPose.py
