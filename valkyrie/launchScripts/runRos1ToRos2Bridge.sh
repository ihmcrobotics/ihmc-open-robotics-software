#!/bin/bash

source /home/val/.val_config
source /home/val/.ihmc_bridge_config
/usr/bin/python3 /opt/ros/ardent/bin/ros2 run ros1_bridge dynamic_bridge --bridge-all-topics &
BRIDGE_PID=$!
trap "kill $BRIDGE_PID && source /home/val/.val_config && rosnode kill ros_bridge" INT TERM
wait

