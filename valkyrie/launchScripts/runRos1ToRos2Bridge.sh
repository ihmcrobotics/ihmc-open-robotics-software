#!/bin/bash

source $HOME/.val_config
source $HOME/.ihmc_bridge_config
while [[ 1 ]]; do 
    /usr/bin/python3 /opt/ros/ardent/bin/ros2 run ros1_bridge dynamic_bridge --bridge-all-topics &
    BRIDGE_PID=$!
    trap "kill $BRIDGE_PID && source $HOME/.val_config && rosnode kill ros_bridge; exit 0" INT TERM
    wait
done
