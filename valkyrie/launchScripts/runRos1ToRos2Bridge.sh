#!/bin/bash

source $HOME/.val_config
source $HOME/.ihmc_bridge_config
scriptname=$(basename $0)
BRIDGE_NAME=valkyrie_static_bridge_yaml

# No bridge options for static bridge
BRIDGE_OPTS=$HOME/ihmc_bridge_ws/install/yaml_configs/valkyrie_bridged_topics.yaml

# Kill off old processes

# If the ROS bridge script is running, this must be killed first because it will attempt to
# restart the bridge.
bridge_script_processes=$(pgrep -f $scriptname)
for p in $bridge_script_processes; do
    # Kill others while preserving ourselves
    if [[ "$p" != "$$" ]]; then
        echo Cleaning up old bridge script with pid $p
        kill $p
        sleep 2
        kill -0 $p 2>/dev/null 1>&2 && echo "Unable to clean up old bridge script with pid $p" && exit 1
    fi
done

# Nuke any existing bridges, giving processes time to die
killall -q $BRIDGE_NAME && echo Waiting for old bridge processes to die && sleep 3
killall -q -9 $BRIDGE_NAME && echo Waiting for old bridge processes to die harder && sleep 3

# Check whether there's something still not dead, in which case give up
killall -q -0 $BRIDGE_NAME && echo Unable to kill old bridge processes && exit 1

echo "Starting bridge"

while [[ 1 ]]; do 
    /usr/bin/python3 /opt/ros/ardent/bin/ros2 run ros1_bridge $BRIDGE_NAME $BRIDGE_OPTS &
    BRIDGE_PID=$!
    trap "kill $BRIDGE_PID && source $HOME/.val_config && rosnode kill ros_bridge; exit 0" INT TERM
    wait
done
