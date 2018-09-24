#!/bin/bash

echo "sourcing /opt/ros/kinetic/setup.bash"
source /opt/ros/kinetic/setup.bash

echo "sourcing /root/ihmc_catkin_ws/install/setup.bash"
source /root/ihmc_catkin_ws/install/setup.bash

echo "sourcing /opt/ros/ardent/setup.bash"
source /opt/ros/ardent/setup.bash

#echo "sourcing /root/ros2_ws/install/local_setup.bash"
#source /root/ros2_ws/install/local_setup.bash

echo "sourcing /root/ihmc_ros1_bridge/install/local_setup.bash"
source /root/ihmc_ros1_bridge/install/local_setup.bash


