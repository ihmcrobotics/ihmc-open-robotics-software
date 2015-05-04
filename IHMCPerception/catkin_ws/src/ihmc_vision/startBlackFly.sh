#!/bin/bash

source /opt/ros/indigo/setup.bash
source $HOME/IHMCPerception/catkin_ws/devel/setup.bash
source $HOME/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.7.4.100:11311/
export ROS_IP=10.7.4.102

roslaunch ihmc_vision blackFly.launch
