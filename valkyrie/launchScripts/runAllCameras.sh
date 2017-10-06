#!/bin/bash

. /opt/ros/groovy/setup.bash

export ROS_IP=139.169.44.27
export ROS_HOSTNAME=$ROS_IP
export ROS_MASTER_URI=http://139.169.44.21:11311

roslaunch launchPerception.launch
