#!/bin/sh

modelPath=src/us/ihmc/darpaRoboticsChallenge/models/GFE/

rosrun xacro xacro.py $modelPath/sandia_hand_left.urdf.xacro > $modelPath/sandia_hand_left.urdf
rosrun xacro xacro.py $modelPath/sandia_hand_right.urdf.xacro > $modelPath/sandia_hand_right.urdf

gzsdf print $modelPath/sandia_hand_left.urdf > $modelPath/sandia_hand_left.sdf
gzsdf print $modelPath/sandia_hand_right.urdf > $modelPath/sandia_hand_right.sdf

#gzsdf print /usr/share/sandia-hand-5.1/ros/sandia_hand_description/gazebo/sandia_hand_left_on_box/sandia_hand_left_on_box.urdf > $modelPath/sandia_left_hand_on_box.sdf
gzsdf print /opt/ros/groovy/share/sandia-hand-5.1/ros/sandia_hand_description/gazebo/sandia_hand_left_on_box/sandia_hand_left_on_box.urdf > $modelPath/sandia_left_hand_on_box.sdf