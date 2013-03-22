#!/bin/sh

modelPath=src/us/ihmc/darpaRoboticsChallenge/models/GFE/

rosrun xacro xacro.py $modelPath/sandia_hand_left.urdf.xacro > $modelPath/sandia_hand_left.urdf
rosrun xacro xacro.py $modelPath/sandia_hand_right.urdf.xacro > $modelPath/sandia_hand_right.urdf

gzsdf print $modelPath/sandia_hand_left.urdf > $modelPath/sandia_hand_left.sdf
gzsdf print $modelPath/sandia_hand_right.urdf > $modelPath/sandia_hand_right.sdf
