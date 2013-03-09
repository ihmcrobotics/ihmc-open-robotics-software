#!/bin/sh

modelPath=src/us/ihmc/darpaRoboticsChallenge/models/GFE/
cp -r /usr/share/drcsim-2.0/gazebo_models/ $modelPath

gzsdf print $modelPath/gazebo_models/atlas_description/atlas/atlas.urdf > $modelPath/atlas.sdf
gzsdf print $modelPath/gazebo_models/atlas_description/atlas_sandia_hands/atlas_sandia_hands.urdf > $modelPath/atlas_sandia_hands.sdf
gzsdf print $modelPath/gazebo_models/atlas_description/atlas_irobot_hands/atlas_irobot_hands.urdf > $modelPath/atlas_irobot_hands.sdf
