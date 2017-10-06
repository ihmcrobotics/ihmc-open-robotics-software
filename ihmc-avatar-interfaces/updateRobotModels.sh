#!/bin/sh

gazeboPath=/usr/share/gazebo-3.0

gazeboStateCommunicatorDir="../GazeboStateCommunicator/launch/models"
drcsimdir="/opt/ros/groovy/share/atlas_description"
multisensedir="/opt/ros/groovy/share/multisense_sl_description/"
modelPath=src/us/ihmc/darpaRoboticsChallenge/models/GFE/


cp -r ~/.gazebo/models $modelPath
cp -r $drcsimdir $modelPath
cp -r $multisensedir $modelPath

cp -r $gazeboPath/media $modelPath/gazebo 
cp -r $drcsimPath/media $modelPath/drcsim

cp $modelPath/gazebo/media/materials/textures/road1.jpg ../SDFLoader/src/us/ihmc/SdfLoader/models


gzsdf print $gazeboStateCommunicatorDir/atlas_v3/model.sdf > $modelPath/atlas.sdf
gzsdf print $gazeboStateCommunicatorDir/atlas_v3_sandia_hands/model.sdf > $modelPath/atlas_sandia_hands.sdf
gzsdf print $gazeboStateCommunicatorDir/atlas_v3_irobot_hands/model.sdf > $modelPath/atlas_irobot_hands.sdf


mkdir -p $modelPath/worlds/

cp $gazeboStateCommunicatorDir/*.world $modelPath/worlds/ 
