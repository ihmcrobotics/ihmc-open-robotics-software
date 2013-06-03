#!/bin/sh

drcsimPath=/usr/share/drcsim-2.6
gazeboPath=/usr/share/gazebo-1.8

modelPath=src/us/ihmc/darpaRoboticsChallenge/models/GFE/
cp -r ~/.gazebo/models/polaris_ranger_ev $modelPath/models/
gzsdf print ../GazeboStateCommunicator/launch/models/atlasVehiclePurgatory.world > $modelPath/atlasVehicleWorld.sdf
gzsdf print $modelPath/drc_vehicle.world > $modelPath/drc_vehicle.sdf

