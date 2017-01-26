#!/bin/bash

source /opt/ros/indigo//setup.bash
export ROS_PACKAGE_PATH=$PWD/../:$ROS_PACKAGE_PATH

#rosrun xacro xacro.py ../atlas_description/robots/atlas_v3.urdf.xacro > atlas_v3.urdf
#gzsdf print atlas_v3.urdf > atlas_v3.sdf
#rosparam set /robot_description -t atlas_v3.urdf
#DO NOT use, as robot state publisher doesn't know it's updated

#rosrun xacro xacro.py ../atlas_description/robots/atlas_v3_irobot_hands.urdf.xacro > atlas_v3_irobot_hands.urdf
#gzsdf print atlas_v3_irobot_hands.urdf|sed 's/\[\([[:digit:]]\)\]/_\1/g' > atlas_v3_irobot_hands.sdf
#rosparam set /robot_description_irobot_hands -t atlas_v3_irobot_hands.urdf

#rosrun xacro xacro.py atlas_v3_hook.urdf.xacro > sdf/atlas_v3_hook.urdf
#gzsdf print sdf/atlas_v3_hook.urdf > sdf/atlas_v3_hook.sdf

for INPUT_XACRO in robots/*urdf.xacro
do
	INPUT=${INPUT_XACRO##robots/}
	INPUT_URDF=urdf/${INPUT%.xacro}
	INPUT_SDF=sdf/${INPUT%.urdf.xacro}.sdf
	echo $INPUT_XACRO to $INPUT_URDF to $INPUT_SDF

	rosrun xacro xacro.py $INPUT_XACRO |sed 's/\[\([[:digit:]]\)\]\//_\1_/g' > $INPUT_URDF
	gzsdf print $INPUT_URDF  > $INPUT_SDF

	ROBOT=${INPUT%.urdf.xacro}
	MODELDIR=$HOME/.gazebo/models/$ROBOT
	mkdir -p  $MODELDIR
	echo '<?xml version="1.0"?>' > $MODELDIR/model.config
	echo '<model>' >> $MODELDIR/model.config
	echo "<name>${ROBOT}</name>" >> $MODELDIR/model.config
	echo "<sdf version='1.4'>model.sdf</sdf>"  >> $MODELDIR/model.config
	echo "</model>" >> $MODELDIR/model.config

	cp $INPUT_SDF $MODELDIR/model.sdf
done


#echo cp sdf/*sdf ../../src/us/ihmc/darpaRoboticsChallenge/models/GFE/
echo cp sdf/*sdf ../../../Atlas/resources/models/GFE/

Atlas/resources/models/GFE/drc_no_hands.sdf

#roslauch ./show_in_rviz.launch &
#rosparam set /robot_description -t sdf/atlas_v3_hook.urdf

#       <visual name='l_box'>
#        <pose>0.15 +0.25 0.05  -1.57 0 -2.36 </pose>
#        <geometry>
#           <mesh>
#            <scale>1 1 1</scale>
#            <uri>file://ihmc/calibration_cube.dae</uri>
#           </mesh>
#        </geometry>
#      </visual>   
#
