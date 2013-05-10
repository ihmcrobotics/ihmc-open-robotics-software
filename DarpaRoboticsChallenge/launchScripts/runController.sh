#!/bin/bash

source getClasspath.sh
java -classpath "${CLASSPATH}" -Djava.library.path="${MYDIR}/lib" us.ihmc.darpaRoboticsChallenge.gazebo.AtlasROSControllerFactory --headless --host http://192.168.100.2:11311 $@
