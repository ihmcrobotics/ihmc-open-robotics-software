#!/bin/bash

source getClasspath.sh
java -classpath "${CLASSPATH}" -Djava.library.path="${MYDIR}/lib" us.ihmc.darpaRoboticsChallenge.gazebo.AtlasROSControllerFactory --headless --host http://localhost:11311 $@
