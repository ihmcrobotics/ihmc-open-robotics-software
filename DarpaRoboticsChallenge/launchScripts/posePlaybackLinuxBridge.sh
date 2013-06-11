#!/bin/bash

source getControllerClasspath.sh
java -classpath "${CLASSPATH}" -Djava.library.path="${MYDIR}/lib"  us.ihmc.darpaRoboticsChallenge.posePlayback.PosePlaybackLinuxBridge --headless --host http://localhost:11311 $@
