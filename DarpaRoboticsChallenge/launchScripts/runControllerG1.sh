#!/bin/bash

source getControllerClasspath.sh
java -Xms10240m -Xmx10240m -XX:+UseG1GC -XX:MaxGCPauseMillis=3 -XX:InitiatingHeapOccupancyPercent=65 -XX:ParallelGCThreads=8 -XX:ConcGCThreads=8 -XX:+AggressiveOpts -classpath "${CLASSPATH}" -Djava.library.path="${MYDIR}/lib" us.ihmc.darpaRoboticsChallenge.gazebo.AtlasROSControllerFactory --headless --host "$1" "${@:2}"
