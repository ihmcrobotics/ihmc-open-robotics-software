#!/bin/bash

source getNetworkProcessorClasspath.sh
java -Xms10240m -Xmx10240m -classpath "${CLASSPATH}" -Djava.library.path="${MYDIR}/lib" us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkProcessor --ros-uri $1 --scs-ip $2
