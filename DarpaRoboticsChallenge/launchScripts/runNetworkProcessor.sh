#!/bin/bash

source getClasspath.sh
java -classpath "${CLASSPATH}" -Djava.library.path="${MYDIR}/lib" us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkProcessor $1
