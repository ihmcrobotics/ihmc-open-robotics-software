#!/bin/bash

source /home/ubuntu/cloudsim/ros.bash

cd /home/ubuntu/vrc

source getClasspath.sh

java -classpath "${CLASSPATH}" -Djava.library.path="${MYDIR}/lib" us.ihmc.darpaRoboticsChallenge.processManagement.DRCControllerEnterpriseCloudDispatcherBackend -h -g -r "$1"
