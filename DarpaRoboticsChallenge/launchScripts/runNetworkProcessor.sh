#!/bin/bash

source /home/ubuntu/cloudsim/ros.bash

cd /home/ubuntu/vrc

source getClasspath.sh

java -classpath "${CLASSPATH}" -Djava.library.path="${MYDIR}/lib" us.ihmc.darpaRoboticsChallenge.processManagement.DRCNetworkProcessorEnterpriseCloudDispatcherBackend -h --ros-uri $1 --scs-ip $2
