#!/bin/sh

if [ $# -ne 1 ]
then echo "Usage: $0 [user]@[host]"
exit 1;
fi

gradle distTar -DmainClass=us.ihmc.robotDataCommunication.logger.YoVariableLoggerDispatcher

scp build/distributions/RobotDataCommunication.tar "$1":
ssh "$1" "tar xvf RobotDataCommunication.tar"
