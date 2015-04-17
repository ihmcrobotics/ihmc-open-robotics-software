#!/bin/sh

gradle distTar -DmainClass=us.ihmc.robotDataCommunication.logger.YoVariableLoggerDispatcher

scp build/distributions/RobotDataCommunication-1.0.tar unknownid@10.7.4.46:
ssh unknownid@10.7.4.46 "tar xvf RobotDataCommunication-1.0.tar"
