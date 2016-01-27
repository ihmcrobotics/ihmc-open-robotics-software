#!/bin/sh

gradle -PmainClass=us.ihmc.robotDataCommunication.gui.GUICaptureViewer  distTar

scp build/distributions/RobotDataCommunication.tar unknownid@10.7.4.47:
ssh unknownid@10.7.4.47 "tar xvf RobotDataCommunication.tar"
