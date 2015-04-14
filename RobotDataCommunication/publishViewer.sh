#!/bin/sh

gradle -PmainClass=us.ihmc.robotDataCommunication.gui.GUICaptureViewer distTar

scp build/distributions/RobotDataCommunication-1.0.tar unknownid@10.7.4.47:
ssh unknownid@10.7.4.47 "tar xvf RobotDataCommunication-1.0.tar"
