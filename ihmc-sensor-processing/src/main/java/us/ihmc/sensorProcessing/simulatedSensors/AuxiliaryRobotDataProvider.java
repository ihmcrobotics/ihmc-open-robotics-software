package us.ihmc.sensorProcessing.simulatedSensors;

import controller_msgs.msg.dds.AtlasAuxiliaryRobotData;

public interface AuxiliaryRobotDataProvider
{
   AtlasAuxiliaryRobotData newAuxiliaryRobotDataInstance();
}
