package us.ihmc.sensorProcessing.simulatedSensors;

import us.ihmc.sensorProcessing.communication.packets.dataobjects.AtlasAuxiliaryRobotData;

public interface AuxiliaryRobotDataProvider
{
   AtlasAuxiliaryRobotData newAuxiliaryRobotDataInstance();
}
