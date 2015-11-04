package us.ihmc.sensorProcessing.simulatedSensors;

import us.ihmc.sensorProcessing.communication.packets.dataobjects.AuxiliaryRobotData;

public interface AuxiliaryRobotDataProvider
{
   AuxiliaryRobotData newAuxiliaryRobotDataInstance();
}
