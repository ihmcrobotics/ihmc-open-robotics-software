package us.ihmc.sensorProcessing.simulatedSensors;

import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.AuxiliaryRobotData;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;

public interface SensorReader
{
   public abstract void read();

   public abstract SensorOutputMapReadOnly getSensorOutputMapReadOnly();

   public abstract SensorRawOutputMapReadOnly getSensorRawOutputMapReadOnly();

   AuxiliaryRobotData newAuxiliaryRobotDataInstance();
}