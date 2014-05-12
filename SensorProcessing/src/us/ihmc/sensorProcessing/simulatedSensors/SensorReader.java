package us.ihmc.sensorProcessing.simulatedSensors;

import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensors.ForceSensorDataHolder;

public interface SensorReader
{
   public abstract void read();
   
   public abstract SensorOutputMapReadOnly getSensorOutputMapReadOnly();

   public abstract void setControllerDispatcher(ControllerDispatcher controllerDispatcher);

   public abstract void setForceSensorDataHolder(ForceSensorDataHolder forceSensorDataHolder);
}