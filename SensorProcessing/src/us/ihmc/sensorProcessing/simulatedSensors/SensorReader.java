package us.ihmc.sensorProcessing.simulatedSensors;

import us.ihmc.sensorProcessing.sensors.ForceSensorDataHolder;

public interface SensorReader
{
   public abstract JointAndIMUSensorMap getJointAndIMUSensorMap();

   public abstract void setControllerDispatcher(ControllerDispatcher controllerDispatcher);

   public abstract void setForceSensorDataHolder(ForceSensorDataHolder forceSensorDataHolder);
}