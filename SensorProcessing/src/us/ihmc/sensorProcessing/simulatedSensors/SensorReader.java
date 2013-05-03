package us.ihmc.sensorProcessing.simulatedSensors;

import us.ihmc.sensorProcessing.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.stateEstimation.JointAndIMUSensorDataSource;

public interface SensorReader
{

   public abstract void setJointAndIMUSensorDataSource(JointAndIMUSensorDataSource jointAndIMUSensorDataSource);

   public abstract void setControllerDispatcher(ControllerDispatcher controllerDispatcher);

   public abstract void setForceSensorDataHolder(ForceSensorDataHolder forceSensorDataHolder);

}