package us.ihmc.sensorProcessing.simulatedSensors;

import us.ihmc.controlFlow.ControlFlowElement;
import us.ihmc.sensorProcessing.sensorData.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.stateEstimation.JointAndIMUSensorDataSource;

public interface SensorReader
{

   public abstract void setJointAndIMUSensorDataSource(JointAndIMUSensorDataSource jointAndIMUSensorDataSource);

   public abstract void setControllerDispatcher(ControlFlowElement controllerDispatcher);

   public abstract void setForceSensorDataHolder(ForceSensorDataHolder forceSensorDataHolder);

}