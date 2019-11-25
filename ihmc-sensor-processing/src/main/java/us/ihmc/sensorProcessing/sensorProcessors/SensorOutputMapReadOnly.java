package us.ihmc.sensorProcessing.sensorProcessors;

import java.util.List;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;

public interface SensorOutputMapReadOnly extends SensorTimestampHolder
{
   public double getJointPositionOutput(OneDoFJointBasics oneDoFJoint);

   public double getJointVelocityOutput(OneDoFJointBasics oneDoFJoint);

   public double getJointAccelerationOutput(OneDoFJointBasics oneDoFJoint);

   public double getJointTauOutput(OneDoFJointBasics oneDoFJoint);

   public boolean isJointEnabled(OneDoFJointBasics oneDoFJoint);

   public List<? extends IMUSensorReadOnly> getIMUOutputs();

   public ForceSensorDataHolderReadOnly getForceSensorOutputs();
}
