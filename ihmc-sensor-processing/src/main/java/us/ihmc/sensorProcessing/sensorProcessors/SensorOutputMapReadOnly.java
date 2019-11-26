package us.ihmc.sensorProcessing.sensorProcessors;

import java.util.List;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;

public interface SensorOutputMapReadOnly extends SensorTimestampHolder
{
   OneDoFJointSensorOutputReadOnly getJointOutput(OneDoFJointBasics oneDoFJoint);

   default double getJointPositionOutput(OneDoFJointBasics oneDoFJoint)
   {
      return getJointOutput(oneDoFJoint).getPosition();
   }

   default double getJointVelocityOutput(OneDoFJointBasics oneDoFJoint)
   {
      return getJointOutput(oneDoFJoint).getVelocity();
   }

   default double getJointAccelerationOutput(OneDoFJointBasics oneDoFJoint)
   {
      return getJointOutput(oneDoFJoint).getAcceleration();
   }

   default double getJointTauOutput(OneDoFJointBasics oneDoFJoint)
   {
      return getJointOutput(oneDoFJoint).getEffort();
   }

   default boolean isJointEnabled(OneDoFJointBasics oneDoFJoint)
   {
      return getJointOutput(oneDoFJoint).isJointEnabled();
   }

   public List<? extends IMUSensorReadOnly> getIMUOutputs();

   public ForceSensorDataHolderReadOnly getForceSensorOutputs();
}
