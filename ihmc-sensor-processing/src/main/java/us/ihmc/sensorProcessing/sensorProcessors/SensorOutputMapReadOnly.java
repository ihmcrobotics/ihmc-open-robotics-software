package us.ihmc.sensorProcessing.sensorProcessors;

import java.util.List;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;

public interface SensorOutputMapReadOnly extends SensorTimestampHolder
{  
   public double getJointPositionProcessedOutput(OneDoFJointBasics oneDoFJoint);

   public double getJointVelocityProcessedOutput(OneDoFJointBasics oneDoFJoint);
   
   public double getJointAccelerationProcessedOutput(OneDoFJointBasics oneDoFJoint);

   public double getJointTauProcessedOutput(OneDoFJointBasics oneDoFJoint);

   public boolean isJointEnabled(OneDoFJointBasics oneDoFJoint);
   
   public List<? extends IMUSensorReadOnly> getIMUProcessedOutputs();

   public ForceSensorDataHolderReadOnly getForceSensorProcessedOutputs();

   public default void reset()
   {
   }
}
