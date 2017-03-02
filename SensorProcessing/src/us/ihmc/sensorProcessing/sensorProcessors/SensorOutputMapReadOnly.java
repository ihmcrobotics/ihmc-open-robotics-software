package us.ihmc.sensorProcessing.sensorProcessors;

import java.util.List;

import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;

public interface SensorOutputMapReadOnly extends SensorTimestampHolder
{  
   public double getJointPositionProcessedOutput(OneDoFJoint oneDoFJoint);

   public double getJointVelocityProcessedOutput(OneDoFJoint oneDoFJoint);
   
   public double getJointAccelerationProcessedOutput(OneDoFJoint oneDoFJoint);

   public double getJointTauProcessedOutput(OneDoFJoint oneDoFJoint);

   public boolean isJointEnabled(OneDoFJoint oneDoFJoint);
   
   public List<? extends IMUSensorReadOnly> getIMUProcessedOutputs();

   public ForceSensorDataHolderReadOnly getForceSensorProcessedOutputs();
}
