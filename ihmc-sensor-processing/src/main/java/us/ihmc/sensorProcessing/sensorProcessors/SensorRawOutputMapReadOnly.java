package us.ihmc.sensorProcessing.sensorProcessors;

import java.util.List;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;

public interface SensorRawOutputMapReadOnly extends SensorTimestampHolder
{
   @Override
   public long getTimestamp();

   @Override
   public long getVisionSensorTimestamp();

   @Override
   public long getSensorHeadPPSTimestamp();
   
   public double getJointPositionRawOutput(OneDoFJointBasics oneDoFJoint);

   public double getJointVelocityRawOutput(OneDoFJointBasics oneDoFJoint);
   
   public double getJointAccelerationRawOutput(OneDoFJointBasics oneDoFJoint);

   public double getJointTauRawOutput(OneDoFJointBasics oneDoFJoint);
   
   public List<? extends IMUSensorReadOnly> getIMURawOutputs();

   public ForceSensorDataHolderReadOnly getForceSensorRawOutputs();
}
