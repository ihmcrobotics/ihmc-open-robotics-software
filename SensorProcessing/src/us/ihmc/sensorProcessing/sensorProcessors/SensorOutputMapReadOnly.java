package us.ihmc.sensorProcessing.sensorProcessors;

import java.util.List;

import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.robotics.humanoidRobot.model.ForceSensorDataHolderReadOnly;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

public interface SensorOutputMapReadOnly
{
   public long getTimestamp();

   public long getVisionSensorTimestamp();

   public long getSensorHeadPPSTimestamp();
   
   public double getJointPositionProcessedOutput(OneDoFJoint oneDoFJoint);

   public double getJointVelocityProcessedOutput(OneDoFJoint oneDoFJoint);
   
   public double getJointAccelerationProcessedOutput(OneDoFJoint oneDoFJoint);

   public double getJointTauProcessedOutput(OneDoFJoint oneDoFJoint);

   public boolean isJointEnabled(OneDoFJoint oneDoFJoint);
   
   public List<? extends IMUSensorReadOnly> getIMUProcessedOutputs();

   public ForceSensorDataHolderReadOnly getForceSensorProcessedOutputs();
}
