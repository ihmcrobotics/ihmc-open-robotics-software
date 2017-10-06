package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;

public interface ForceSensorCalibrationModule
{
   public ForceSensorDataHolderReadOnly getForceSensorOutput();

   public void requestFootForceSensorCalibrationAtomic();

}