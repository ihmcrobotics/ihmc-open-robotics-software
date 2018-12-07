package us.ihmc.stateEstimation.humanoid;

import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorMode;

public interface StateEstimatorModeSubscriber
{
   /**
    * Sets the operating mode of the state estimator. This will tell the estimator whether the robot
    * should be fixed in world or if the position of the robot should be estimated. If the robot is
    * hanging in the air and drifting away in normal operating mode this method can be used to fix
    * the robot and avoid the drift.
    * <p>
    * The implementation of this method is estimator specific. However, this method needs to be
    * thread-safe as this might be called from other threads then the one that is running the estimator.
    * </p>
    *
    * @param operatingMode to be set for the estimator.
    */
   public void requestStateEstimatorMode(StateEstimatorMode operatingMode);
}
