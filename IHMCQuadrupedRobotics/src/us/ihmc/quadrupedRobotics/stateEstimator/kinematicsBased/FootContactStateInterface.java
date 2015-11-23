package us.ihmc.stateEstimation.quadruped.kinematicsBasedStateEstimation;

import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface FootContactStateInterface
{
   public boolean isFootInContactWithGround(RobotQuadrant footToBeChecked);
}
