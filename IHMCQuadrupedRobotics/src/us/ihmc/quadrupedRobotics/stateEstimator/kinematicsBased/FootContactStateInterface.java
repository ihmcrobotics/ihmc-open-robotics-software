package us.ihmc.quadrupedRobotics.stateEstimator.kinematicsBased;

import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface FootContactStateInterface
{
   public boolean isFootInContactWithGround(RobotQuadrant footToBeChecked);
}
