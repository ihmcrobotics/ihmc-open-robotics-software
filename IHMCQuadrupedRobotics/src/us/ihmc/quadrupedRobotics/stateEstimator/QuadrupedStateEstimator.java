package us.ihmc.quadrupedRobotics.stateEstimator;

import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface QuadrupedStateEstimator
{
   public boolean isFootInContact(RobotQuadrant quadrant);

}
