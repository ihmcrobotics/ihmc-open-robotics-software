package us.ihmc.quadrupedRobotics.estimator.sensorProcessing.sensorProcessors;

import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface FootSwitchUpdater
{
   public boolean isFootInContactWithGround(RobotQuadrant footToBeChecked);
}
