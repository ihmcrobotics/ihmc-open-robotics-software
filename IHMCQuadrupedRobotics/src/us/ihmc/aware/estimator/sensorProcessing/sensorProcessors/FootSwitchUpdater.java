package us.ihmc.aware.estimator.sensorProcessing.sensorProcessors;

import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface FootSwitchUpdater
{
   public boolean isFootInContactWithGround(RobotQuadrant footToBeChecked);
}
