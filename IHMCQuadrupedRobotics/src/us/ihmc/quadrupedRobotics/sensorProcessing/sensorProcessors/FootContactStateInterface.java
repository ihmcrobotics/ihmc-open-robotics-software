package us.ihmc.quadrupedRobotics.sensorProcessing.sensorProcessors;

import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface FootContactStateInterface
{
   public boolean isFootInContactWithGround(RobotQuadrant footToBeChecked);
}
