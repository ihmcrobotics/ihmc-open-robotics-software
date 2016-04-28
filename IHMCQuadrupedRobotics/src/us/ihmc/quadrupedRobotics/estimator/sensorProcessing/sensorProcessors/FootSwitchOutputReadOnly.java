package us.ihmc.quadrupedRobotics.estimator.sensorProcessing.sensorProcessors;

import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface FootSwitchOutputReadOnly
{
   public boolean isFootInContact(RobotQuadrant quadrant); 
}
