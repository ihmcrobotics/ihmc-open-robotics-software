package us.ihmc.aware.estimator.sensorProcessing.sensorProcessors;

import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface FootSwitchOutputReadOnly
{
   public boolean isFootInContact(RobotQuadrant quadrant); 
}
