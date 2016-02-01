package us.ihmc.quadrupedRobotics.stateEstimator;

import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface QuadrupedStateEstimator
{
   public void initialize();
   
   public void enable();
   
   public void doControl();
   
   public boolean isFootInContact(RobotQuadrant quadrant);
   
   public double getCurrentTime();

}
