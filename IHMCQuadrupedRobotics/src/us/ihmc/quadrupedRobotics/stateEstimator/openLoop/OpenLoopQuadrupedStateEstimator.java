package us.ihmc.quadrupedRobotics.stateEstimator.openLoop;

import us.ihmc.quadrupedRobotics.stateEstimator.QuadrupedStateEstimator;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class OpenLoopQuadrupedStateEstimator implements QuadrupedStateEstimator
{

   public OpenLoopQuadrupedStateEstimator()
   {
      // TODO Auto-generated constructor stub
   }
   
   @Override
   public boolean isFootInContact(RobotQuadrant quadrant)
   {
      // TODO Auto-generated method stub
      return false;
   }

   @Override
   public void doControl()
   {
      // TODO Auto-generated method stub
      
   }

}
