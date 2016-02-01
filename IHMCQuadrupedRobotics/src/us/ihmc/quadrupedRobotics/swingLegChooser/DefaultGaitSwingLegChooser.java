package us.ihmc.quadrupedRobotics.swingLegChooser;

import us.ihmc.quadrupedRobotics.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class DefaultGaitSwingLegChooser implements NextSwingLegChooser
{
   @Override
   public RobotQuadrant chooseNextSwingLeg(QuadrupedSupportPolygon supportPolygon, RobotQuadrant lastStepQuadrant, FrameVector desiredVelocity, double desiredYawRate)
   {
      if(desiredVelocity.getX() >= 0)
      {
         return lastStepQuadrant.getNextRegularGaitSwingQuadrant();
      }
      
      return lastStepQuadrant.getNextReversedRegularGaitSwingQuadrant();
   }

}
