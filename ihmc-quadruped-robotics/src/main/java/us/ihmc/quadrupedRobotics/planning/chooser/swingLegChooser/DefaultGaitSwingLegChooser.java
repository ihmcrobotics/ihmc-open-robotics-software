package us.ihmc.quadrupedRobotics.planning.chooser.swingLegChooser;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.quadrupedRobotics.geometry.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class DefaultGaitSwingLegChooser implements NextSwingLegChooser
{
   @Override
   public RobotQuadrant chooseNextSwingLeg(QuadrupedSupportPolygon supportPolygon, RobotQuadrant lastStepQuadrant, FrameVector3D desiredVelocity, double desiredYawRate)
   {
      if(desiredVelocity.getX() >= 0)
      {
         return lastStepQuadrant.getNextRegularGaitSwingQuadrant();
      }
      
      return lastStepQuadrant.getNextReversedRegularGaitSwingQuadrant();
   }

}
