package us.ihmc.stateEstimation.quadruped.kinematicsBasedStateEstimation;

import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class FootContactStateBasedOnOtherFeet implements FootContactStateInterface
{

   private QuadrantDependentList<YoFramePoint> footPositions = new QuadrantDependentList<YoFramePoint>();
   
   public FootContactStateBasedOnOtherFeet()
   {
   }

   @Override
   public boolean isFootInContactWithGround(RobotQuadrant footToBeChecked)
   {
      
      
      
      return false;
   }

}
