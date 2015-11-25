package us.ihmc.quadrupedRobotics.sensorProcessing.sensorProcessors;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class FootContactStateBasedOnOtherFeet implements FootContactStateInterface
{ 
   private static final double EPSILON = 1e-5;
   private QuadrantDependentList<FramePoint> footPositions = new QuadrantDependentList<FramePoint>();
   
   public FootContactStateBasedOnOtherFeet(QuadrantDependentList<FramePoint> footPositions)
   {
      this.footPositions = footPositions;
   }

   @Override
   public boolean isFootInContactWithGround(RobotQuadrant footToBeChecked)
   {
      double footToBeCheckedZ = footPositions.get(footToBeChecked).getZ(); 
      double otherFeetZ = 0.0;
      
      for(RobotQuadrant robotQuadrant : RobotQuadrant.values())
      {
         if(robotQuadrant != footToBeChecked)
         {
            otherFeetZ += footPositions.get(robotQuadrant).getZ();
         }
         otherFeetZ /= 3.0;
      }
     
      if(MathTools.epsilonEquals(footToBeCheckedZ, otherFeetZ, EPSILON))
         return true;
      
      return false;
   }
}


