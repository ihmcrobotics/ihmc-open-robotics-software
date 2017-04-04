package us.ihmc.robotics.testing;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.math.frames.YoFramePoint;

public class YoGeometryTestGoals
{
   public static YoVariableTestGoal boundingBox(YoFramePoint yoFramePoint, Point3DReadOnly boxCenter, double boxEpsilon)
   {
      YoVariableTestGoal goalX = YoVariableTestGoal.doubleWithinEpsilon(yoFramePoint.getYoX(), boxCenter.getX(), boxEpsilon);
      YoVariableTestGoal goalY = YoVariableTestGoal.doubleWithinEpsilon(yoFramePoint.getYoY(), boxCenter.getY(), boxEpsilon);
      YoVariableTestGoal goalZ = YoVariableTestGoal.doubleWithinEpsilon(yoFramePoint.getYoZ(), boxCenter.getZ(), boxEpsilon);
      
      return new YoVariableTestGoal(yoFramePoint.getYoX(), yoFramePoint.getYoY(), yoFramePoint.getYoZ())
      {
         @Override
         public boolean currentlyMeetsGoal()
         {
            return goalX.currentlyMeetsGoal() && goalY.currentlyMeetsGoal() && goalZ.currentlyMeetsGoal();
         }

         @Override
         public String toString()
         {
            return "\n" + goalX.toString() + "\n" + goalY.toString() + "\n" + goalZ.toString();
         }
      };
   }
}
