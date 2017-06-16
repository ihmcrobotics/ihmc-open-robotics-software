package us.ihmc.commonWalkingControlModules.configurations;

import java.util.List;

import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.robotSide.RobotSide;

public abstract class ExtendedCapturePointPlannerParameters extends CapturePointPlannerParameters
{
   public abstract List<FrameVector2d> getCoPOffsets(RobotSide side);
   
   public int getNumberOfPointsPerFoot()
   {
      return 2;
   }
   
   public int getOrderOfCoPInterpolation()
   {
      return 3;
   }
   
}
