package us.ihmc.commonWalkingControlModules.configurations;

import java.util.List;

import us.ihmc.robotics.geometry.FrameVector2d;

public abstract class ExtendedCapturePointPlannerParameters extends CapturePointPlannerParameters
{
   public abstract List<FrameVector2d> getCoPOffsets();
   
   public int getNumberOfPointsPerFoot()
   {
      return 2;
   }
   
   public int getOrderOfCoPInterpolation()
   {
      return 3;
   }
   
}
