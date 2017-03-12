package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;

public interface ICPPlannerWithTimeFreezer extends ICPPlanner
{
   public boolean getIsTimeBeingFrozen();

   public void getDesiredCapturePointPositionAndVelocity(FramePoint2d desiredCapturePointPositionToPack, FrameVector2d desiredCapturePointVelocityToPack,
         FramePoint2d currentCapturePointPosition, double time);
}
