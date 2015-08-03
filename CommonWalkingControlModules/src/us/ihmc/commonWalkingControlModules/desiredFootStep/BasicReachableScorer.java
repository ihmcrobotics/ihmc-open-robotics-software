package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.commonWalkingControlModules.captureRegion.OneStepCaptureRegionCalculator;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.robotSide.RobotSide;


public class BasicReachableScorer implements StepLocationScorer
{
   protected final OneStepCaptureRegionCalculator captureRegionCalculator;

   public BasicReachableScorer(OneStepCaptureRegionCalculator captureRegionCalculator)
   {
      this.captureRegionCalculator = captureRegionCalculator;
   }

   public double getStepLocationScore(RobotSide supportLeg, FramePose desiredFootPose)
   {
      FrameConvexPolygon2d reachableRegion = captureRegionCalculator.getReachableRegion(supportLeg);
      if (reachableRegion.isPointInside(new FramePoint2d(desiredFootPose.getReferenceFrame(), desiredFootPose.getX(), desiredFootPose.getY())))
         return 1.0;
      else
         return 0.0;
   }
}
