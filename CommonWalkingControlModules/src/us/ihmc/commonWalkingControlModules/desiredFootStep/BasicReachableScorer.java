package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.commonWalkingControlModules.captureRegion.CaptureRegionCalculator;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint2d;


public class BasicReachableScorer implements StepLocationScorer
{
   protected final CaptureRegionCalculator captureRegionCalculator;

   public BasicReachableScorer(CaptureRegionCalculator captureRegionCalculator)
   {
      this.captureRegionCalculator = captureRegionCalculator;
   }

   public double getStepLocationScore(RobotSide supportLeg, Footstep desiredFootstep)
   {
      FrameConvexPolygon2d reachableRegion = captureRegionCalculator.getReachableRegion(supportLeg);
      if (reachableRegion.isPointInside(new FramePoint2d(desiredFootstep.getReferenceFrame(), desiredFootstep.getPose().getX(),
              desiredFootstep.getPose().getY())))
         return 1.0;
      else
         return 0.0;
   }
}
