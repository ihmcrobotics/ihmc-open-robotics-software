package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;

public class SwingSpeedUpCalculator
{
   private final FrameLine2D desiredICPToFinalICPLine = new FrameLine2D();
   private final FrameLineSegment2D desiredICPToFinalICPLineSegment = new FrameLineSegment2D();
   private final FramePoint2D projectedICPEstimate = new FramePoint2D();

   public double estimateDeltaTimeBetweenDesiredICPAndActualICP(FramePoint2DReadOnly desiredICPPosition,
                                                                FramePoint2DReadOnly desiredCMPPosition,
                                                                FramePoint2DReadOnly desiredICPAtTouchdown,
                                                                FramePoint2DReadOnly actualICPPosition,
                                                                double omega0)
   {
      if (desiredICPPosition.distance(desiredICPAtTouchdown) < 1.0e-10)
         return Double.NaN;

      desiredICPToFinalICPLineSegment.set(desiredICPPosition, desiredICPAtTouchdown);
      double percentAlongLineSegmentICP = desiredICPToFinalICPLineSegment.percentageAlongLineSegment(actualICPPosition);
      if (percentAlongLineSegmentICP < 0.0)
      {
         desiredICPToFinalICPLine.set(desiredICPPosition, desiredICPAtTouchdown);
         desiredICPToFinalICPLine.orthogonalProjection(actualICPPosition, projectedICPEstimate);
      }
      else
      {
         desiredICPToFinalICPLineSegment.orthogonalProjection(actualICPPosition, projectedICPEstimate);
      }

      double actualDistanceDueToDisturbance = desiredCMPPosition.distance(projectedICPEstimate);
      double expectedDistanceAccordingToPlan = desiredCMPPosition.distance(desiredICPPosition);

      double distanceRatio = actualDistanceDueToDisturbance / expectedDistanceAccordingToPlan;

      if (distanceRatio < 1.0e-3)
         return 0.0;
      else
         return Math.log(distanceRatio) / omega0;
   }
}
