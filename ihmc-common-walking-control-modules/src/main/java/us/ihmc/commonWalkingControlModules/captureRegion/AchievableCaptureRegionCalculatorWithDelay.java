package us.ihmc.commonWalkingControlModules.captureRegion;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;

public class AchievableCaptureRegionCalculatorWithDelay
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final boolean includeSlowerTimeInRegion = false;

   // variables for the capture region calculation
   private final FrameConvexPolygon2D supportFootPolygon = new FrameConvexPolygon2D();
   private final FramePoint2D footCentroid = new FramePoint2D(worldFrame);

   private final FramePoint2D predictedICPAtTouchdown = new FramePoint2D(worldFrame);
   private final FramePoint2D predictedICPAfterTransfer = new FramePoint2D(worldFrame);

   private final FramePoint2D extremeCoP = new FramePoint2D();
   private final FramePoint2D capturePoint = new FramePoint2D(worldFrame);
   private final FrameConvexPolygon2D captureRegion = new FrameConvexPolygon2D(worldFrame);

   private final PoseReferenceFrame supportFrame = new PoseReferenceFrame("supportFrame", worldFrame);
   private final ZUpFrame supportSoleZUp = new ZUpFrame(worldFrame, supportFrame, "supportZUpFrame");

   private final TDoubleArrayList candidateSwingTimes = new TDoubleArrayList();

   public boolean calculateCaptureRegion(double nextTransferDuration,
                                         double swingTimeRemaining,
                                         FramePoint2DReadOnly currentICP,
                                         double omega0,
                                         FramePose3DReadOnly supportPose,
                                         FrameConvexPolygon2DReadOnly footPolygon)
   {
      candidateSwingTimes.reset();
      candidateSwingTimes.add(swingTimeRemaining);

      return calculateCaptureRegion(nextTransferDuration, candidateSwingTimes, currentICP, omega0, supportPose, footPolygon);
   }

   public boolean calculateCaptureRegion(double nextTransferDuration,
                                         TDoubleArrayList candidateSwingTimes,
                                         FramePoint2DReadOnly currentICP,
                                         double omega0,
                                         FramePose3DReadOnly supportPose,
                                         FrameConvexPolygon2DReadOnly footPolygon)
   {
      // 1. Set up all needed variables and reference frames for the calculation:
      supportFrame.setPoseAndUpdate(supportPose);
      supportSoleZUp.update();

      this.supportFootPolygon.setIncludingFrame(footPolygon);
      this.supportFootPolygon.changeFrameAndProjectToXYPlane(supportSoleZUp);

      capturePoint.setIncludingFrame(currentICP);
      capturePoint.changeFrame(supportSoleZUp);

      footCentroid.setIncludingFrame(supportFootPolygon.getCentroid());

      predictedICPAtTouchdown.setToZero(supportSoleZUp);

      captureRegion.clear(supportSoleZUp);

      // 2. Get extreme CoP positions
      if (supportFootPolygon.isPointInside(capturePoint))
      {
         // If the ICP is in the support polygon return the whole reachable region.
         captureRegion.setToNaN();
         return true;
      }

      // 3. For every possible extreme CoP predict the corresponding ICP given the remaining swing time
      for (int i = 0; i < supportFootPolygon.getNumberOfVertices(); i++)
      {
         extremeCoP.setIncludingFrame(supportFootPolygon.getVertex(i));
         extremeCoP.changeFrame(supportSoleZUp);

         // compute min
         for (int timeIdx = 0; timeIdx < candidateSwingTimes.size(); timeIdx++)
         {
            CapturePointTools.computeDesiredCapturePointPosition(omega0, candidateSwingTimes.get(timeIdx), capturePoint, extremeCoP, predictedICPAtTouchdown);
            computeCoPLocationToCapture(predictedICPAtTouchdown, extremeCoP, omega0, nextTransferDuration, predictedICPAfterTransfer);

            captureRegion.addVertexMatchingFrame(predictedICPAfterTransfer, false);
         }
      }

      captureRegion.update();

      return false;
   }

   public boolean calculateCaptureRegionAtFixedTime(double nextTransferDuration,
                                         double remainingSwingDuration,
                                         FramePoint2DReadOnly currentICP,
                                         double omega0,
                                         FramePose3DReadOnly supportPose,
                                         FrameConvexPolygon2DReadOnly footPolygon)
   {
      // 1. Set up all needed variables and reference frames for the calculation:
      supportFrame.setPoseAndUpdate(supportPose);
      supportSoleZUp.update();

      this.supportFootPolygon.setIncludingFrame(footPolygon);
      this.supportFootPolygon.changeFrameAndProjectToXYPlane(supportSoleZUp);

      capturePoint.setIncludingFrame(currentICP);
      capturePoint.changeFrame(supportSoleZUp);

      footCentroid.setIncludingFrame(supportFootPolygon.getCentroid());

      predictedICPAtTouchdown.setToZero(supportSoleZUp);

      captureRegion.clear(supportSoleZUp);

      // 2. Get extreme CoP positions
      if (supportFootPolygon.isPointInside(capturePoint))
      {
         // If the ICP is in the support polygon return the whole reachable region.
         captureRegion.setToNaN();
         return true;
      }

      // 3. For every possible extreme CoP predict the corresponding ICP given the remaining swing time
      for (int i = 0; i < supportFootPolygon.getNumberOfVertices(); i++)
      {
         extremeCoP.setIncludingFrame(supportFootPolygon.getVertex(i));
         extremeCoP.changeFrame(supportSoleZUp);

         CapturePointTools.computeDesiredCapturePointPosition(omega0, remainingSwingDuration, capturePoint, extremeCoP, predictedICPAtTouchdown);
         computeCoPLocationToCapture(predictedICPAtTouchdown, extremeCoP, omega0, nextTransferDuration, predictedICPAfterTransfer);

         captureRegion.addVertexMatchingFrame(predictedICPAfterTransfer, false);
      }

      captureRegion.update();

      return false;
   }

   public FrameConvexPolygon2DReadOnly getCaptureRegion()
   {
      return captureRegion;
   }

   public static void computeCoPLocationToCapture(FramePoint2DReadOnly initialICP,
                                                  FramePoint2DReadOnly initialCMP,
                                                  double omega,
                                                  double duration,
                                                  FramePoint2DBasics finalCMPToPack)
   {
      finalCMPToPack.setToZero(initialCMP.getReferenceFrame());
      double omegaT = omega * duration;
      double exponential = Math.exp(omegaT);
      finalCMPToPack.setAndScale(omegaT * exponential, initialICP);
      finalCMPToPack.scaleAdd(exponential * (1.0 - omegaT) - 1.0, initialCMP, finalCMPToPack);
      finalCMPToPack.scale(1.0 / (exponential - 1.0));
   }
}
