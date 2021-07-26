package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.registry.YoRegistry;

public class AchievableCaptureRegionCalculatorWithDelay
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();


   // variables for the capture region calculation
   private final FrameConvexPolygon2D supportFootPolygon = new FrameConvexPolygon2D();
   private final FramePoint2D footCentroid = new FramePoint2D(worldFrame);

   private final FramePoint2D predictedICPAtTouchdown = new FramePoint2D(worldFrame);
   private final FramePoint2D predictedICPAfterTransfer = new FramePoint2D(worldFrame);

   private final FramePoint2D extremeCoP = new FramePoint2D();
   private final FramePoint2D capturePoint = new FramePoint2D(worldFrame);
   private final FrameConvexPolygon2D unconstrainedCaptureRegion = new FrameConvexPolygon2D(worldFrame);
   private final FrameConvexPolygon2D unconstrainedCaptureRegionAtTouchdown = new FrameConvexPolygon2D(worldFrame);

   private final PoseReferenceFrame supportFrame = new PoseReferenceFrame("supportFrame", worldFrame);
   private final ZUpFrame supportSoleZUp = new ZUpFrame(worldFrame, supportFrame, "supportZUpFrame");

   public boolean calculateCaptureRegion(double swingTimeRemaining,
                                         double nextTransferDuration,
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

      swingTimeRemaining = MathTools.clamp(swingTimeRemaining, 0.0, Double.POSITIVE_INFINITY);

      unconstrainedCaptureRegion.clear(supportSoleZUp);
      unconstrainedCaptureRegionAtTouchdown.clear(supportSoleZUp);

      // 2. Get extreme CoP positions
      if (supportFootPolygon.isPointInside(capturePoint))
      {
         // If the ICP is in the support polygon return the whole reachable region.
         unconstrainedCaptureRegion.setToNaN();
         unconstrainedCaptureRegionAtTouchdown.setToNaN();
         return true;
      }

      // 3. For every possible extreme CoP predict the corresponding ICP given the remaining swing time
      for (int i = 0; i < supportFootPolygon.getNumberOfVertices(); i++)
      {
         extremeCoP.setIncludingFrame(supportFootPolygon.getVertex(i));
         extremeCoP.changeFrame(supportSoleZUp);

         CapturePointTools.computeDesiredCapturePointPosition(omega0, swingTimeRemaining, capturePoint, extremeCoP, predictedICPAtTouchdown);
         computeCoPLocationToCapture(predictedICPAtTouchdown, extremeCoP, omega0, nextTransferDuration, predictedICPAfterTransfer);

         unconstrainedCaptureRegionAtTouchdown.addVertexMatchingFrame(predictedICPAtTouchdown, false);
         unconstrainedCaptureRegion.addVertexMatchingFrame(predictedICPAfterTransfer, false);
      }

      unconstrainedCaptureRegionAtTouchdown.update();
      unconstrainedCaptureRegion.update();

      return false;
   }

   public FrameConvexPolygon2DReadOnly getUnconstrainedCaptureRegionAtTouchdown()
   {
      return unconstrainedCaptureRegionAtTouchdown;
   }

   public FrameConvexPolygon2DReadOnly getUnconstrainedCaptureRegion()
   {
      return unconstrainedCaptureRegion;
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
