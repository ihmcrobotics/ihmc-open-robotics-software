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
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.registry.YoRegistry;

public class AchievableCaptureRegionCalculatorWithDelay
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int MAX_CAPTURE_REGION_POLYGON_POINTS = 20;
   private static final boolean VISUALIZE = true;

   private double reachableRegionCutoffAngle = 1.0;

   private CaptureRegionVisualizer captureRegionVisualizer = null;
   private final FrameConvexPolygon2D captureRegionPolygon = new FrameConvexPolygon2D(worldFrame);

   // necessary variables for the reachable region and capture calculation:
   //   private final double midFootAnkleXOffset;
   private final double footWidth;
   private final double kinematicStepRange;
   private final SideDependentList<? extends ReferenceFrame> soleZUpFrames;
   private final SideDependentList<FrameConvexPolygon2D> reachableRegions = new SideDependentList<>(new FrameConvexPolygon2D(), new FrameConvexPolygon2D());

   private final ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

   public AchievableCaptureRegionCalculatorWithDelay(double footWidth,
                                                     double kinematicStepRange,
                                                     SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                                     String suffix,
                                                     YoRegistry parentRegistry,
                                                     YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.kinematicStepRange = kinematicStepRange;
      this.soleZUpFrames = soleZUpFrames;
      //      this.midFootAnkleXOffset = midFootAnkleXOffset;
      this.footWidth = footWidth;

      calculateReachableRegions(footWidth);

      // set up registry and visualizer
      if (yoGraphicsListRegistry != null && VISUALIZE)
      {
         YoRegistry registry = new YoRegistry(getClass().getSimpleName());
         parentRegistry.addChild(registry);
         captureRegionVisualizer = new CaptureRegionVisualizer(this::getUnconstrainedCaptureRegion, suffix, yoGraphicsListRegistry, registry);
      }
   }

   private void calculateReachableRegions(double footWidth)
   {
      for (RobotSide side : RobotSide.values)
      {
         FrameConvexPolygon2D reachableRegion = reachableRegions.get(side);
         reachableRegion.clear(soleZUpFrames.get(side));
         double sign = side.negateIfLeftSide(1.0);

         for (int i = 0; i < MAX_CAPTURE_REGION_POLYGON_POINTS - 1; i++)
         {
            double angle = sign * reachableRegionCutoffAngle * Math.PI * (i) / (MAX_CAPTURE_REGION_POLYGON_POINTS - 2);
            double x = kinematicStepRange * Math.cos(angle);
            double y = kinematicStepRange * Math.sin(angle);
            if (Math.abs(y) < footWidth / 2.0)
               y = sign * footWidth / 2.0;
            reachableRegion.addVertex(soleZUpFrames.get(side), x, y);
         }
         reachableRegion.addVertex(soleZUpFrames.get(side), 0, sign * footWidth / 2.0);
         reachableRegion.update();
      }
   }

   // variables for the capture region calculation
   private final FrameConvexPolygon2D supportFootPolygon = new FrameConvexPolygon2D();
   private final FramePoint2D footCentroid = new FramePoint2D(worldFrame);

   private final FramePoint2D predictedICPAtTouchdown = new FramePoint2D(worldFrame);
   private final FramePoint2D predictedICPAfterTransfer = new FramePoint2D(worldFrame);

   private final FramePoint2D extremeCoP = new FramePoint2D();
   private final FramePoint2D capturePoint = new FramePoint2D(worldFrame);
   private final FrameConvexPolygon2D unconstrainedCaptureRegion = new FrameConvexPolygon2D(worldFrame);

   public void calculateCaptureRegion(RobotSide swingSide,
                                      double swingTimeRemaining,
                                      double nextTransferDuration,
                                      FramePoint2DReadOnly currentICP,
                                      double omega0,
                                      FrameConvexPolygon2DReadOnly footPolygon)
   {
      // 1. Set up all needed variables and reference frames for the calculation:
      ReferenceFrame supportSoleZUp = soleZUpFrames.get(swingSide.getOppositeSide());

      this.supportFootPolygon.setIncludingFrame(footPolygon);
      this.supportFootPolygon.changeFrameAndProjectToXYPlane(supportSoleZUp);

      capturePoint.setIncludingFrame(currentICP);
      capturePoint.changeFrame(supportSoleZUp);

      footCentroid.setIncludingFrame(supportFootPolygon.getCentroid());

      predictedICPAtTouchdown.setToZero(supportSoleZUp);

      swingTimeRemaining = MathTools.clamp(swingTimeRemaining, 0.0, Double.POSITIVE_INFINITY);

      unconstrainedCaptureRegion.clear(supportSoleZUp);
      captureRegionPolygon.clear(supportSoleZUp);

      // 2. Get extreme CoP positions
      FrameConvexPolygon2D reachableRegion = reachableRegions.get(swingSide.getOppositeSide());
      if (supportFootPolygon.isPointInside(capturePoint))
      {
         // If the ICP is in the support polygon return the whole reachable region.
         captureRegionPolygon.setIncludingFrame(reachableRegion);
         updateVisualizer();
         return;
      }

      // 3. For every possible extreme CoP predict the corresponding ICP given the remaining swing time
      for (int i = 0; i < supportFootPolygon.getNumberOfVertices(); i++)
      {
         extremeCoP.setIncludingFrame(supportFootPolygon.getVertex(i));
         extremeCoP.changeFrame(supportSoleZUp);

         CapturePointTools.computeDesiredCapturePointPosition(omega0, swingTimeRemaining, capturePoint, extremeCoP, predictedICPAtTouchdown);
         computeCoPLocationToCapture(predictedICPAtTouchdown, extremeCoP, omega0, nextTransferDuration, predictedICPAfterTransfer);

         unconstrainedCaptureRegion.addVertexMatchingFrame(predictedICPAfterTransfer, false);
      }

      // 6. Intersect the capture region with the reachable region
      if (!unconstrainedCaptureRegion.isEmpty())
      {
         // This causes the capture region to always be null if it is null once.
         // This assumes that once there is no capture region the robot will fall for sure.
         unconstrainedCaptureRegion.update();
         unconstrainedCaptureRegion.checkReferenceFrameMatch(reachableRegion);

         captureRegionPolygon.clear(unconstrainedCaptureRegion.getReferenceFrame());
         convexPolygonTools.computeIntersectionOfPolygons(unconstrainedCaptureRegion, reachableRegion, captureRegionPolygon);
      }

      captureRegionPolygon.update();
      updateVisualizer();
   }

   private void updateVisualizer()
   {
      if (captureRegionVisualizer != null)
      {
         captureRegionVisualizer.update();
      }
      else
      {
         hideCaptureRegion();
      }
   }

   public void hideCaptureRegion()
   {
      if (captureRegionVisualizer != null)
      {
         captureRegionVisualizer.hide();
      }
   }

   public FrameConvexPolygon2DReadOnly getCaptureRegion()
   {
      return captureRegionPolygon;
   }

   public FrameConvexPolygon2DReadOnly getUnconstrainedCaptureRegion()
   {
      return unconstrainedCaptureRegion;
   }

   public void setReachableRegionCutoffAngle(double reachableRegionCutoffAngle)
   {
      this.reachableRegionCutoffAngle = reachableRegionCutoffAngle;
      calculateReachableRegions(footWidth);
   }

   public FrameConvexPolygon2D getReachableRegion(RobotSide robotSide)
   {
      return reachableRegions.get(robotSide);
   }

   public double getKinematicStepRange()
   {
      return kinematicStepRange;
   }

   public double getCaptureRegionArea()
   {
      captureRegionPolygon.update();
      return captureRegionPolygon.getArea();
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
