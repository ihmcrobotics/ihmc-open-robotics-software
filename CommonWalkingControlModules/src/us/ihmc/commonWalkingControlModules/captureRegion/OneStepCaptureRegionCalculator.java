package us.ihmc.commonWalkingControlModules.captureRegion;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;

public class OneStepCaptureRegionCalculator
{
   private final CaptureRegionMathTools captureRegionMath = new CaptureRegionMathTools();

   private static final boolean VISUALIZE = true;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int MAX_CAPTURE_REGION_POLYGON_POINTS = 20;
   private static final int KINEMATIC_LIMIT_POINTS = 8;
   private double reachableRegionCutoffAngle = 1.0;

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final ExecutionTimer globalTimer = new ExecutionTimer(name + "Timer", registry);

   private CaptureRegionVisualizer captureRegionVisualizer = null;
   private final FrameConvexPolygon2d captureRegionPolygon = new FrameConvexPolygon2d(worldFrame);

   // necessary variables for the reachable region and capture calculation:
   private final double midFootAnkleXOffset;
   private final double footWidth;
   private final double kinematicStepRange;
   private final SideDependentList<ReferenceFrame> ankleZUpFrames;
   private final SideDependentList<FrameConvexPolygon2d> reachableRegions;

   public OneStepCaptureRegionCalculator(CommonHumanoidReferenceFrames referenceFrames, WalkingControllerParameters walkingControllerParameters,
         YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(walkingControllerParameters.getFootForwardOffset() - walkingControllerParameters.getFootLength() / 2.0, walkingControllerParameters.getFootWidth(),
            walkingControllerParameters.getMaxStepLength(), referenceFrames.getAnkleZUpReferenceFrames(), parentRegistry, yoGraphicsListRegistry);
   }

   public OneStepCaptureRegionCalculator(double midFootAnkleXOffset, double footWidth, double kinematicStepRange,
         SideDependentList<ReferenceFrame> ankleZUpFrames, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.kinematicStepRange = kinematicStepRange;
      this.ankleZUpFrames = ankleZUpFrames;
      this.midFootAnkleXOffset = midFootAnkleXOffset;
      this.footWidth = footWidth;

      reachableRegions = new SideDependentList<FrameConvexPolygon2d>();
      calculateReachableRegions();

      // set up registry and visualizer
      parentRegistry.addChild(registry);
      if (yoGraphicsListRegistry != null && VISUALIZE)
      {
         captureRegionVisualizer = new CaptureRegionVisualizer(this, yoGraphicsListRegistry, registry);
      }
   }

   private void calculateReachableRegions()
   {
      for (RobotSide side : RobotSide.values)
      {
         FrameConvexPolygon2d reachableRegion = new FrameConvexPolygon2d();
         reachableRegion.clear(ankleZUpFrames.get(side));
         double sign = side == RobotSide.RIGHT ? 1.0 : -1.0;

         for (int i = 0; i < MAX_CAPTURE_REGION_POLYGON_POINTS - 1; i++)
         {
            double angle = sign * reachableRegionCutoffAngle * Math.PI * ((double) i) / ((double) (MAX_CAPTURE_REGION_POLYGON_POINTS - 2));
            double x = kinematicStepRange * Math.cos(angle) + midFootAnkleXOffset;
            double y = kinematicStepRange * Math.sin(angle);
            if (Math.abs(y) < footWidth / 2.0)
               y = sign * footWidth / 2.0;
            reachableRegion.addVertex(ankleZUpFrames.get(side), x, y);
         }
         reachableRegion.addVertex(ankleZUpFrames.get(side), midFootAnkleXOffset, sign * footWidth / 2.0);
         reachableRegion.update();
         reachableRegions.set(side, reachableRegion);
      }
   }

   // variables for the capture region calculation
   private static final int APPROXIMATION_MULTILIER = 100;
   private RobotSide previousSwingSide;
   private final FrameConvexPolygon2d supportFootPolygon = new FrameConvexPolygon2d();
   private final FramePoint2d footCentroid = new FramePoint2d(worldFrame);
   private final FramePoint2d predictedICP = new FramePoint2d(worldFrame);
   private final FramePoint2d capturePoint = new FramePoint2d(worldFrame);
   private final FramePoint2d kinematicExtreme = new FramePoint2d(worldFrame);
   private final FramePoint2d additionalKinematicPoint = new FramePoint2d(worldFrame);
   private final FrameVector2d projectedLine = new FrameVector2d(worldFrame);
   private final FrameVector2d firstKinematicExtremeDirection = new FrameVector2d(worldFrame);
   private final FrameVector2d lastKinematicExtremeDirection = new FrameVector2d(worldFrame);
   private final FrameConvexPolygon2d rawCaptureRegion = new FrameConvexPolygon2d(worldFrame);

   public void calculateCaptureRegion(RobotSide swingSide, double swingTimeRemaining, FramePoint2d icp, double omega0, FrameConvexPolygon2d footPolygon)
   {
      globalTimer.startMeasurement();

      // 1. Set up all needed variables and reference frames for the calculation:
      ReferenceFrame supportAnkleZUp = ankleZUpFrames.get(swingSide.getOppositeSide());
      if (!swingSide.equals(previousSwingSide))
      {
         // change the support foot polygon only if the swing side changed to avoid garbage every tick.
         this.supportFootPolygon.setIncludingFrameAndUpdate(footPolygon);
         this.supportFootPolygon.changeFrameAndProjectToXYPlane(supportAnkleZUp);
         previousSwingSide = swingSide;
      }
      capturePoint.setIncludingFrame(icp);
      capturePoint.changeFrame(supportAnkleZUp);
      footCentroid.changeFrame(supportAnkleZUp);
      firstKinematicExtremeDirection.changeFrame(supportAnkleZUp);
      lastKinematicExtremeDirection.changeFrame(supportAnkleZUp);
      predictedICP.changeFrame(supportAnkleZUp);
      projectedLine.changeFrame(supportAnkleZUp);

      swingTimeRemaining = MathTools.clamp(swingTimeRemaining, 0.0, Double.POSITIVE_INFINITY);
      supportFootPolygon.getCentroid(footCentroid);
      rawCaptureRegion.clear(supportAnkleZUp);
      captureRegionPolygon.clear(supportAnkleZUp);

      // 2. Get extreme CoP positions
      ArrayList<FramePoint2d> extremesOfFeasibleCOP = supportFootPolygon.getAllVisibleVerticesFromOutsideLeftToRightCopy(capturePoint);
      if (extremesOfFeasibleCOP == null)
      {
         // If the ICP is in the support polygon return the whole reachable region.
         globalTimer.stopMeasurement();
         captureRegionPolygon.setIncludingFrameAndUpdate(reachableRegions.get(swingSide.getOppositeSide()));
         updateVisualizer();
         return;
      }

      // 3. For every possible extreme CoP predict the corresponding ICP given the remaining swing time
      for (int i = 0; i < extremesOfFeasibleCOP.size(); i++)
      {
         FramePoint2d copExtreme = extremesOfFeasibleCOP.get(i);
         copExtreme.changeFrame(supportAnkleZUp);
         CaptureRegionMathTools.predictCapturePoint(capturePoint, copExtreme, swingTimeRemaining, omega0, predictedICP);
         rawCaptureRegion.addVertexChangeFrameAndProjectToXYPlane(predictedICP);

         // 4. Project the predicted ICP on a circle around the foot with the radius of the step range.
         projectedLine.set(predictedICP);
         projectedLine.sub(copExtreme);
         CaptureRegionMathTools.solveIntersectionOfRayAndCircle(footCentroid, predictedICP, projectedLine, APPROXIMATION_MULTILIER * kinematicStepRange,
               kinematicExtreme);

         if (kinematicExtreme.containsNaN())
         {
            globalTimer.stopMeasurement();
            captureRegionPolygon.update();
            return;
         }
         rawCaptureRegion.addVertexChangeFrameAndProjectToXYPlane(kinematicExtreme);

         if (i == 0)
         {
            firstKinematicExtremeDirection.set(kinematicExtreme);
            firstKinematicExtremeDirection.sub(footCentroid);
         }
         else
         {
            lastKinematicExtremeDirection.set(kinematicExtreme);
            lastKinematicExtremeDirection.sub(footCentroid);
         }
      }

      // 5. Add additional points to the capture region polygon on the circle between the kinematic extreme points
      for (int i = 0; i < KINEMATIC_LIMIT_POINTS - 1; i++)
      {
         double alphaFromAToB = ((double) (i + 1)) / ((double) (KINEMATIC_LIMIT_POINTS + 1));
         captureRegionMath.getPointBetweenVectorsAtDistanceFromOriginCircular(firstKinematicExtremeDirection, lastKinematicExtremeDirection, alphaFromAToB,
               APPROXIMATION_MULTILIER * kinematicStepRange, footCentroid, additionalKinematicPoint);
         rawCaptureRegion.addVertexAndChangeFrame(additionalKinematicPoint);
      }

      // 6. Intersect the capture region with the reachable region
      if (!rawCaptureRegion.isEmpty())
      {
         // This causes the capture region to always be null if it is null once.
         // This assumes that once there is no capture region the robot will fall for sure.
         rawCaptureRegion.update();
         rawCaptureRegion.intersectionWith(reachableRegions.get(swingSide.getOppositeSide()), captureRegionPolygon);
      }

      captureRegionPolygon.update();

      globalTimer.stopMeasurement();
      updateVisualizer();
   }

   private void updateVisualizer()
   {
      if (captureRegionVisualizer != null && captureRegionPolygon != null)
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

   public FrameConvexPolygon2d getCaptureRegion()
   {
      return captureRegionPolygon;
   }

   public void setReachableRegionCutoffAngle(double reachableRegionCutoffAngle)
   {
      this.reachableRegionCutoffAngle = reachableRegionCutoffAngle;
      calculateReachableRegions();
   }

   public FrameConvexPolygon2d getReachableRegion(RobotSide robotSide)
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
}
