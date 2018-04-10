package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

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
   private final FrameConvexPolygon2D captureRegionPolygon = new FrameConvexPolygon2D(worldFrame);

   // necessary variables for the reachable region and capture calculation:
   private final double midFootAnkleXOffset;
   private final double footWidth;
   private final double kinematicStepRange;
   private final SideDependentList<? extends ReferenceFrame> ankleZUpFrames;
   private final SideDependentList<FrameConvexPolygon2D> reachableRegions;

   private final RecyclingArrayList<FramePoint2D> visibleVertices = new RecyclingArrayList<>(MAX_CAPTURE_REGION_POLYGON_POINTS, FramePoint2D.class);

   private final ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

   public OneStepCaptureRegionCalculator(CommonHumanoidReferenceFrames referenceFrames, WalkingControllerParameters walkingControllerParameters,
         YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(walkingControllerParameters.getSteppingParameters().getFootForwardOffset()
            - walkingControllerParameters.getSteppingParameters().getFootLength() / 2.0,
           walkingControllerParameters.getSteppingParameters().getFootWidth(), walkingControllerParameters.getSteppingParameters().getMaxStepLength(),
           referenceFrames.getAnkleZUpReferenceFrames(), "", parentRegistry, yoGraphicsListRegistry);
   }

   public OneStepCaptureRegionCalculator(CommonHumanoidReferenceFrames referenceFrames, WalkingControllerParameters walkingControllerParameters,
                                         String suffix, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(walkingControllerParameters.getSteppingParameters().getFootForwardOffset()
                 - walkingControllerParameters.getSteppingParameters().getFootLength() / 2.0,
           walkingControllerParameters.getSteppingParameters().getFootWidth(), walkingControllerParameters.getSteppingParameters().getMaxStepLength(),
           referenceFrames.getAnkleZUpReferenceFrames(), suffix, parentRegistry, yoGraphicsListRegistry);
   }

   public OneStepCaptureRegionCalculator(BipedSupportPolygons referenceFrames, WalkingControllerParameters walkingControllerParameters,
                                         YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(walkingControllerParameters.getSteppingParameters().getFootForwardOffset()
                 - walkingControllerParameters.getSteppingParameters().getFootLength() / 2.0,
           walkingControllerParameters.getSteppingParameters().getFootWidth(), walkingControllerParameters.getSteppingParameters().getMaxStepLength(),
           referenceFrames.getAnkleZUpFrames(), "", parentRegistry, yoGraphicsListRegistry);
   }

   public OneStepCaptureRegionCalculator(BipedSupportPolygons referenceFrames, WalkingControllerParameters walkingControllerParameters, String suffix,
                                         YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(walkingControllerParameters.getSteppingParameters().getFootForwardOffset()
                 - walkingControllerParameters.getSteppingParameters().getFootLength() / 2.0,
           walkingControllerParameters.getSteppingParameters().getFootWidth(), walkingControllerParameters.getSteppingParameters().getMaxStepLength(),
           referenceFrames.getAnkleZUpFrames(), suffix, parentRegistry, yoGraphicsListRegistry);
   }

   public OneStepCaptureRegionCalculator(double midFootAnkleXOffset, double footWidth, double kinematicStepRange,
                                         SideDependentList<? extends ReferenceFrame> ankleZUpFrames, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(midFootAnkleXOffset, footWidth, kinematicStepRange, ankleZUpFrames, "", parentRegistry, yoGraphicsListRegistry);
   }

   public OneStepCaptureRegionCalculator(double midFootAnkleXOffset, double footWidth, double kinematicStepRange,
         SideDependentList<? extends ReferenceFrame> ankleZUpFrames, String suffix, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.kinematicStepRange = kinematicStepRange;
      this.ankleZUpFrames = ankleZUpFrames;
      this.midFootAnkleXOffset = midFootAnkleXOffset;
      this.footWidth = footWidth;

      reachableRegions = new SideDependentList<FrameConvexPolygon2D>();
      calculateReachableRegions();

      // set up registry and visualizer
      parentRegistry.addChild(registry);
      if (yoGraphicsListRegistry != null && VISUALIZE)
      {
         captureRegionVisualizer = new CaptureRegionVisualizer(this, suffix, yoGraphicsListRegistry, registry);
      }
   }

   private void calculateReachableRegions()
   {
      for (RobotSide side : RobotSide.values)
      {
         FrameConvexPolygon2D reachableRegion = new FrameConvexPolygon2D();
         reachableRegion.clear(ankleZUpFrames.get(side));
         double sign = side == RobotSide.RIGHT ? 1.0 : -1.0;

         for (int i = 0; i < MAX_CAPTURE_REGION_POLYGON_POINTS - 1; i++)
         {
            double angle = sign * reachableRegionCutoffAngle * Math.PI * (i) / (MAX_CAPTURE_REGION_POLYGON_POINTS - 2);
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
   private final FrameConvexPolygon2D supportFootPolygon = new FrameConvexPolygon2D();
   private final FramePoint2D footCentroid = new FramePoint2D(worldFrame);
   private final FramePoint2D predictedICP = new FramePoint2D(worldFrame);
   private final FramePoint2D capturePoint = new FramePoint2D(worldFrame);
   private final FramePoint2D kinematicExtreme = new FramePoint2D(worldFrame);
   private final FramePoint2D additionalKinematicPoint = new FramePoint2D(worldFrame);
   private final FrameVector2D projectedLine = new FrameVector2D(worldFrame);
   private final FrameVector2D firstKinematicExtremeDirection = new FrameVector2D(worldFrame);
   private final FrameVector2D lastKinematicExtremeDirection = new FrameVector2D(worldFrame);
   private final FrameConvexPolygon2D rawCaptureRegion = new FrameConvexPolygon2D(worldFrame);

   public void calculateCaptureRegion(RobotSide swingSide, double swingTimeRemaining, FramePoint2DReadOnly icp, double omega0, FrameConvexPolygon2D footPolygon)
   {
      globalTimer.startMeasurement();

      // 1. Set up all needed variables and reference frames for the calculation:
      ReferenceFrame supportAnkleZUp = ankleZUpFrames.get(swingSide.getOppositeSide());
      if (!swingSide.equals(previousSwingSide))
      {
         // change the support foot polygon only if the swing side changed to avoid garbage every tick.
         this.supportFootPolygon.setIncludingFrame(footPolygon);
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
      footCentroid.setIncludingFrame(supportFootPolygon.getCentroid());
      rawCaptureRegion.clear(supportAnkleZUp);
      captureRegionPolygon.clear(supportAnkleZUp);

      // 2. Get extreme CoP positions
      boolean icpOusideSupport = computeVisibleVerticesFromOutsideLeftToRightCopy(supportFootPolygon, capturePoint);
      FrameConvexPolygon2D reachableRegion = reachableRegions.get(swingSide.getOppositeSide());
      if (!icpOusideSupport)
      {
         // If the ICP is in the support polygon return the whole reachable region.
         globalTimer.stopMeasurement();
         captureRegionPolygon.setIncludingFrame(reachableRegion);
         updateVisualizer();
         return;
      }

      // 3. For every possible extreme CoP predict the corresponding ICP given the remaining swing time
      for (int i = 0; i < visibleVertices.size(); i++)
      {
         FramePoint2D copExtreme = visibleVertices.get(i);
         copExtreme.changeFrame(supportAnkleZUp);
         CaptureRegionMathTools.predictCapturePoint(capturePoint, copExtreme, swingTimeRemaining, omega0, predictedICP);
         rawCaptureRegion.addVertexMatchingFrame(predictedICP, false);

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
         rawCaptureRegion.addVertexMatchingFrame(kinematicExtreme, false);

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
         rawCaptureRegion.addVertexMatchingFrame(additionalKinematicPoint, false);
      }

      // 6. Intersect the capture region with the reachable region
      if (!rawCaptureRegion.isEmpty())
      {
         // This causes the capture region to always be null if it is null once.
         // This assumes that once there is no capture region the robot will fall for sure.
         rawCaptureRegion.update();
         rawCaptureRegion.checkReferenceFrameMatch(reachableRegion);
         captureRegionPolygon.clear(rawCaptureRegion.getReferenceFrame());
         convexPolygonTools.computeIntersectionOfPolygons(rawCaptureRegion, reachableRegion, captureRegionPolygon);
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

   public FrameConvexPolygon2D getCaptureRegion()
   {
      return captureRegionPolygon;
   }

   public void setReachableRegionCutoffAngle(double reachableRegionCutoffAngle)
   {
      this.reachableRegionCutoffAngle = reachableRegionCutoffAngle;
      calculateReachableRegions();
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

   /**
    * Returns all of the vertices that are visible from the observerPoint2d, in left to right order.
    * If the observerPoint2d is inside the polygon, returns null.
    *
    * @param observerFramePoint Point2d
    * @return Point2d[]
    */
   public boolean computeVisibleVerticesFromOutsideLeftToRightCopy(FrameConvexPolygon2DReadOnly convexPolygon, FramePoint2DReadOnly observerFramePoint)
   {
      visibleVertices.clear();

      convexPolygon.checkReferenceFrameMatch(observerFramePoint);
      int lineOfSightStartIndex = convexPolygon.lineOfSightStartIndex(observerFramePoint);
      int lineOfSightEndIndex = convexPolygon.lineOfSightEndIndex(observerFramePoint);
      if (lineOfSightStartIndex == -1 || lineOfSightEndIndex == -1)
         return false;

      int index = lineOfSightEndIndex;

      while (true)
      {
         visibleVertices.add().setIncludingFrame(convexPolygon.getVertex(index));
         index = convexPolygon.getPreviousVertexIndex(index);
         if (index == lineOfSightStartIndex)
         {
            visibleVertices.add().setIncludingFrame(convexPolygon.getVertex(index));
            break;
         }
      }

      return true;
   }
}
