package us.ihmc.commonWalkingControlModules.captureRegion;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.time.GlobalTimer;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class OneStepCaptureRegionCalculator
{
   private static final boolean VISUALIZE = true;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int MAX_CAPTURE_REGION_POLYGON_POINTS = 26;
   private static final int KINEMATIC_LIMIT_POINTS = 10;
   private double reachableRegionCutoffAngle = 0.80;
   
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final GlobalTimer globalTimer = new GlobalTimer(name + "Timer", registry);
   
   private CaptureRegionVisualizer captureRegionVisualizer = null;
   private FrameConvexPolygon2d captureRegionPolygon = new FrameConvexPolygon2d(worldFrame);
   
   // necessary variables for the reachable region calculation:
   private final double midFootAnkleXOffset;
   private final double footWidth;
   // necessary variables for the capture region calculation:
   private SideDependentList<FrameConvexPolygon2d> reachableRegions;

   private final SideDependentList<ReferenceFrame> ankleZUpFrames;
   private final double kineamaticStepRange;
   
   public OneStepCaptureRegionCalculator(CommonWalkingReferenceFrames referenceFrames,
                                         WalkingControllerParameters walkingControllerParameters,
                                         YoVariableRegistry parentRegistry,
                                         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      this(walkingControllerParameters.getFootForwardOffset() - walkingControllerParameters.getFootLength()/2.0,
           walkingControllerParameters.getFootWidth(),
           walkingControllerParameters.getMaxStepLength(),
           referenceFrames.getAnkleZUpReferenceFrames(),
           parentRegistry,
           dynamicGraphicObjectsListRegistry);
   }
   
   public OneStepCaptureRegionCalculator(double midFootAnkleXOffset,
                                         double footWidth,
                                         double kineamaticStepRange,
                                         SideDependentList<ReferenceFrame> ankleZUpFrames,
                                         YoVariableRegistry parentRegistry,
                                         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      this.kineamaticStepRange = kineamaticStepRange;
      this.ankleZUpFrames = ankleZUpFrames;
      this.midFootAnkleXOffset = midFootAnkleXOffset;
      this.footWidth = footWidth;
      
      calculateReachableRegions();
      
      // set up registry and visualizer
      parentRegistry.addChild(registry);
      if(dynamicGraphicObjectsListRegistry != null && VISUALIZE) 
      {
         captureRegionVisualizer = new CaptureRegionVisualizer(this, dynamicGraphicObjectsListRegistry, registry);
      }
   }
   
   private void calculateReachableRegions()
   {
      ArrayList<FramePoint2d> reachableRegionPoints = new ArrayList<FramePoint2d>(MAX_CAPTURE_REGION_POLYGON_POINTS - 2);
      reachableRegions = new SideDependentList<FrameConvexPolygon2d>();
      for(RobotSide side : RobotSide.values)
      {
         reachableRegionPoints.clear();
         double sign = side == RobotSide.RIGHT ? 1.0 : -1.0;
         for (int i = 0; i < MAX_CAPTURE_REGION_POLYGON_POINTS - 1; i++)
         {
            double angle = sign * reachableRegionCutoffAngle * Math.PI * ((double)i) / 
                  ((double)(MAX_CAPTURE_REGION_POLYGON_POINTS - 2));
            double x = kineamaticStepRange * Math.cos(angle) + midFootAnkleXOffset;
            double y = kineamaticStepRange * Math.sin(angle);
//            if(Math.abs(y) < footWidth/2.0)
//               y = sign*footWidth/2.0;
            reachableRegionPoints.add(new FramePoint2d(ankleZUpFrames.get(side), x, y));
         }
         reachableRegionPoints.add(new FramePoint2d(ankleZUpFrames.get(side), midFootAnkleXOffset, sign * footWidth / 2.0));
         FrameConvexPolygon2d reachableRegion = new FrameConvexPolygon2d(reachableRegionPoints);
         reachableRegions.set(side, reachableRegion);
      }
   }
   
   // variables for the capture region calculation
   // set them up here to avoid crating new ones every every tick
   private final FramePoint2d footCentroid = new FramePoint2d(worldFrame);
   private final FramePoint2d predictedICP = new FramePoint2d(worldFrame);
   private final FrameVector2d projectedLine = new FrameVector2d(worldFrame);
   private final FrameVector2d firstKinematicExtremeDirection = new FrameVector2d(worldFrame);
   private final FrameVector2d lastKinematicExtremeDirection = new FrameVector2d(worldFrame);
   private final ArrayList<FramePoint> captureRegionPoints = new ArrayList<FramePoint>();
   private FramePoint2d capturePoint = new FramePoint2d(worldFrame);
   private FrameConvexPolygon2d supportFootPolygon = new FrameConvexPolygon2d(worldFrame);
   private FrameConvexPolygon2d captureRegion = new FrameConvexPolygon2d(worldFrame);
   
   public void calculateCaptureRegion(RobotSide swingSide,
                                      double swingTimeRemaining,
                                      FramePoint2d icp,
                                      double omega0,
                                      FrameConvexPolygon2d footPolygon)
   {
      globalTimer.startTimer();
      // 1. Set up all needed variables and reference frames for the calculation:
      ReferenceFrame supportAnkleZUp = ankleZUpFrames.get(swingSide.getOppositeSide());
      capturePoint = icp.changeFrameCopy(supportAnkleZUp);
      footCentroid.changeFrame(supportAnkleZUp);
      firstKinematicExtremeDirection.changeFrame(supportAnkleZUp);
      lastKinematicExtremeDirection.changeFrame(supportAnkleZUp);
      predictedICP.changeFrame(supportAnkleZUp);
      projectedLine.changeFrame(supportAnkleZUp);
      supportFootPolygon = footPolygon.changeFrameCopy(supportAnkleZUp);
      
      swingTimeRemaining = MathTools.clipToMinMax(swingTimeRemaining, 0.0, Double.POSITIVE_INFINITY);
      supportFootPolygon.getCentroid(footCentroid);
      captureRegionPoints.clear();
      
      // 2. Get extreme CoP positions
      ArrayList<FramePoint2d> extremesOfFeasibleCOP =
            supportFootPolygon.getAllVisibleVerticesFromOutsideLeftToRight(capturePoint);
      if (extremesOfFeasibleCOP == null)
      {
         // If the ICP is in the support polygon return the whole reachable region.
         globalTimer.stopTimer();
         captureRegionPolygon = reachableRegions.get(swingSide.getOppositeSide());
         updateVisualizer();
         return;
      }
      
      // 3. For every possible extreme CoP predict the corresponding ICP give the remaining swing time
      for (int i = 0; i < extremesOfFeasibleCOP.size(); i++)
      {
         FramePoint2d copExtreme = extremesOfFeasibleCOP.get(i);
         copExtreme.changeFrame(supportAnkleZUp);
         CaptureRegionMath.predictCapturePoint(capturePoint, copExtreme, swingTimeRemaining, omega0, predictedICP);
         captureRegionPoints.add(predictedICP.toFramePoint());
         
      // 4. Project the predicted ICP on a circle around the foot with the radius of the step range.
         projectedLine.set(predictedICP);
         projectedLine.sub(copExtreme);
         FramePoint2d kinematicExtreme =
               CaptureRegionMath.solveIntersectionOfRayAndCircle(footCentroid, predictedICP,
                                                                 projectedLine, kineamaticStepRange);

         if (kinematicExtreme == null)
         {
            globalTimer.stopTimer();
            captureRegionPolygon = null;
            return;
         }
         captureRegionPoints.add(kinematicExtreme.toFramePoint());
         
         if(i == 0)
         {
            firstKinematicExtremeDirection.set(kinematicExtreme);
            firstKinematicExtremeDirection.sub(footCentroid);
         } else {
            lastKinematicExtremeDirection.set(kinematicExtreme);
            lastKinematicExtremeDirection.sub(footCentroid);
         }
      }
      
      // 5. Add additional points to the capture region polygon on the circle between the kinematic extreme points
      for (int i = 0; i < KINEMATIC_LIMIT_POINTS - 1; i++)
      {
         double alphaFromAToB = ((double) (i + 1)) / ((double) (KINEMATIC_LIMIT_POINTS + 1));
         FramePoint2d additionalKinematicPoint =
               CaptureRegionMath.getPointBetweenVectorsAtDistanceFromOriginCircular(firstKinematicExtremeDirection,
                                                                  lastKinematicExtremeDirection,
                                                                  alphaFromAToB,
                                                                  kineamaticStepRange,
                                                                  footCentroid);
         captureRegionPoints.add(additionalKinematicPoint.toFramePoint());
      }
      
      // 6. Intersect the capture region with the reachable region
      if(captureRegion != null)
      {
         // This causes the capture region to always be null if it is null once.
         // This assumes that once there is no capture region the robot will fall for sure.
         captureRegion.updateByProjectionOntoXYPlane(captureRegionPoints, supportAnkleZUp);
         captureRegion = captureRegion.intersectionWith(reachableRegions.get(swingSide.getOppositeSide()));
      }
      
      globalTimer.stopTimer();
      captureRegionPolygon = captureRegion;
      updateVisualizer();
   }
   
   private void updateVisualizer()
   {
      if(captureRegionVisualizer != null && captureRegionPolygon != null)
      {
         captureRegionVisualizer.update();
      } else {
         hideCaptureRegion();
      }
   }
   
   public void hideCaptureRegion()
   {
      if(captureRegionVisualizer != null)
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
   
   public void setReachableRegions(SideDependentList<FrameConvexPolygon2d> reachableRegions)
   {
      this.reachableRegions = reachableRegions;
   }
}
