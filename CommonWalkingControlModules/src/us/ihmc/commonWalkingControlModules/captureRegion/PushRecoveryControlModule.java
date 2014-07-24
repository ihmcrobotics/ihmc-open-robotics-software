package us.ihmc.commonWalkingControlModules.captureRegion;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point2d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.ICPAndMomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.trajectories.StraightLinePositionTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.SwingTimeCalculationProvider;
import us.ihmc.plotting.shapes.PointArtifact;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameLine2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.LineSegment2d;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.trajectories.providers.DoubleProvider;
import us.ihmc.utilities.math.trajectories.providers.PositionProvider;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.plotting.YoFrameLine2dArtifact;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameLine2d;
import com.yobotics.simulationconstructionset.util.statemachines.StateMachine;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransitionCondition;

public class PushRecoveryControlModule
{
   private static final boolean ENABLE = false;
   private static final boolean ENABLE_DOUBLE_SUPPORT_PUSH_RECOVERY = false;
   private static final boolean USE_ICP_PROJECTION_PLANNER = false;
   private static final boolean USE_PUSH_RECOVERY_ICP_PLANNER = true;
   private static final boolean ENABLE_PROJECTION_INSIDE_PUSH_RECOVERY_ICP_PLANNER = true;

   private static final double MINIMUM_TIME_BEFORE_RECOVER_WITH_REDUCED_POLYGON = 2.0;
   private static final double DOUBLESUPPORT_SUPPORT_POLYGON_SCALE = 0.75;
   private static final double TRUST_TIME_SCALE = 0.95;
   private static final double MINIMUM_TIME_TO_REPLAN = 0.1;
   private static final double MINIMUM_SWING_TIME_FOR_DOUBLE_SUPPORT_RECOVERY = 0.3;
   private static final double MINIMUN_CAPTURE_REGION_PERCENTAGE_OF_FOOT_AREA = 2.0;
   private static final double REDUCE_SWING_TIME_MULTIPLIER = 0.9;  
   private static final double DISTANCE_BETWEEN_FOOT_CENTER_AND_SUPPORT_WITH_OPPOSITE_LINE = 0.01;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   private final BooleanYoVariable enablePushRecovery; 
   private final BooleanYoVariable useICPProjection;
   private final BooleanYoVariable usePushRecoveryICPPlanner;
   private final BooleanYoVariable enablePushRecoveryFromDoubleSupport;
   private final BooleanYoVariable enableProjectionInsidePushRecoveryICPPlanner;
   private final ICPAndMomentumBasedController icpAndMomentumBasedController;
   private final MomentumBasedController momentumBasedController;
   private final OrientationStateVisualizer orientationStateVisualizer;
   private final FootstepAdjustor footstepAdjustor;
   private final OneStepCaptureRegionCalculator captureRegionCalculator;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final BooleanYoVariable footstepWasProjectedInCaptureRegion;
   private final CommonWalkingReferenceFrames referenceFrames;
   private final StateMachine<?> stateMachine;
   private final SwingTimeCalculationProvider swingTimeCalculationProvider;
   private final PushRecoveryICPPlanner pushRecoveryICPPlanner;

   private boolean recoveringFromDoubleSupportFall;
   private boolean usingReducedSwingTime;
   private double reducedSwingTime, doubleSupportInitialSwingTime;
   private Footstep recoverFromDoubleSupportFallFootStep;
   private final BooleanYoVariable recovering;
   private final BooleanYoVariable tryingUncertainRecover;
   private final BooleanYoVariable existsAMinimumSwingTimeCaptureRegion;
   private BooleanYoVariable readyToGrabNextFootstep;
   private final DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry;
   private final SideDependentList<? extends ContactablePlaneBody> feet;
   private final List<FramePoint> tempContactPoints;
   private final FrameConvexPolygon2d tempFootPolygon;
   private final FramePoint2d initialDesiredICP, finalDesiredICP;

   private final YoFrameLine2d capturePointTrajectoryLine;
   private final YoFrameLine2dArtifact capturePointTrajectoryLineArtifact;
   private final PointArtifact projectedCapturePointArtifact;
   private final Point2d projectedCapturePoint;
   private final DoubleYoVariable swingTimeRemaining;
   private final DoubleYoVariable captureRegionAreaWithDoubleSupportMinimumSwingTime;

   public PushRecoveryControlModule(MomentumBasedController momentumBasedController, WalkingControllerParameters walkingControllerParameters,
         BooleanYoVariable readyToGrabNextFootstep, ICPAndMomentumBasedController icpAndMomentumBasedController, StateMachine<?> stateMachine,
         YoVariableRegistry parentRegistry, SwingTimeCalculationProvider swingTimeCalculationProvider, SideDependentList<? extends ContactablePlaneBody> feet)
   {
      this.momentumBasedController = momentumBasedController;
      this.readyToGrabNextFootstep = readyToGrabNextFootstep;
      this.icpAndMomentumBasedController = icpAndMomentumBasedController;
      this.referenceFrames = momentumBasedController.getReferenceFrames();
      this.stateMachine = stateMachine;
      this.swingTimeCalculationProvider = swingTimeCalculationProvider;
      this.feet = feet;
      this.pushRecoveryICPPlanner = new PushRecoveryICPPlanner("pushRecoveryICPPlanner", worldFrame, remainingSwingTimeProvider, initialICPPositionProvider, finalICPPositionProvider, registry);

      this.enablePushRecovery = new BooleanYoVariable("enablePushRecovery", registry);
      this.enablePushRecovery.set(ENABLE);
      this.enablePushRecoveryFromDoubleSupport = new BooleanYoVariable("enablePushRecoveryFromDoubleSupport", registry);
      this.enablePushRecoveryFromDoubleSupport.set(ENABLE_DOUBLE_SUPPORT_PUSH_RECOVERY);
      this.useICPProjection = new BooleanYoVariable("useICPProjection", registry);
      this.useICPProjection.set(USE_ICP_PROJECTION_PLANNER);
      this.usePushRecoveryICPPlanner = new BooleanYoVariable("usePushRecoveryICPPlanner", registry);
      this.usePushRecoveryICPPlanner.set(USE_PUSH_RECOVERY_ICP_PLANNER);  
      this.enableProjectionInsidePushRecoveryICPPlanner = new BooleanYoVariable("enableProjectionInsidePushRecoveryICPPlanner", registry);
      this.enableProjectionInsidePushRecoveryICPPlanner.set(ENABLE_PROJECTION_INSIDE_PUSH_RECOVERY_ICP_PLANNER);  
      
//      if (useICPProjection.getBooleanValue() && usePushRecoveryICPPlanner.getBooleanValue())
//      {
//         throw new RuntimeException("Select only one default ICP planner for push recovery.");
//      }
    
      this.usingReducedSwingTime = false;

      this.dynamicGraphicObjectsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();
      captureRegionCalculator = new OneStepCaptureRegionCalculator(referenceFrames, walkingControllerParameters, registry, dynamicGraphicObjectsListRegistry);
      footstepAdjustor = new FootstepAdjustor(registry, dynamicGraphicObjectsListRegistry);
      orientationStateVisualizer = new OrientationStateVisualizer(dynamicGraphicObjectsListRegistry, registry);

      footstepWasProjectedInCaptureRegion = new BooleanYoVariable("footstepWasProjectedInCaptureRegion", registry);
      recovering = new BooleanYoVariable("recovering", registry);
      tryingUncertainRecover = new BooleanYoVariable("tryingUncertainRecover", registry);
      existsAMinimumSwingTimeCaptureRegion = new BooleanYoVariable("existsAMinimumSwingTimeCaptureRegion", registry);
      initialDesiredICP = new FramePoint2d(worldFrame);
      finalDesiredICP = new FramePoint2d(worldFrame);

      capturePointTrajectoryLine = new YoFrameLine2d(getClass().getSimpleName(), "CapturePointTrajectoryLine", ReferenceFrame.getWorldFrame(), registry);
      projectedCapturePoint = new Point2d();
      projectedCapturePointArtifact = new PointArtifact("ProjectedCapturePointArtifact", projectedCapturePoint);
      tempContactPoints = new ArrayList<FramePoint>();
      tempFootPolygon = new FrameConvexPolygon2d(worldFrame);
      swingTimeRemaining = new DoubleYoVariable("pushRecoverySwingTimeRemaining", registry);
      captureRegionAreaWithDoubleSupportMinimumSwingTime = new DoubleYoVariable("captureRegionAreaWithMinimumSwingTime", registry);

      this.capturePointTrajectoryLineArtifact = new YoFrameLine2dArtifact("CapturePointTrajectoryLineArtifact", capturePointTrajectoryLine, Color.red);
      dynamicGraphicObjectsListRegistry.registerArtifact("CapturePointTrajectoryArtifact", capturePointTrajectoryLineArtifact);
      dynamicGraphicObjectsListRegistry.registerArtifact("ProjectedCapturePointArtifact", projectedCapturePointArtifact);

      parentRegistry.addChild(registry);
      
      reset();
   }


   public void reset()
   {
      footstepWasProjectedInCaptureRegion.set(false);
      recovering.set(false);
      recoverFromDoubleSupportFallFootStep = null;
      captureRegionCalculator.hideCaptureRegion();

      if (recoveringFromDoubleSupportFall)
      {
         if (usingReducedSwingTime)
         {
            this.swingTimeCalculationProvider.updateSwingTime();
            usingReducedSwingTime = false;
         }
         recoveringFromDoubleSupportFall = false;
         tryingUncertainRecover.set(false);
         existsAMinimumSwingTimeCaptureRegion.set(false);
      }
   }

   public class IsFallingFromDoubleSupportCondition implements StateTransitionCondition
   {
      private final FramePoint2d capturePoint2d = new FramePoint2d();
      private RobotSide swingSide = null;
      private RobotSide transferToSide = null;

      private final Transform3D fromWorldToPelvis = new Transform3D();
      private final Transform3D scaleTransformation = new Transform3D();
      private final FrameConvexPolygon2d reducedSupportPolygon;
      private final ReferenceFrame midFeetZUp;
      private double regularSwingTime;
      private LineSegment2d closestEdge; 
      private Point2d closestPointOnEdge;
      private double capturePointDistanceFromLeftFoot, capturePointDistanceFromRightFoot;
      private ReferenceFrame footFrame;
      private FramePoint projectedCapturePoint;
      private FramePoint2d projectedCapturePoint2d;
      private Footstep currentFootstep = null;
      private FrameConvexPolygon2d footPolygon;
      private boolean isICPOutside, leftFootSupportWithOpposite, rightFootSupportWithOpposite;

      public IsFallingFromDoubleSupportCondition(RobotSide robotSide)
      {
         this.transferToSide = robotSide;
         this.scaleTransformation.setScale(DOUBLESUPPORT_SUPPORT_POLYGON_SCALE);
         this.reducedSupportPolygon = new FrameConvexPolygon2d(icpAndMomentumBasedController.getBipedSupportPolygons().getSupportPolygonInMidFeetZUp());
         this.projectedCapturePoint = new FramePoint(worldFrame, 0.0, 0.0, 0.0);
         this.projectedCapturePoint2d = new FramePoint2d(worldFrame, 0.0, 0.0);
         this.footPolygon = new FrameConvexPolygon2d(worldFrame);
         midFeetZUp = referenceFrames.getMidFeetZUpFrame();
      }

      @Override
      public boolean checkCondition()
      {
         if (enablePushRecovery.getBooleanValue())
         {
            if (isEnabledInDoubleSupport())
            {
               FrameConvexPolygon2d supportPolygonInMidFeetZUp = icpAndMomentumBasedController.getBipedSupportPolygons().getSupportPolygonInMidFeetZUp();

               // get current robot status
               reducedSupportPolygon.changeFrame(midFeetZUp);
               reducedSupportPolygon.setAndUpdate(supportPolygonInMidFeetZUp);
               icpAndMomentumBasedController.getCapturePoint().getFrameTuple2dIncludingFrame(capturePoint2d);

               capturePoint2d.changeFrame(midFeetZUp);
               reducedSupportPolygon.applyTransform(scaleTransformation);

               // update the visualization
               momentumBasedController.getFullRobotModel().getPelvis().getBodyFixedFrame().getTransformToDesiredFrame(fromWorldToPelvis, worldFrame);
               orientationStateVisualizer.updatePelvisReferenceFrame(fromWorldToPelvis);
               orientationStateVisualizer.updateReducedSupportPolygon(reducedSupportPolygon);

               if (stateMachine.timeInCurrentState() < MINIMUM_TIME_BEFORE_RECOVER_WITH_REDUCED_POLYGON)
               {
                  isICPOutside = !supportPolygonInMidFeetZUp.isPointInside(capturePoint2d);
               }
               else
               {
                  isICPOutside = !reducedSupportPolygon.isPointInside(capturePoint2d);
               }

               if (isICPOutside && recoverFromDoubleSupportFallFootStep == null)
               {
//                  System.out.println("Robot is falling from double support");
                  projectedCapturePoint.changeFrame(capturePoint2d.getReferenceFrame());
                  projectedCapturePoint.set(capturePoint2d.getX(), capturePoint2d.getY(), 0.0);

                  for (RobotSide side : RobotSide.values())
                  {
                     footFrame = momentumBasedController.getFullRobotModel().getSoleFrame(side);
                     projectedCapturePoint.changeFrame(footFrame);
                     currentFootstep = createFootstepAtCurrentLocation(side);
                     calculateTouchdownFootPolygon(currentFootstep, projectedCapturePoint.getReferenceFrame(), footPolygon);
                     projectedCapturePoint2d.setIncludingFrame(projectedCapturePoint.getReferenceFrame(), projectedCapturePoint.getX(), projectedCapturePoint.getY());

                     // In the following the reference frames must be equal
                     if (side == RobotSide.LEFT)
                     {
                        leftFootSupportWithOpposite = projectedCapturePoint.getY() + DISTANCE_BETWEEN_FOOT_CENTER_AND_SUPPORT_WITH_OPPOSITE_LINE > 0;
                        closestEdge = footPolygon.getClosestEdge(projectedCapturePoint2d).getLineSegment2d();
                        closestPointOnEdge = closestEdge.getClosestPointOnLineSegment(projectedCapturePoint2d.getPoint());
                        capturePointDistanceFromLeftFoot = closestPointOnEdge.distance(projectedCapturePoint2d.getPoint()); 
                     }
                     else
                     {
                        rightFootSupportWithOpposite = projectedCapturePoint.getY() - DISTANCE_BETWEEN_FOOT_CENTER_AND_SUPPORT_WITH_OPPOSITE_LINE < 0;
                        closestEdge = footPolygon.getClosestEdge(projectedCapturePoint2d).getLineSegment2d();
                        closestPointOnEdge = closestEdge.getClosestPointOnLineSegment(projectedCapturePoint2d.getPoint());
                        capturePointDistanceFromRightFoot = closestPointOnEdge.distance(projectedCapturePoint2d.getPoint());
                     }
                  }
                  
                  if (capturePointDistanceFromLeftFoot > capturePointDistanceFromRightFoot)
                  {
                     if(rightFootSupportWithOpposite)
                     {
                        swingSide = RobotSide.RIGHT;
                     }
                     else
                     {
                        swingSide = RobotSide.LEFT;
                     }
                  }                  
                  else
                  {
                     if (leftFootSupportWithOpposite)
                     {
                        swingSide = RobotSide.LEFT;
                     }
                     else
                     {
                        swingSide = RobotSide.RIGHT;
                     }
                  }

                  if (transferToSide == swingSide)
                  {
                     return false;
                  }

                  currentFootstep = createFootstepAtCurrentLocation(swingSide);
                  capturePoint2d.changeFrame(worldFrame);
                  
                  regularSwingTime = swingTimeCalculationProvider.getValue();
                  
                  doubleSupportInitialSwingTime = computeInitialSwingTimeForDoubleSupportRecovery(swingSide, regularSwingTime, capturePoint2d);
                  
                  if(Double.isNaN(captureRegionAreaWithDoubleSupportMinimumSwingTime.getDoubleValue()))
                  {
                     if (existsAMinimumSwingTimeCaptureRegion.getBooleanValue())
                     {              
                        // we try anyway with the capture region calculated using the minimum swing time, but the real swing time is the minimum for double support. 
                        // TODO N-step recover
                        swingTimeCalculationProvider.setSwingTime(MINIMUM_SWING_TIME_FOR_DOUBLE_SUPPORT_RECOVERY);
                        usingReducedSwingTime = true;
                        readyToGrabNextFootstep.set(false);
                        momentumBasedController.getUpcomingSupportLeg().set(transferToSide.getOppositeSide());
                        
//                        // we orient the foot towards the external to exploit the length of the foot, needs to be improved for backward pushes
//                        Transform3D footPose = new Transform3D(); 
//                        currentFootstep.getPose(footPose);
//                        footPose.rotZ(swingSide.negateIfRightSide(DEFAULT_FOOT_ROTATION_TO_RECOVER));
//                        FramePose poseToPack = new FramePose();
//                        currentFootstep.getPose(poseToPack);
//                        poseToPack.setPose(footPose);
//                        currentFootstep.setPose(poseToPack);
   
                        recoverFromDoubleSupportFallFootStep = currentFootstep;
                        recoveringFromDoubleSupportFall = true;
                        reducedSwingTime = MINIMUM_SWING_TIME_FOR_DOUBLE_SUPPORT_RECOVERY;
                        tryingUncertainRecover.set(true);           
                        return true;
                     }
                     
                     // TODO prepare to fall
                     return false;
                  }

                  if (doubleSupportInitialSwingTime < MINIMUM_SWING_TIME_FOR_DOUBLE_SUPPORT_RECOVERY)
                  {
                     // here we try to take a step with the double support minimum swing time 
                     swingTimeCalculationProvider.setSwingTime(MINIMUM_SWING_TIME_FOR_DOUBLE_SUPPORT_RECOVERY);
                     usingReducedSwingTime = true;
                     readyToGrabNextFootstep.set(false);
                     momentumBasedController.getUpcomingSupportLeg().set(transferToSide.getOppositeSide());
                     recoverFromDoubleSupportFallFootStep = currentFootstep;
                     recoveringFromDoubleSupportFall = true;
                     reducedSwingTime = MINIMUM_SWING_TIME_FOR_DOUBLE_SUPPORT_RECOVERY;
                     return true;
                  }
                  
                  if(doubleSupportInitialSwingTime <= regularSwingTime)
                  {
                     swingTimeCalculationProvider.setSwingTime(doubleSupportInitialSwingTime);
                     usingReducedSwingTime = true;
                     readyToGrabNextFootstep.set(false);
                     momentumBasedController.getUpcomingSupportLeg().set(transferToSide.getOppositeSide());
                     recoverFromDoubleSupportFallFootStep = currentFootstep;
                     recoveringFromDoubleSupportFall = true;
                     reducedSwingTime = doubleSupportInitialSwingTime;
                  }
                  
                  return true;
               }

               // we need this to reset the reference frame 
               reducedSupportPolygon.changeFrame(worldFrame);
               return false;
            }
         }

         return false;
      }
   }
   
   public class PushRecoveryICPPlanner extends StraightLinePositionTrajectoryGenerator
   {
      private FramePoint tempPositionToPack;
      private FrameVector tempVelocityToPack;
      
      public PushRecoveryICPPlanner(String namePrefix, ReferenceFrame referenceFrame, DoubleProvider trajectoryTimeProvider,
            PositionProvider initialPositionProvider, PositionProvider finalPositionProvider, YoVariableRegistry parentRegistry)
      {
         super(namePrefix, referenceFrame, trajectoryTimeProvider, initialPositionProvider, finalPositionProvider, parentRegistry);
         tempPositionToPack = new FramePoint(worldFrame);
         tempVelocityToPack = new FrameVector(worldFrame);
      }  
      
      public void getICPPosition(FramePoint2d desiredICPPositionToPack, FramePoint2d currentICP)
      {
         if(enableProjectionInsidePushRecoveryICPPlanner.getBooleanValue())
         {
            desiredICPPositionToPack.set(capturePointTrajectoryLine.orthogonalProjectionCopy(currentICP));
         }
         else
         {
            get(tempPositionToPack);
            tempPositionToPack.changeFrame(desiredICPPositionToPack.getReferenceFrame());
            desiredICPPositionToPack.set(tempPositionToPack.getX(), tempPositionToPack.getY());         
         }
      }
      
      public void getICPVelocity(FrameVector2d desiredICPVelocityToPack)
      {
         if(enableProjectionInsidePushRecoveryICPPlanner.getBooleanValue())
         {
            desiredICPVelocityToPack.setToZero(worldFrame);
         }
         else
         {
            packVelocity(tempVelocityToPack);
            tempVelocityToPack.changeFrame(desiredICPVelocityToPack.getReferenceFrame());
            desiredICPVelocityToPack.set(tempVelocityToPack.getX(), tempVelocityToPack.getY());
         }
      }
   }

   public boolean checkAndUpdateFootstep(RobotSide swingSide, double swingTimeRemaining, FramePoint2d capturePoint2d, Footstep nextFootstep, double omega0,
         FrameConvexPolygon2d footPolygon)
   {
      this.swingTimeRemaining.set(swingTimeRemaining);
      
      if (enablePushRecovery.getBooleanValue())
      {
         if (tryingUncertainRecover.getBooleanValue())
         {
            captureRegionCalculator.calculateCaptureRegion(swingSide, MINIMUM_TIME_TO_REPLAN, capturePoint2d, omega0, footPolygon);
         }
         else
         {
            captureRegionCalculator.calculateCaptureRegion(swingSide, swingTimeRemaining, capturePoint2d, omega0, footPolygon);
         }
         
         if (swingTimeRemaining < MINIMUM_TIME_TO_REPLAN)
         {
            // do not re-plan if we are almost at touch-down
            return false;
         }

         footstepWasProjectedInCaptureRegion.set(footstepAdjustor.adjustFootstep(nextFootstep, footPolygon.getCentroid(),
               captureRegionCalculator.getCaptureRegion(), isRecoveringFromDoubleSupportFall()));

         if (footstepWasProjectedInCaptureRegion.getBooleanValue())
         {
            recovering.set(true);
         }
         
         return footstepWasProjectedInCaptureRegion.getBooleanValue();
      }
      return false;
   }
   
   private void calculateTouchdownFootPolygon(Footstep footstep, ReferenceFrame desiredFrame, FrameConvexPolygon2d polygonToPack)
   {
      List<FramePoint> expectedContactPoints = footstep.getExpectedContactPoints();
      int numberOfVertices = expectedContactPoints.size();
      for (int i=0; i<numberOfVertices; i++)
      {
         expectedContactPoints.get(i).changeFrame(desiredFrame);
      }
      polygonToPack.setIncludingFrameByProjectionOntoXYPlaneAndUpdate(desiredFrame, expectedContactPoints);
   }
   
   private double computeMinimumSwingTime(RobotSide swingSide, double swingTimeRemaining, FramePoint2d capturePoint2d, double omega0, FrameConvexPolygon2d footPolygon)
   {
      double reducedSwingTime = swingTimeRemaining;
      captureRegionCalculator.calculateCaptureRegion(swingSide, swingTimeRemaining, capturePoint2d, omega0, footPolygon);
      
      // If the capture region is too small we reduce the swing time. 
      while (captureRegionCalculator.getCaptureRegionArea() < PushRecoveryControlModule.MINIMUN_CAPTURE_REGION_PERCENTAGE_OF_FOOT_AREA * footPolygon.getArea() || 
                  Double.isNaN(captureRegionCalculator.getCaptureRegionArea()))
      {
         reducedSwingTime = reducedSwingTime * REDUCE_SWING_TIME_MULTIPLIER;
         captureRegionCalculator.calculateCaptureRegion(swingSide, reducedSwingTime, capturePoint2d, omega0, footPolygon);

         // avoid infinite loops
         if (reducedSwingTime < MINIMUM_TIME_TO_REPLAN)
         {
            if(!Double.isNaN(captureRegionCalculator.getCaptureRegionArea()))
            {
               existsAMinimumSwingTimeCaptureRegion.set(true);
            }
            break;
         }
      }
      
      return reducedSwingTime;
      
   }
   
   private double computeInitialSwingTimeForDoubleSupportRecovery(RobotSide swingSide, double swingTimeRemaining, FramePoint2d capturePoint2d)
   {
      momentumBasedController.getContactPoints(feet.get(swingSide.getOppositeSide()), tempContactPoints);
      tempFootPolygon.setIncludingFrameByProjectionOntoXYPlaneAndUpdate(momentumBasedController.getReferenceFrames().getAnkleZUpFrame(swingSide.getOppositeSide()), tempContactPoints);
      
      captureRegionCalculator.calculateCaptureRegion(swingSide, MINIMUM_SWING_TIME_FOR_DOUBLE_SUPPORT_RECOVERY, capturePoint2d, icpAndMomentumBasedController.getOmega0(), tempFootPolygon);
      captureRegionAreaWithDoubleSupportMinimumSwingTime.set(captureRegionCalculator.getCaptureRegionArea());
      
      return computeMinimumSwingTime(swingSide, swingTimeRemaining, capturePoint2d, icpAndMomentumBasedController.getOmega0(), tempFootPolygon);
   }

   public double computeTimeToProjectDesiredICPToClosestPointOnTrajectoryToActualICP(FramePoint2d capturePoint2d, FramePoint2d constantCenterOfPressure,
         FramePoint2d initialICP, FramePoint2d finalDesiredICP, double omega0)
   {
      FrameLine2d capturePointTrajectoryLine = new FrameLine2d(finalDesiredICP, initialICP);
      this.capturePointTrajectoryLine.setFrameLine2d(capturePointTrajectoryLine);

      //project current capture point to desired capture point trajectory line
      capturePointTrajectoryLine.orthogonalProjection(capturePoint2d);

      projectedCapturePoint.set(capturePoint2d.getPointCopy());

      return computeOffsetTimeToMoveDesiredICPToTrajectoryLine(capturePoint2d, constantCenterOfPressure, initialICP, finalDesiredICP, omega0);
   }
   
   private Footstep createFootstepAtCurrentLocation(RobotSide robotSide)
   {
      ContactablePlaneBody foot = feet.get(robotSide);
      ReferenceFrame footReferenceFrame = foot.getRigidBody().getParentJoint().getFrameAfterJoint();
      FramePose framePose = new FramePose(footReferenceFrame);
      framePose.changeFrame(worldFrame);

      PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("poseReferenceFrame", framePose);

      boolean trustHeight = true;
      Footstep footstep = new Footstep(foot, poseReferenceFrame, trustHeight);

      return footstep;
   }

   private double computeOffsetTimeToMoveDesiredICPToTrajectoryLine(FramePoint2d capturePoint, FramePoint2d constantCenterOfPressure, FramePoint2d initialICP,
         FramePoint2d finalDesiredICP, double omega0)
   {
      // Given a desired capture point that is on the capture point trajectory, an initial capture point, a constant center of pressure, and omega, this 
      // method computes a time offset that will move the desired capture point to a desired point on that trajectory given by the argument capturePoint.
      FramePoint2d tmpNumerator = capturePoint;
      tmpNumerator.sub(constantCenterOfPressure);

      FramePoint2d tmpDenominator = initialICP;
      tmpDenominator.sub(constantCenterOfPressure);

      return (1 / omega0) * Math.log(Math.abs(tmpNumerator.getX()) / Math.abs(tmpDenominator.getX()));
   }

   private PositionProvider initialICPPositionProvider = new PositionProvider()
   {
      @Override
      public void get(FramePoint positionToPack)
      {
         initialDesiredICP.checkReferenceFrameMatch(worldFrame);
         positionToPack.setIncludingFrame(worldFrame, initialDesiredICP.getX(), initialDesiredICP.getY(), 0.0);
      }  
   };
   
   private PositionProvider finalICPPositionProvider = new PositionProvider()
   {
      @Override
      public void get(FramePoint positionToPack)
      {
         finalDesiredICP.checkReferenceFrameMatch(worldFrame);
         positionToPack.setIncludingFrame(worldFrame, finalDesiredICP.getX(), finalDesiredICP.getY(), 0.0);         
      }
   };
   
   private DoubleProvider remainingSwingTimeProvider = new DoubleProvider()
   {
      @Override
      public double getValue()
      {
         return getSwingTimeRemaining();
      } 
   };

   public Footstep getRecoverFromDoubleSupportFootStep()
   {
      return recoverFromDoubleSupportFallFootStep;
   }

   public double getTrustTimeToConsiderSwingFinished()
   {
      return this.reducedSwingTime * TRUST_TIME_SCALE;
   }
   
   public double getSwingTimeRemaining()
   {
      return this.swingTimeRemaining.getDoubleValue();
   }
   
   public PushRecoveryICPPlanner getICPPlanner()
   {
      return pushRecoveryICPPlanner;
   }
   
   public boolean isEnabled()
   {
      return enablePushRecovery.getBooleanValue();
   }

   public boolean isEnabledInDoubleSupport()
   {
      return enablePushRecoveryFromDoubleSupport.getBooleanValue();
   }
   
   public boolean isRecoveringFromDoubleSupportFall()
   {
      return recoveringFromDoubleSupportFall;
   }
   
   public boolean isPerformingUncertainRecover()
   {
      return tryingUncertainRecover.getBooleanValue();
   }
   
   public void setSwingTimeRemaining(double value)
   {
      this.swingTimeRemaining.set(value);
   }

   public void setRecoveringFromDoubleSupportState(boolean value)
   {
      recoveringFromDoubleSupportFall = value;
   }

   public void setRecoverFromDoubleSupportFootStep(Footstep recoverFootStep)
   {
      recoverFromDoubleSupportFallFootStep = recoverFootStep;
   }
   
   public void setFinalDesiredICP(FramePoint2d tempPoint)
   {
      tempPoint.changeFrame(worldFrame);
      finalDesiredICP.setIncludingFrame(worldFrame, tempPoint.getX(), tempPoint.getY());
   }
   
   public void setInitialDesiredICP(FramePoint2d tempPoint)
   {
      tempPoint.changeFrame(worldFrame);
      initialDesiredICP.setIncludingFrame(worldFrame, tempPoint.getX(), tempPoint.getY());
   }

   public boolean useICPProjection()
   {
      return this.useICPProjection.getBooleanValue();
   }
   
   public boolean usePushRecoveryICPPlanner()
   {
      return this.usePushRecoveryICPPlanner.getBooleanValue();
   }

   public boolean isRecovering()
   {
      return recovering.getBooleanValue();
   }
}
