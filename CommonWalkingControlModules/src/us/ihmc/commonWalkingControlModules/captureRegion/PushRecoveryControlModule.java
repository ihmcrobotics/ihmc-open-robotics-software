package us.ihmc.commonWalkingControlModules.captureRegion;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.ICPAndMomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.trajectories.SwingTimeCalculationProvider;
import us.ihmc.plotting.shapes.PointArtifact;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.humanoidRobot.frames.CommonHumanoidReferenceFrames;
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
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.math.geometry.Transform3d;
import us.ihmc.utilities.math.trajectories.providers.DoubleProvider;
import us.ihmc.utilities.math.trajectories.providers.PositionProvider;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.plotting.YoArtifactLine2d;
import us.ihmc.yoUtilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.yoUtilities.humanoidRobot.footstep.Footstep;
import us.ihmc.yoUtilities.humanoidRobot.footstep.FootstepUtils;
import us.ihmc.yoUtilities.math.frames.YoFrameLine2d;
import us.ihmc.yoUtilities.math.trajectories.StraightLinePositionTrajectoryGenerator;
import us.ihmc.yoUtilities.stateMachines.StateMachine;
import us.ihmc.yoUtilities.stateMachines.StateTransitionCondition;


public class PushRecoveryControlModule
{
   private static final boolean ENABLE = false;
   private static final boolean ENABLE_DOUBLE_SUPPORT_PUSH_RECOVERY = false;
   private static final boolean USE_PUSH_RECOVERY_ICP_PLANNER = true;
   private static final boolean ENABLE_PROJECTION_INSIDE_PUSH_RECOVERY_ICP_PLANNER = true;

   private static final double MINIMUM_TIME_BEFORE_RECOVER_WITH_REDUCED_POLYGON = 3.0;
   private static final double DOUBLESUPPORT_SUPPORT_POLYGON_SCALE = 0.75;
   private static final double TRUST_TIME_SCALE = 0.95;
   private static final double MINIMUM_TIME_TO_REPLAN = 0.1;
   private static final double MINIMUM_SWING_TIME_FOR_DOUBLE_SUPPORT_RECOVERY = 0.3;
   private static final double MINIMUN_CAPTURE_REGION_PERCENTAGE_OF_FOOT_AREA = 2.0;
   private static final double REDUCE_SWING_TIME_MULTIPLIER = 0.9;
   private static final double DISTANCE_BETWEEN_FOOT_CENTER_AND_SUPPORT_WITH_OPPOSITE_LINE = 0.01;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanYoVariable enablePushRecovery;
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
   private final CommonHumanoidReferenceFrames referenceFrames;
   private final StateMachine<?> stateMachine;
   private final SwingTimeCalculationProvider swingTimeCalculationProvider;
   private final PushRecoveryICPPlanner pushRecoveryICPPlanner;

   private boolean recoveringFromDoubleSupportFall;
   private boolean usingReducedSwingTime;
   private double reducedSwingTime, doubleSupportInitialSwingTime;
   private final BooleanYoVariable recovering;
   private final BooleanYoVariable tryingUncertainRecover;
   private final BooleanYoVariable existsAMinimumSwingTimeCaptureRegion;
   private final BooleanYoVariable readyToGrabNextFootstep;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;
   private final SideDependentList<? extends ContactablePlaneBody> feet;
   private final List<FramePoint> tempContactPoints;
   private final FrameConvexPolygon2d tempFootPolygon;
   private final FramePoint2d initialDesiredICP, finalDesiredICP;

   private final YoFrameLine2d capturePointTrajectoryLine;
   private final YoArtifactLine2d capturePointTrajectoryLineArtifact;
   private final PointArtifact projectedCapturePointArtifact;
   private final Point2d projectedCapturePoint;
   private final DoubleYoVariable swingTimeRemaining;
   private final DoubleYoVariable captureRegionAreaWithDoubleSupportMinimumSwingTime;
   
   private Footstep recoverFromDoubleSupportFallFootStep;
   private RobotSide swingSideFromHighLevel;
   private Footstep nextFootstepFromHighLevel;

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
      this.pushRecoveryICPPlanner = new PushRecoveryICPPlanner("pushRecoveryICPPlanner", worldFrame, remainingSwingTimeProvider, initialICPPositionProvider,
            finalICPPositionProvider, registry);

      this.enablePushRecovery = new BooleanYoVariable("enablePushRecovery", registry);
      this.enablePushRecovery.set(ENABLE);
      this.enablePushRecoveryFromDoubleSupport = new BooleanYoVariable("enablePushRecoveryFromDoubleSupport", registry);
      this.enablePushRecoveryFromDoubleSupport.set(ENABLE_DOUBLE_SUPPORT_PUSH_RECOVERY);
      this.usePushRecoveryICPPlanner = new BooleanYoVariable("usePushRecoveryICPPlanner", registry);
      this.usePushRecoveryICPPlanner.set(USE_PUSH_RECOVERY_ICP_PLANNER);
      this.enableProjectionInsidePushRecoveryICPPlanner = new BooleanYoVariable("enableProjectionInsidePushRecoveryICPPlanner", registry);
      this.enableProjectionInsidePushRecoveryICPPlanner.set(ENABLE_PROJECTION_INSIDE_PUSH_RECOVERY_ICP_PLANNER);
      this.usingReducedSwingTime = false;
      this.yoGraphicsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();
      captureRegionCalculator = new OneStepCaptureRegionCalculator(referenceFrames, walkingControllerParameters, registry, yoGraphicsListRegistry);
      footstepAdjustor = new FootstepAdjustor(registry, yoGraphicsListRegistry);
      orientationStateVisualizer = new OrientationStateVisualizer(yoGraphicsListRegistry, registry);

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

      this.capturePointTrajectoryLineArtifact = new YoArtifactLine2d("CapturePointTrajectoryLineArtifact", capturePointTrajectoryLine, Color.red);
      yoGraphicsListRegistry.registerArtifact("CapturePointTrajectoryArtifact", capturePointTrajectoryLineArtifact);
      yoGraphicsListRegistry.registerArtifact("ProjectedCapturePointArtifact", projectedCapturePointArtifact);

      parentRegistry.addChild(registry);

      reset();
   }

   public class IsFallingFromDoubleSupportCondition implements StateTransitionCondition
   {
      private final FramePoint2d capturePoint2d = new FramePoint2d();
      private RobotSide swingSide = null;
      private RobotSide transferToSide = null;

      private final RigidBodyTransform fromWorldToPelvis = new RigidBodyTransform();
      private final Transform3d scaleTransformation = new Transform3d();
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

      /**
       * To check if the robot is falling from double support we check if the ICP is outside of a polygon which is the current support polygon (reduced in order to have a safe margin).
       * Subsequently the supporting leg is selected having a look to the distance of the ICP from each edge of the feet, the closest foot to the icp is used as supporting, while the opposite side is used to swing.
       * This because we know that moving the CoP close to the ICP decreases the ICP velocity.
       * If left foot has been selected, we check if the ICP is to too far on the left side (like ICP is outside the reduced support polygon, but since is too on the left side the capture region will be on the left side as well 
       * and when stepping with the right foot the legs will cross each other), and in this case we select the right foot even if in not the closest to the ICP. The same check is done for the right foot. For both checks 
       * is used a line at a fixed distance from the feet center to detect if the ICP is too left or too right.
       * Subsequently the swing time is computed based on the area of the area of the capture region and a fake next foot step is generated at the current location of the selected swing leg. In this way the 
       * walking high level state machine switches to single support with a desired foot step outside of the capture region and the normal push recovery will perform the recover adjusting the fake foot step inside of the capture region.
       * 
       * Note. since the double support push recovery takes a step even if the capture region area is very small, we have been able to achieve a multi-step push recovery continuously recovering from double support.
       * 
       */
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
                  projectedCapturePoint.changeFrame(capturePoint2d.getReferenceFrame());
                  projectedCapturePoint.set(capturePoint2d.getX(), capturePoint2d.getY(), 0.0);

                  for (RobotSide side : RobotSide.values())
                  {
                     footFrame = momentumBasedController.getFullRobotModel().getSoleFrame(side);
                     projectedCapturePoint.changeFrame(footFrame);
                     currentFootstep = createFootstepAtCurrentLocation(side);
                     calculateTouchdownFootPolygon(currentFootstep, side, projectedCapturePoint.getReferenceFrame(), footPolygon);
                     projectedCapturePoint2d.setIncludingFrame(projectedCapturePoint.getReferenceFrame(), projectedCapturePoint.getX(),
                           projectedCapturePoint.getY());

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
                     if (rightFootSupportWithOpposite)
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

                  /*
                   * In the following lines we select the swing time and the
                   * recover state (certain or uncertain) based on the computed
                   * doubleSupportInitialSwingTime (see method description), the
                   * area of the capture region computed using the
                   * MINIMUM_SWING_TIME_FOR_DOUBLE_SUPPORT_RECOVERY and the
                   * capture region computed using the MINIMUM_TIME_TO_REPLAN.
                   */
                  if (Double.isNaN(captureRegionAreaWithDoubleSupportMinimumSwingTime.getDoubleValue()))
                  {
                     if (existsAMinimumSwingTimeCaptureRegion.getBooleanValue())
                     {
                        // we try anyway with the capture region calculated using the minimum swing time, but the real swing time is the minimum for double support. 
                        // TODO N-step recover (already working but need to be improved)
                        swingTimeCalculationProvider.setSwingTime(MINIMUM_SWING_TIME_FOR_DOUBLE_SUPPORT_RECOVERY);
                        usingReducedSwingTime = true;
                        readyToGrabNextFootstep.set(false);
                        momentumBasedController.getUpcomingSupportLeg().set(transferToSide.getOppositeSide());
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

                  if (doubleSupportInitialSwingTime <= regularSwingTime)
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

   /**
    * This is a simple ICP planner that can work in two different ways. First way computes the desired ICP using a strait line position trajectory, while the second way 
    * (active if enableProjectionInsidePushRecoveryICPPlanner is true) simply defines the desired ICP as the projection of the ICP on the capture point trajectory,
    * which is the line connecting the initial ICP to the final). For the second way the desired ICP is set to be the previous desired ICP in order to try to slow down the current ICP.
    *
    */
   public class PushRecoveryICPPlanner extends StraightLinePositionTrajectoryGenerator
   {
      private FramePoint tempPositionToPack;
      private FrameVector tempVelocityToPack;
      private FramePoint2d previousDesiredICP;

      public PushRecoveryICPPlanner(String namePrefix, ReferenceFrame referenceFrame, DoubleProvider trajectoryTimeProvider,
            PositionProvider initialPositionProvider, PositionProvider finalPositionProvider, YoVariableRegistry parentRegistry)
      {
         super(namePrefix, referenceFrame, trajectoryTimeProvider, initialPositionProvider, finalPositionProvider, parentRegistry);
         tempPositionToPack = new FramePoint(worldFrame);
         tempVelocityToPack = new FrameVector(worldFrame);
      }

      public void getICPPosition(FramePoint2d desiredICPPositionToPack, FramePoint2d currentICP)
      {
         if (enableProjectionInsidePushRecoveryICPPlanner.getBooleanValue())
         {
            desiredICPPositionToPack.set(previousDesiredICP);
            previousDesiredICP.set(capturePointTrajectoryLine.orthogonalProjectionCopy(currentICP));
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
         if (enableProjectionInsidePushRecoveryICPPlanner.getBooleanValue())
         {
            desiredICPVelocityToPack.setToZero(worldFrame); // sure?
         }
         else
         {
            packVelocity(tempVelocityToPack);
            tempVelocityToPack.changeFrame(desiredICPVelocityToPack.getReferenceFrame());
            desiredICPVelocityToPack.set(tempVelocityToPack.getX(), tempVelocityToPack.getY());
         }
      }

      public void initialize(FramePoint2d desiredICPPositionToPack, FramePoint2d currentICP)
      {
         super.initialize();
         previousDesiredICP = new FramePoint2d(capturePointTrajectoryLine.orthogonalProjectionCopy(currentICP));
      }
   }

   /**
    * This method checks if the next footstep is inside of the capture region. If is outside it will be re-projected inside of the capture region.
    * The method can also handle the capture region calculation for "uncertain recover". In this case the capture region is calculated with the
    * MINIMUM_TIME_TO_REPLAN even if we are performing the step with the MINIMUM_SWING_TIME_FOR_DOUBLE_SUPPORT_RECOVERY.
    * 
    * @param swingSide
    * @param swingTimeRemaining
    * @param capturePoint2d
    * @param nextFootstep
    * @param omega0
    * @param footPolygon
    * @return
    */
   public boolean checkAndUpdateFootstep(RobotSide swingSide, double swingTimeRemaining, FramePoint2d capturePoint2d, Footstep nextFootstep, double omega0,
         FrameConvexPolygon2d footPolygon)
   {
      this.swingTimeRemaining.set(swingTimeRemaining);
      this.swingSideFromHighLevel = swingSide;
      this.nextFootstepFromHighLevel = nextFootstep;

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

         footstepWasProjectedInCaptureRegion.set(footstepAdjustor.adjustFootstep(nextFootstep, feet.get(nextFootstep.getRobotSide()), footPolygon.getCentroid(),
               captureRegionCalculator.getCaptureRegion(), isRecoveringFromDoubleSupportFall()));

         if (footstepWasProjectedInCaptureRegion.getBooleanValue())
         {
            recovering.set(true);
         }

         return footstepWasProjectedInCaptureRegion.getBooleanValue();
      }
      return false;
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

   private void calculateTouchdownFootPolygon(Footstep footstep, RobotSide robotSide, ReferenceFrame desiredFrame, FrameConvexPolygon2d polygonToPack)
   {
      List<FramePoint> expectedContactPoints = FootstepUtils.calculateExpectedContactPoints(footstep, feet.get(robotSide));
      int numberOfVertices = expectedContactPoints.size();
      for (int i = 0; i < numberOfVertices; i++)
      {
         expectedContactPoints.get(i).changeFrame(desiredFrame);
      }
      polygonToPack.setIncludingFrameByProjectionOntoXYPlaneAndUpdate(desiredFrame, expectedContactPoints);
   }

   /**
    * This method computes the minimum swing time based on the area of the capture region. In particular starting from the input swingTimeRemaining the swing time is reduced if the area of the capture region is less than 
    * a percentage of the foot area. This is required to guarantee that the capture region is large enough to safely recover from a push. 
    * The swing time is reduced up to a MINIMUM_TIME_TO_REPLAN (defined as a simple variable to exit from the loop, will not be used to perform the swing). 
    * In case the swing time is less than MINIMUM_TIME_TO_REPLAN we check if for this swing time exist a capture region, if exist we will try to perform a step with the MINIMUM_SWING_TIME_FOR_DOUBLE_SUPPORT_RECOVERY anyway,
    * if doesn't exist we should prepare to fall(see description inside of the state transition condition 'isFallingFromDoubleSupport').
    * 
    * @param swingSide
    * @param swingTimeRemaining
    * @param capturePoint2d
    * @param omega0
    * @param footPolygon
    * @return
    */
   private double computeMinimumSwingTime(RobotSide swingSide, double swingTimeRemaining, FramePoint2d capturePoint2d, double omega0,
         FrameConvexPolygon2d footPolygon)
   {
      double reducedSwingTime = swingTimeRemaining;
      captureRegionCalculator.calculateCaptureRegion(swingSide, swingTimeRemaining, capturePoint2d, omega0, footPolygon);

      // If the capture region is too small we reduce the swing time. 
      while (captureRegionCalculator.getCaptureRegionArea() < PushRecoveryControlModule.MINIMUN_CAPTURE_REGION_PERCENTAGE_OF_FOOT_AREA * footPolygon.getArea()
            || Double.isNaN(captureRegionCalculator.getCaptureRegionArea()))
      {
         reducedSwingTime = reducedSwingTime * REDUCE_SWING_TIME_MULTIPLIER;
         captureRegionCalculator.calculateCaptureRegion(swingSide, reducedSwingTime, capturePoint2d, omega0, footPolygon);

         // avoid infinite loops
         if (reducedSwingTime < MINIMUM_TIME_TO_REPLAN)
         {
            if (!Double.isNaN(captureRegionCalculator.getCaptureRegionArea()))
            {
               existsAMinimumSwingTimeCaptureRegion.set(true);
            }
            break;
         }
      }

      return reducedSwingTime;
   }

   /**
    * This method computes the minimum swing time based on the area of the capture region. 
    * 
    * @param swingSide
    * @param swingTimeRemaining
    * @param capturePoint2d
    * @return
    */
   private double computeInitialSwingTimeForDoubleSupportRecovery(RobotSide swingSide, double swingTimeRemaining, FramePoint2d capturePoint2d)
   {
      momentumBasedController.getContactPoints(feet.get(swingSide.getOppositeSide()), tempContactPoints);
      tempFootPolygon.setIncludingFrameByProjectionOntoXYPlaneAndUpdate(
            momentumBasedController.getReferenceFrames().getAnkleZUpFrame(swingSide.getOppositeSide()), tempContactPoints);

      captureRegionCalculator.calculateCaptureRegion(swingSide, MINIMUM_SWING_TIME_FOR_DOUBLE_SUPPORT_RECOVERY, capturePoint2d,
            icpAndMomentumBasedController.getOmega0(), tempFootPolygon);
      captureRegionAreaWithDoubleSupportMinimumSwingTime.set(captureRegionCalculator.getCaptureRegionArea());

      return computeMinimumSwingTime(swingSide, swingTimeRemaining, capturePoint2d, icpAndMomentumBasedController.getOmega0(), tempFootPolygon);
   }

   public double computeTimeToProjectDesiredICPToClosestPointOnTrajectoryToActualICP(FramePoint2d capturePoint2d, FramePoint2d constantCenterOfPressure,
         FramePoint2d initialICP, FramePoint2d finalDesiredICP, double omega0)
   {
      FrameLine2d capturePointTrajectoryLine = new FrameLine2d(finalDesiredICP, initialICP);
      FramePoint2d tmpCapturePoint = new FramePoint2d(capturePoint2d.getReferenceFrame(), capturePoint2d.getX(), capturePoint2d.getY());
      this.capturePointTrajectoryLine.setFrameLine2d(capturePointTrajectoryLine);

      //project current capture point to desired capture point trajectory line
      capturePointTrajectoryLine.orthogonalProjection(tmpCapturePoint);

      projectedCapturePoint.set(tmpCapturePoint.getPointCopy());

      return computeOffsetTimeToMoveDesiredICPToTrajectoryLine(tmpCapturePoint, constantCenterOfPressure, initialICP, finalDesiredICP, omega0);
   }

   private Footstep createFootstepAtCurrentLocation(RobotSide robotSide)
   {
      ContactablePlaneBody foot = feet.get(robotSide);
      ReferenceFrame footReferenceFrame = foot.getRigidBody().getParentJoint().getFrameAfterJoint();
      FramePose framePose = new FramePose(footReferenceFrame);
      framePose.changeFrame(worldFrame);

      PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("poseReferenceFrame", framePose);

      boolean trustHeight = true;
      Footstep footstep = new Footstep(foot.getRigidBody(), robotSide, foot.getSoleFrame(), poseReferenceFrame, trustHeight);

      return footstep;
   }
   
   public double getDistanceBetweenCurrentAndDesiredFootStep()
   {
      Footstep currentFootstep = createFootstepAtCurrentLocation(this.swingSideFromHighLevel);
      FramePoint framePointToPack = new FramePoint();
      this.nextFootstepFromHighLevel.getPositionIncludingFrame(framePointToPack);
      
      FramePoint framePointToPack2 = new FramePoint();
      currentFootstep.getPositionIncludingFrame(framePointToPack2);
      
      return framePointToPack.getXYplaneDistance(framePointToPack2);
   }

   /**
    * Given a desired capture point that is on the capture point trajectory, an initial capture point, a constant center of pressure, and omega, this 
    * method computes a time offset that will move the desired capture point to a desired point on that trajectory given by the argument capturePoint.
    * 
    * @param capturePoint
    * @param constantCenterOfPressure
    * @param initialICP
    * @param finalDesiredICP
    * @param omega0
    * @return
    */
   private double computeOffsetTimeToMoveDesiredICPToTrajectoryLine(FramePoint2d capturePoint, FramePoint2d constantCenterOfPressure, FramePoint2d initialICP,
         FramePoint2d finalDesiredICP, double omega0)
   {
      FramePoint2d tmpNumerator = new FramePoint2d(capturePoint.getReferenceFrame(), capturePoint.getX(), capturePoint.getY());
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

   /**
    * This method returns the swing time that is used to perform the double support push recovery swing, but reduced by a small amount.
    * This is required to activate the transition between single support and double support as soon as the swing is finished. 
    * 
    * @return reduced swing time
    */
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

   public boolean usePushRecoveryICPPlanner()
   {
      return this.usePushRecoveryICPPlanner.getBooleanValue();
   }

   public boolean isRecovering()
   {
      return recovering.getBooleanValue();
   }
}
