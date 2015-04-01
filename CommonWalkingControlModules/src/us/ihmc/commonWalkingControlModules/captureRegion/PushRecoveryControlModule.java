package us.ihmc.commonWalkingControlModules.captureRegion;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.trajectories.SwingTimeCalculationProvider;
import us.ihmc.plotting.shapes.PointArtifact;
import us.ihmc.utilities.humanoidRobot.frames.CommonHumanoidReferenceFrames;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.plotting.YoArtifactLine2d;
import us.ihmc.utilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.utilities.humanoidRobot.footstep.Footstep;
import us.ihmc.yoUtilities.math.frames.YoFrameLine2d;
import us.ihmc.yoUtilities.stateMachines.StateMachine;
import us.ihmc.yoUtilities.stateMachines.StateTransitionCondition;


public class PushRecoveryControlModule
{
   private static final boolean ENABLE = false;
   private static final boolean ENABLE_DOUBLE_SUPPORT_PUSH_RECOVERY = false;
   private static final boolean USE_PUSH_RECOVERY_ICP_PLANNER = false;
   private static final boolean ENABLE_PROJECTION_INSIDE_PUSH_RECOVERY_ICP_PLANNER = true;

   private static final double DOUBLESUPPORT_SUPPORT_POLYGON_SHRINK_DISTANCE = 0.02;
   private static final double TRUST_TIME_SCALE = 0.95;
   private static final double MINIMUM_TIME_TO_REPLAN = 0.1;
   private static final double MINIMUM_SWING_TIME_FOR_DOUBLE_SUPPORT_RECOVERY = 0.3;
   private static final double MINIMUN_CAPTURE_REGION_PERCENTAGE_OF_FOOT_AREA = 2.5;
   private static final double REDUCE_SWING_TIME_MULTIPLIER = 0.9;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanYoVariable enablePushRecovery;
   private final BooleanYoVariable usePushRecoveryICPPlanner;
   private final BooleanYoVariable enablePushRecoveryFromDoubleSupport;
   private final BooleanYoVariable enableProjectionInsidePushRecoveryICPPlanner;
   private final MomentumBasedController momentumBasedController;
   private final OrientationStateVisualizer orientationStateVisualizer;
   private final FootstepAdjustor footstepAdjustor;
   private final OneStepCaptureRegionCalculator captureRegionCalculator;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final BooleanYoVariable footstepWasProjectedInCaptureRegion;
   private final CommonHumanoidReferenceFrames referenceFrames;
   private final StateMachine<?> stateMachine;
   private final SwingTimeCalculationProvider swingTimeCalculationProvider;

   private boolean recoveringFromDoubleSupportFall;
   private boolean usingReducedSwingTime;
   private double reducedSwingTime;
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
   
   private Footstep recoverFromDoubleSupportFallFootstep;
   private double omega0;
   private final FramePoint2d capturePoint2d = new FramePoint2d();
   private final FrameConvexPolygon2d supportPolygonInMidFeetZUp = new FrameConvexPolygon2d();

   public PushRecoveryControlModule(MomentumBasedController momentumBasedController, WalkingControllerParameters walkingControllerParameters,
         BooleanYoVariable readyToGrabNextFootstep, StateMachine<?> stateMachine, YoVariableRegistry parentRegistry,
         SwingTimeCalculationProvider swingTimeCalculationProvider)
   {
      this.momentumBasedController = momentumBasedController;
      this.readyToGrabNextFootstep = readyToGrabNextFootstep;
      this.referenceFrames = momentumBasedController.getReferenceFrames();
      this.stateMachine = stateMachine;
      this.swingTimeCalculationProvider = swingTimeCalculationProvider;
      this.feet = momentumBasedController.getContactableFeet();

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
      private static final double ICP_TOO_FAR_DISTANCE_THRESHOLD = 0.01;
      private static final double MINIMUM_TIME_BEFORE_RECOVER_WITH_REDUCED_POLYGON = 0.3;

      private RobotSide swingSide = null;
      private RobotSide transferToSide = null;

      private final YoVariableRegistry registry;

      private final RigidBodyTransform fromWorldToPelvis = new RigidBodyTransform();
      private final FrameConvexPolygon2d reducedSupportPolygon;
      private final ReferenceFrame midFeetZUp;
      private double regularSwingTime;
      private ReferenceFrame soleFrame;
      private FramePoint projectedCapturePoint;
      private FramePoint2d projectedCapturePoint2d;
      private Footstep currentFootstep = null;
      private FrameConvexPolygon2d footPolygon;
      private final SideDependentList<DoubleYoVariable> distanceICPToFeet = new SideDependentList<>();
      private final DoubleYoVariable doubleSupportInitialSwingTime;
      private final BooleanYoVariable isICPOutside;
      private final EnumYoVariable<RobotSide> closestFootToICP;
      private final EnumYoVariable<RobotSide> icpIsTooFarOnSide;

      public IsFallingFromDoubleSupportCondition(RobotSide robotSide)
      {
         this.transferToSide = robotSide;
         this.reducedSupportPolygon = new FrameConvexPolygon2d(supportPolygonInMidFeetZUp);
         this.projectedCapturePoint = new FramePoint(worldFrame, 0.0, 0.0, 0.0);
         this.projectedCapturePoint2d = new FramePoint2d(worldFrame, 0.0, 0.0);
         this.footPolygon = new FrameConvexPolygon2d(worldFrame);
         midFeetZUp = referenceFrames.getMidFeetZUpFrame();

         String namePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
         isICPOutside = new BooleanYoVariable(namePrefix + "IsICPOutside", registry);
         icpIsTooFarOnSide = new EnumYoVariable<>(namePrefix + "ICPIsTooFarOnSide", registry, RobotSide.class, true);
         closestFootToICP = new EnumYoVariable<>(namePrefix + "ClosestFootToICP", registry, RobotSide.class, true);
         doubleSupportInitialSwingTime = new DoubleYoVariable(namePrefix + "SwingTimeForDoubleSupportRecovery", registry);

         for (RobotSide side : RobotSide.values)
         {
            DoubleYoVariable distanceICPToFoot = new DoubleYoVariable(namePrefix + "FallCondition" + "DistanceICPTo" + side.getCamelCaseNameForMiddleOfExpression() + "Foot", registry);
            distanceICPToFeet.put(side, distanceICPToFoot);
         }

         PushRecoveryControlModule.this.registry.addChild(registry);
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
               // Initialize variables
               icpIsTooFarOnSide.set(null);
               closestFootToICP.set(null);
               
               for (RobotSide robotSide : RobotSide.values)
                  distanceICPToFeet.get(robotSide).set(Double.NaN);

               // get current robot status
               reducedSupportPolygon.changeFrame(midFeetZUp);
               reducedSupportPolygon.setAndUpdate(supportPolygonInMidFeetZUp);
               reducedSupportPolygon.shrink(DOUBLESUPPORT_SUPPORT_POLYGON_SHRINK_DISTANCE);

               capturePoint2d.changeFrame(midFeetZUp);

               // update the visualization
               momentumBasedController.getFullRobotModel().getPelvis().getBodyFixedFrame().getTransformToDesiredFrame(fromWorldToPelvis, worldFrame);
               orientationStateVisualizer.updatePelvisReferenceFrame(fromWorldToPelvis);
               orientationStateVisualizer.updateReducedSupportPolygon(reducedSupportPolygon);

               if (stateMachine.timeInCurrentState() < MINIMUM_TIME_BEFORE_RECOVER_WITH_REDUCED_POLYGON)
               {
                  isICPOutside.set(!supportPolygonInMidFeetZUp.isPointInside(capturePoint2d));
               }
               else
               {
                  isICPOutside.set(!reducedSupportPolygon.isPointInside(capturePoint2d));
               }

               if (isICPOutside.getBooleanValue() && recoverFromDoubleSupportFallFootstep == null)
               {
                  projectedCapturePoint.setXYIncludingFrame(capturePoint2d);

                  for (RobotSide robotSide : RobotSide.values)
                  {
                     soleFrame = momentumBasedController.getFullRobotModel().getSoleFrame(robotSide);
                     projectedCapturePoint.changeFrame(soleFrame);
                     currentFootstep = createFootstepAtCurrentLocation(robotSide);
                     footPolygon.setIncludingFrameAndUpdate(feet.get(robotSide).getContactPoints2d());
                     projectedCapturePoint2d.setByProjectionOntoXYPlaneIncludingFrame(projectedCapturePoint);

                     boolean isICPTooFarOutside = robotSide.negateIfRightSide(projectedCapturePoint2d.getY()) > ICP_TOO_FAR_DISTANCE_THRESHOLD;
                     if (isICPTooFarOutside)
                        icpIsTooFarOnSide.set(robotSide);

                     distanceICPToFeet.get(robotSide).set(footPolygon.distance(projectedCapturePoint2d));
                  }
                  
                  boolean isLeftFootCloser = distanceICPToFeet.get(RobotSide.LEFT).getDoubleValue() <= distanceICPToFeet.get(RobotSide.RIGHT).getDoubleValue();
                  closestFootToICP.set(isLeftFootCloser ? RobotSide.LEFT : RobotSide.RIGHT);
                  
                  // Edge case: the ICP is really far out such that it is preferable to swing the loaded foot to prevent doing a crossover step or simply falling on the outside.
                  if (icpIsTooFarOnSide.getEnumValue() != null && icpIsTooFarOnSide.getEnumValue() == closestFootToICP.getEnumValue())
                     swingSide = closestFootToICP.getEnumValue();
                  else // Normal case: it is better to swing the foot that is the farthest from the ICP so the ICP will move slower in the upcoming single support.
                     swingSide = closestFootToICP.getEnumValue().getOppositeSide();

                  if (transferToSide == swingSide)
                  {
                     return false;
                  }

                  currentFootstep = createFootstepAtCurrentLocation(swingSide);
                  capturePoint2d.changeFrame(worldFrame);

                  regularSwingTime = swingTimeCalculationProvider.getValue();

                  doubleSupportInitialSwingTime.set(computeInitialSwingTimeForDoubleSupportRecovery(swingSide, regularSwingTime, capturePoint2d));

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
                        recoverFromDoubleSupportFallFootstep = currentFootstep;
                        recoveringFromDoubleSupportFall = true;
                        reducedSwingTime = MINIMUM_SWING_TIME_FOR_DOUBLE_SUPPORT_RECOVERY;
                        tryingUncertainRecover.set(true);
                        return true;
                     }

                     // TODO prepare to fall
                     return false;
                  }

                  if (doubleSupportInitialSwingTime.getDoubleValue() < MINIMUM_SWING_TIME_FOR_DOUBLE_SUPPORT_RECOVERY)
                  {
                     // here we try to take a step with the double support minimum swing time 
                     swingTimeCalculationProvider.setSwingTime(MINIMUM_SWING_TIME_FOR_DOUBLE_SUPPORT_RECOVERY);
                     usingReducedSwingTime = true;
                     readyToGrabNextFootstep.set(false);
                     momentumBasedController.getUpcomingSupportLeg().set(transferToSide.getOppositeSide());
                     recoverFromDoubleSupportFallFootstep = currentFootstep;
                     recoveringFromDoubleSupportFall = true;
                     reducedSwingTime = MINIMUM_SWING_TIME_FOR_DOUBLE_SUPPORT_RECOVERY;
                     return true;
                  }

                  if (doubleSupportInitialSwingTime.getDoubleValue() <= regularSwingTime)
                  {
                     swingTimeCalculationProvider.setSwingTime(doubleSupportInitialSwingTime.getDoubleValue());
                     usingReducedSwingTime = true;
                     readyToGrabNextFootstep.set(false);
                     momentumBasedController.getUpcomingSupportLeg().set(transferToSide.getOppositeSide());
                     recoverFromDoubleSupportFallFootstep = currentFootstep;
                     recoveringFromDoubleSupportFall = true;
                     reducedSwingTime = doubleSupportInitialSwingTime.getDoubleValue();
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

   public void updatePushRecoveryInputs(FramePoint2d capturePoint2d, FrameConvexPolygon2d supportPolygonInMidFeetZUp, double omega0)
   {
      this.capturePoint2d.setIncludingFrame(capturePoint2d);
      this.supportPolygonInMidFeetZUp.setIncludingFrameAndUpdate(supportPolygonInMidFeetZUp);
      this.omega0 = omega0;
   }

   /**
    * This method checks if the next footstep is inside of the capture region. If is outside it will be re-projected inside of the capture region.
    * The method can also handle the capture region calculation for "uncertain recover". In this case the capture region is calculated with the
    * MINIMUM_TIME_TO_REPLAN even if we are performing the step with the MINIMUM_SWING_TIME_FOR_DOUBLE_SUPPORT_RECOVERY.
    * 
    * @param swingSide
    * @param swingTimeRemaining
    * @param nextFootstep
    * @param footPolygon
    * @return
    */
   public boolean checkAndUpdateFootstep(RobotSide swingSide, double swingTimeRemaining, Footstep nextFootstep, FrameConvexPolygon2d footPolygon)
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
      recoverFromDoubleSupportFallFootstep = null;
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
            omega0, tempFootPolygon);
      captureRegionAreaWithDoubleSupportMinimumSwingTime.set(captureRegionCalculator.getCaptureRegionArea());

      return computeMinimumSwingTime(swingSide, swingTimeRemaining, capturePoint2d, omega0, tempFootPolygon);
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
   
   public Footstep getRecoverFromDoubleSupportFootStep()
   {
      return recoverFromDoubleSupportFallFootstep;
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
      recoverFromDoubleSupportFallFootstep = recoverFootStep;
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
