package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.trajectories.SwingTimeCalculationProvider;
import us.ihmc.utilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.utilities.humanoidRobot.footstep.Footstep;
import us.ihmc.utilities.humanoidRobot.frames.CommonHumanoidReferenceFrames;
import us.ihmc.utilities.math.geometry.ConvexPolygonShrinker;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

public class PushRecoveryControlModule
{
   private static final boolean ENABLE = false;

   private static final double DOUBLESUPPORT_SUPPORT_POLYGON_SHRINK_DISTANCE = 0.02;
   private static final double MINIMUM_TIME_TO_REPLAN = 0.1;
   private static final double MINIMUM_SWING_TIME_FOR_DOUBLE_SUPPORT_RECOVERY = 0.3;
   private static final double MINIMUN_CAPTURE_REGION_PERCENTAGE_OF_FOOT_AREA = 2.5;
   private static final double REDUCE_SWING_TIME_MULTIPLIER = 0.9;
   private static final double ICP_TOO_FAR_DISTANCE_THRESHOLD = 0.01;
   private static final double MINIMUM_TIME_BEFORE_RECOVER_WITH_REDUCED_POLYGON = 0.3;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoGraphicsListRegistry yoGraphicsListRegistry;
   private final OrientationStateVisualizer orientationStateVisualizer;

   private final BooleanYoVariable enablePushRecovery;

   private final BooleanYoVariable recovering;
   private final BooleanYoVariable recoveringFromDoubleSupportFall;
   private final BooleanYoVariable tryingUncertainRecover;
   private final BooleanYoVariable existsAMinimumSwingTimeCaptureRegion;
   private final BooleanYoVariable footstepWasProjectedInCaptureRegion;

   private final FootstepAdjustor footstepAdjustor;
   private final OneStepCaptureRegionCalculator captureRegionCalculator;

   private final BipedSupportPolygons bipedSupportPolygon;
   private final SideDependentList<? extends ContactablePlaneBody> feet;
   private final SwingTimeCalculationProvider swingTimeCalculationProvider;

   private final ReferenceFrame midFeetZUp;
   private final SideDependentList<ReferenceFrame> soleFrames;

   private final FrameConvexPolygon2d tempFootPolygon;

   private final DoubleYoVariable captureRegionAreaWithDoubleSupportMinimumSwingTime;

   private Footstep recoverFromDoubleSupportFallFootstep;
   private double omega0;
   private final FramePoint2d capturePoint2d = new FramePoint2d();

   private final FrameConvexPolygon2d supportPolygonInMidFeetZUp = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d reducedSupportPolygon = new FrameConvexPolygon2d();
   private final ConvexPolygonShrinker convexPolygonShrinker = new ConvexPolygonShrinker();

   private final FramePoint projectedCapturePoint = new FramePoint();
   private final FramePoint2d projectedCapturePoint2d = new FramePoint2d();
   private Footstep currentFootstep;
   private final FrameConvexPolygon2d footPolygon = new FrameConvexPolygon2d();
   private final SideDependentList<DoubleYoVariable> distanceICPToFeet = new SideDependentList<>();
   private final DoubleYoVariable doubleSupportInitialSwingTime;
   private final BooleanYoVariable isICPOutside;
   private final EnumYoVariable<RobotSide> closestFootToICP;
   private final EnumYoVariable<RobotSide> icpIsTooFarOnSide;
   private final EnumYoVariable<RobotSide> swingSideForDoubleSupportRecovery;

   public PushRecoveryControlModule(BipedSupportPolygons bipedSupportPolygons, MomentumBasedController momentumBasedController,
         WalkingControllerParameters walkingControllerParameters, SwingTimeCalculationProvider swingTimeCalculationProvider, YoVariableRegistry parentRegistry)
   {
      this.bipedSupportPolygon = bipedSupportPolygons;
      this.swingTimeCalculationProvider = swingTimeCalculationProvider;
      CommonHumanoidReferenceFrames referenceFrames = momentumBasedController.getReferenceFrames();
      feet = momentumBasedController.getContactableFeet();
      midFeetZUp = referenceFrames.getMidFeetZUpFrame();
      soleFrames = referenceFrames.getSoleFrames();

      enablePushRecovery = new BooleanYoVariable("enablePushRecovery", registry);
      enablePushRecovery.set(ENABLE);

      yoGraphicsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();
      captureRegionCalculator = new OneStepCaptureRegionCalculator(referenceFrames, walkingControllerParameters, registry, yoGraphicsListRegistry);
      footstepAdjustor = new FootstepAdjustor(registry, yoGraphicsListRegistry);
      orientationStateVisualizer = new OrientationStateVisualizer(momentumBasedController.getPelvisZUpFrame(), yoGraphicsListRegistry, registry);

      footstepWasProjectedInCaptureRegion = new BooleanYoVariable("footstepWasProjectedInCaptureRegion", registry);
      recovering = new BooleanYoVariable("recovering", registry);
      recoveringFromDoubleSupportFall = new BooleanYoVariable("recoveringFromDoubleSupportFall", registry);
      tryingUncertainRecover = new BooleanYoVariable("tryingUncertainRecover", registry);
      existsAMinimumSwingTimeCaptureRegion = new BooleanYoVariable("existsAMinimumSwingTimeCaptureRegion", registry);

      tempFootPolygon = new FrameConvexPolygon2d(worldFrame);
      captureRegionAreaWithDoubleSupportMinimumSwingTime = new DoubleYoVariable("captureRegionAreaWithMinimumSwingTime", registry);

      isICPOutside = new BooleanYoVariable("isICPOutside", registry);
      icpIsTooFarOnSide = new EnumYoVariable<>("ICPIsTooFarOnSide", registry, RobotSide.class, true);
      closestFootToICP = new EnumYoVariable<>("ClosestFootToICP", registry, RobotSide.class, true);
      swingSideForDoubleSupportRecovery = new EnumYoVariable<>("swingSideForDoubleSupportRecovery", registry, RobotSide.class, true);
      doubleSupportInitialSwingTime = new DoubleYoVariable("SwingTimeForDoubleSupportRecovery", registry);

      for (RobotSide robotSide : RobotSide.values)
      {
         String side = robotSide.getCamelCaseNameForMiddleOfExpression();
         DoubleYoVariable distanceICPToFoot = new DoubleYoVariable("DistanceICPTo" + side + "Foot", registry);
         distanceICPToFeet.put(robotSide, distanceICPToFoot);
      }

      parentRegistry.addChild(registry);

      reset();
   }

   /**
    * Return null if the robot is not falling.
    * If the robot is falling, it returns the suggested swingSide to recover. 
    * @param timeInState
    */
   public RobotSide isRobotFallingFromDoubleSupport(double timeInState)
   {
      if (!isICPOutside.getBooleanValue() || recoverFromDoubleSupportFallFootstep != null)
         return null;
      
      RobotSide swingSide = swingSideForDoubleSupportRecovery.getEnumValue();
      
      // Actually falling pretty hard there. Nothing to do for now.
      if (Double.isNaN(captureRegionAreaWithDoubleSupportMinimumSwingTime.getDoubleValue()) && !existsAMinimumSwingTimeCaptureRegion.getBooleanValue())
         return null;
      
      return swingSide;
   }
   
   public void initializeParametersForDoubleSupportPushRecovery()
   {
      swingTimeCalculationProvider.setSwingTime(doubleSupportInitialSwingTime.getDoubleValue());
      recoverFromDoubleSupportFallFootstep = currentFootstep;
      recoveringFromDoubleSupportFall.set(true);
   }

   public void updateForDoubleSupport(double timeInState)
   {
      // Initialize variables
      icpIsTooFarOnSide.set(null);
      closestFootToICP.set(null);

      for (RobotSide robotSide : RobotSide.values)
         distanceICPToFeet.get(robotSide).set(Double.NaN);

      capturePoint2d.changeFrame(midFeetZUp);

      if (timeInState < MINIMUM_TIME_BEFORE_RECOVER_WITH_REDUCED_POLYGON)
         isICPOutside.set(!supportPolygonInMidFeetZUp.isPointInside(capturePoint2d));
      else
         isICPOutside.set(!reducedSupportPolygon.isPointInside(capturePoint2d));

      if (!isICPOutside.getBooleanValue() || recoverFromDoubleSupportFallFootstep != null)
         return;

      projectedCapturePoint.setXYIncludingFrame(capturePoint2d);

      for (RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame soleFrame = soleFrames.get(robotSide);
         projectedCapturePoint.changeFrame(soleFrame);
         currentFootstep = createFootstepAtCurrentLocation(robotSide);
         footPolygon.setIncludingFrameAndUpdate(bipedSupportPolygon.getFootPolygonInSoleFrame(robotSide));
         projectedCapturePoint2d.setByProjectionOntoXYPlaneIncludingFrame(projectedCapturePoint);

         boolean isICPTooFarOutside = robotSide.negateIfRightSide(projectedCapturePoint2d.getY()) > ICP_TOO_FAR_DISTANCE_THRESHOLD;
         if (isICPTooFarOutside)
            icpIsTooFarOnSide.set(robotSide);

         distanceICPToFeet.get(robotSide).set(footPolygon.distance(projectedCapturePoint2d));
      }

      boolean isLeftFootCloser = distanceICPToFeet.get(RobotSide.LEFT).getDoubleValue() <= distanceICPToFeet.get(RobotSide.RIGHT).getDoubleValue();
      closestFootToICP.set(isLeftFootCloser ? RobotSide.LEFT : RobotSide.RIGHT);

      RobotSide swingSide;
      // Edge case: the ICP is really far out such that it is preferable to swing the loaded foot to prevent doing a crossover step or simply falling on the outside.
      if (icpIsTooFarOnSide.getEnumValue() != null && icpIsTooFarOnSide.getEnumValue() == closestFootToICP.getEnumValue())
         swingSide = closestFootToICP.getEnumValue();
      else
         // Normal case: it is better to swing the foot that is the farthest from the ICP so the ICP will move slower in the upcoming single support.
         swingSide = closestFootToICP.getEnumValue().getOppositeSide();

      currentFootstep = createFootstepAtCurrentLocation(swingSide);
      capturePoint2d.changeFrame(worldFrame);

      doubleSupportInitialSwingTime.set(computeInitialSwingTimeForDoubleSupportRecovery(swingSide, swingTimeCalculationProvider.getValue(), capturePoint2d));
      doubleSupportInitialSwingTime.set(Math.max(doubleSupportInitialSwingTime.getDoubleValue(), MINIMUM_SWING_TIME_FOR_DOUBLE_SUPPORT_RECOVERY));

      if (Double.isNaN(captureRegionAreaWithDoubleSupportMinimumSwingTime.getDoubleValue()))
      {
         if (existsAMinimumSwingTimeCaptureRegion.getBooleanValue())
         {
            doubleSupportInitialSwingTime.set(MINIMUM_SWING_TIME_FOR_DOUBLE_SUPPORT_RECOVERY);
         }
      }

      swingSideForDoubleSupportRecovery.set(swingSide);
   }

   public void updatePushRecoveryInputs(FramePoint2d capturePoint2d, double omega0)
   {
      this.capturePoint2d.setIncludingFrame(capturePoint2d);
      FrameConvexPolygon2d supportPolygonInMidFeetZUp = bipedSupportPolygon.getSupportPolygonInMidFeetZUp();
      this.supportPolygonInMidFeetZUp.setIncludingFrameAndUpdate(supportPolygonInMidFeetZUp);
      this.omega0 = omega0;

      convexPolygonShrinker.shrinkConstantDistanceInto(supportPolygonInMidFeetZUp, DOUBLESUPPORT_SUPPORT_POLYGON_SHRINK_DISTANCE, reducedSupportPolygon);

      orientationStateVisualizer.updatePelvisVisualization();
      orientationStateVisualizer.updateReducedSupportPolygon(reducedSupportPolygon);
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

         footstepWasProjectedInCaptureRegion.set(footstepAdjustor.adjustFootstep(nextFootstep, feet.get(nextFootstep.getRobotSide()),
               footPolygon.getCentroid(), captureRegionCalculator.getCaptureRegion(), isRecoveringFromDoubleSupportFall()));

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

      recoveringFromDoubleSupportFall.set(false);
      tryingUncertainRecover.set(false);
      existsAMinimumSwingTimeCaptureRegion.set(false);
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
      while (captureRegionCalculator.getCaptureRegionArea() < MINIMUN_CAPTURE_REGION_PERCENTAGE_OF_FOOT_AREA * footPolygon.getArea()
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
      tempFootPolygon.setIncludingFrameAndUpdate(bipedSupportPolygon.getFootPolygonInAnkleZUp(swingSide.getOppositeSide()));
      captureRegionCalculator.calculateCaptureRegion(swingSide, MINIMUM_SWING_TIME_FOR_DOUBLE_SUPPORT_RECOVERY, capturePoint2d, omega0, tempFootPolygon);
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

   public boolean isEnabled()
   {
      return enablePushRecovery.getBooleanValue();
   }

   public void setIsEnabled(boolean enable)
   {
      enablePushRecovery.set(enable);
   }

   public boolean isRecoveringFromDoubleSupportFall()
   {
      return recoveringFromDoubleSupportFall.getBooleanValue();
   }

   public boolean isRecovering()
   {
      return recovering.getBooleanValue();
   }
}
