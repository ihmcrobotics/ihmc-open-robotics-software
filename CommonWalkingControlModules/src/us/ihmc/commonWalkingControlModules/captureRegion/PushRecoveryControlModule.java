package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
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
   private final BooleanYoVariable existsAMinimumSwingTimeCaptureRegion;
   private final BooleanYoVariable footstepWasProjectedInCaptureRegion;

   private final FootstepAdjustor footstepAdjustor;
   private final OneStepCaptureRegionCalculator captureRegionCalculator;

   private final BipedSupportPolygons bipedSupportPolygon;
   private final SideDependentList<? extends ContactablePlaneBody> feet;

   private final ReferenceFrame midFeetZUp;
   private final SideDependentList<ReferenceFrame> soleFrames;

   private final FrameConvexPolygon2d footPolygon = new FrameConvexPolygon2d();
   private final double footArea;

   private final DoubleYoVariable captureRegionAreaWithDoubleSupportMinimumSwingTime;

   private double omega0;
   private final FramePoint2d capturePoint2d = new FramePoint2d();

   private final FrameConvexPolygon2d supportPolygonInMidFeetZUp = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d reducedSupportPolygon = new FrameConvexPolygon2d();
   private final ConvexPolygonShrinker convexPolygonShrinker = new ConvexPolygonShrinker();

   private final FramePoint projectedCapturePoint = new FramePoint();
   private final FramePoint2d projectedCapturePoint2d = new FramePoint2d();
   private final SideDependentList<DoubleYoVariable> distanceICPToFeet = new SideDependentList<>();
   private final BooleanYoVariable isICPOutside;
   private final EnumYoVariable<RobotSide> closestFootToICP;
   private final EnumYoVariable<RobotSide> icpIsTooFarOnSide;
   private final EnumYoVariable<RobotSide> swingSideForDoubleSupportRecovery;
   private final DoubleYoVariable preferredSwingTimeRemainingForRecovering;

   public PushRecoveryControlModule(BipedSupportPolygons bipedSupportPolygons, MomentumBasedController momentumBasedController,
         WalkingControllerParameters walkingControllerParameters, YoVariableRegistry parentRegistry)
   {
      this.bipedSupportPolygon = bipedSupportPolygons;
      CommonHumanoidReferenceFrames referenceFrames = momentumBasedController.getReferenceFrames();
      feet = momentumBasedController.getContactableFeet();
      midFeetZUp = referenceFrames.getMidFeetZUpFrame();
      soleFrames = referenceFrames.getSoleFrames();

      enablePushRecovery = new BooleanYoVariable("enablePushRecovery", registry);
      enablePushRecovery.set(ENABLE);

      yoGraphicsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();
      captureRegionCalculator = new OneStepCaptureRegionCalculator(referenceFrames, walkingControllerParameters, registry, yoGraphicsListRegistry);
      footstepAdjustor = new FootstepAdjustor(feet, registry, yoGraphicsListRegistry);
      orientationStateVisualizer = new OrientationStateVisualizer(momentumBasedController.getPelvisZUpFrame(), yoGraphicsListRegistry, registry);

      footstepWasProjectedInCaptureRegion = new BooleanYoVariable("footstepWasProjectedInCaptureRegion", registry);
      recovering = new BooleanYoVariable("recovering", registry);
      recoveringFromDoubleSupportFall = new BooleanYoVariable("recoveringFromDoubleSupportFall", registry);
      existsAMinimumSwingTimeCaptureRegion = new BooleanYoVariable("existsAMinimumSwingTimeCaptureRegion", registry);

      captureRegionAreaWithDoubleSupportMinimumSwingTime = new DoubleYoVariable("captureRegionAreaWithMinimumSwingTime", registry);

      isICPOutside = new BooleanYoVariable("isICPOutside", registry);
      icpIsTooFarOnSide = new EnumYoVariable<>("ICPIsTooFarOnSide", registry, RobotSide.class, true);
      closestFootToICP = new EnumYoVariable<>("ClosestFootToICP", registry, RobotSide.class, true);
      swingSideForDoubleSupportRecovery = new EnumYoVariable<>("swingSideForDoubleSupportRecovery", registry, RobotSide.class, true);
      preferredSwingTimeRemainingForRecovering = new DoubleYoVariable("preferredSwingTimeRemainingForRecovering", registry);

      for (RobotSide robotSide : RobotSide.values)
      {
         String side = robotSide.getCamelCaseNameForMiddleOfExpression();
         DoubleYoVariable distanceICPToFoot = new DoubleYoVariable("DistanceICPTo" + side + "Foot", registry);
         distanceICPToFeet.put(robotSide, distanceICPToFoot);
      }

      footPolygon.setIncludingFrameAndUpdate(feet.get(RobotSide.LEFT).getContactPoints2d());
      footArea = footPolygon.getArea();

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
      if (!isICPOutside.getBooleanValue())
         return null;
      
      RobotSide swingSide = swingSideForDoubleSupportRecovery.getEnumValue();
      
      // Actually falling pretty hard there. Nothing to do for now.
      if (Double.isNaN(captureRegionAreaWithDoubleSupportMinimumSwingTime.getDoubleValue()) && !existsAMinimumSwingTimeCaptureRegion.getBooleanValue())
         return null;
      
      return swingSide;
   }
   
   public void initializeParametersForDoubleSupportPushRecovery()
   {
      recoveringFromDoubleSupportFall.set(true);
   }

   public void updateForDoubleSupport(FramePoint2d capturePoint2d, double omega0, double timeInState)
   {
      this.capturePoint2d.setIncludingFrame(capturePoint2d);
      FrameConvexPolygon2d supportPolygonInMidFeetZUp = bipedSupportPolygon.getSupportPolygonInMidFeetZUp();
      this.supportPolygonInMidFeetZUp.setIncludingFrameAndUpdate(supportPolygonInMidFeetZUp);
      this.omega0 = omega0;

      convexPolygonShrinker.shrinkConstantDistanceInto(supportPolygonInMidFeetZUp, DOUBLESUPPORT_SUPPORT_POLYGON_SHRINK_DISTANCE, reducedSupportPolygon);

      orientationStateVisualizer.updatePelvisVisualization();
      orientationStateVisualizer.updateReducedSupportPolygon(reducedSupportPolygon);

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

      if (!isICPOutside.getBooleanValue())
         return;

      projectedCapturePoint.setXYIncludingFrame(capturePoint2d);

      for (RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame soleFrame = soleFrames.get(robotSide);
         projectedCapturePoint.changeFrame(soleFrame);
         footPolygon.setIncludingFrameAndUpdate(bipedSupportPolygon.getFootPolygonInSoleFrame(robotSide));
         projectedCapturePoint2d.setByProjectionOntoXYPlaneIncludingFrame(projectedCapturePoint);

         boolean isICPTooFarOutside = robotSide.negateIfRightSide(projectedCapturePoint2d.getY()) > ICP_TOO_FAR_DISTANCE_THRESHOLD;
         if (isICPTooFarOutside)
            icpIsTooFarOnSide.set(robotSide);

         distanceICPToFeet.get(robotSide).set(projectedCapturePoint2d.distance(footPolygon.getCentroid()));
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

      capturePoint2d.changeFrame(worldFrame);

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

   public double computePreferredSwingTimeForRecovering(double swingTimeRemaining, RobotSide swingSide)
   {
      if (swingTimeRemaining < MINIMUM_TIME_TO_REPLAN)
      {
         // do not re-plan if we are almost at touch-down
         preferredSwingTimeRemainingForRecovering.set(MINIMUM_TIME_TO_REPLAN);
         return MINIMUM_TIME_TO_REPLAN;
      }

      double preferredSwingTime = swingTimeRemaining;
      RobotSide supportSide = swingSide.getOppositeSide();
      footPolygon.setIncludingFrameAndUpdate(bipedSupportPolygon.getFootPolygonInAnkleZUp(supportSide));
      captureRegionCalculator.calculateCaptureRegion(swingSide, preferredSwingTime, capturePoint2d, omega0, footPolygon);
      double captureRegionArea = captureRegionCalculator.getCaptureRegionArea();

      // If the capture region is too small we reduce the swing time.
      while (captureRegionArea < footArea || Double.isNaN(captureRegionArea))
      {
         preferredSwingTime = preferredSwingTime * REDUCE_SWING_TIME_MULTIPLIER;
         captureRegionCalculator.calculateCaptureRegion(swingSide, preferredSwingTime, capturePoint2d, omega0, footPolygon);
         captureRegionArea = captureRegionCalculator.getCaptureRegionArea();

         // avoid infinite loops
         if (preferredSwingTime < MINIMUM_TIME_TO_REPLAN)
         {
            preferredSwingTime = MINIMUM_TIME_TO_REPLAN;
            break;
         }
      }

      preferredSwingTimeRemainingForRecovering.set(MINIMUM_TIME_TO_REPLAN);
      return preferredSwingTime;
   }

   /**
    * This method checks if the next footstep is inside of the capture region. If is outside it will be re-projected inside of the capture region.
    * The method can also handle the capture region calculation for "uncertain recover". In this case the capture region is calculated with the
    * MINIMUM_TIME_TO_REPLAN even if we are performing the step with the MINIMUM_SWING_TIME_FOR_DOUBLE_SUPPORT_RECOVERY.
    * @param swingTimeRemaining
    * @param nextFootstep
    *
    * @return
    */
   public boolean checkAndUpdateFootstep(double swingTimeRemaining, Footstep nextFootstep)
   {
      RobotSide swingSide = nextFootstep.getRobotSide();
      RobotSide supportSide = swingSide.getOppositeSide();
      footPolygon.setIncludingFrameAndUpdate(bipedSupportPolygon.getFootPolygonInAnkleZUp(supportSide));

      if (swingTimeRemaining < MINIMUM_TIME_TO_REPLAN)
      {
         // do not re-plan if we are almost at touch-down
         return false;
      }

      double preferredSwingTimeForRecovering = computePreferredSwingTimeForRecovering(swingTimeRemaining, swingSide);
      captureRegionCalculator.calculateCaptureRegion(swingSide, preferredSwingTimeForRecovering, capturePoint2d, omega0, footPolygon);

      FramePoint2d footCentroid = footPolygon.getCentroid();
      FrameConvexPolygon2d captureRegion = captureRegionCalculator.getCaptureRegion();
      boolean hasFootstepBeenAdjusted = footstepAdjustor.adjustFootstep(nextFootstep, footCentroid, captureRegion);
      footstepWasProjectedInCaptureRegion.set(hasFootstepBeenAdjusted);

      if (footstepWasProjectedInCaptureRegion.getBooleanValue())
      {
         recovering.set(true);
      }

      return footstepWasProjectedInCaptureRegion.getBooleanValue();
   }

   public Footstep createFootstepForRecoveringFromDisturbance(RobotSide swingSide, double swingTimeRemaining)
   {
      if (!enablePushRecovery.getBooleanValue())
         return null;
      
      Footstep footstepForPushRecovery = createFootstepAtCurrentLocation(swingSide);
      checkAndUpdateFootstep(swingTimeRemaining, footstepForPushRecovery);
      return footstepForPushRecovery;
   }

   public void reset()
   {
      footstepWasProjectedInCaptureRegion.set(false);
      recovering.set(false);
      captureRegionCalculator.hideCaptureRegion();

      recoveringFromDoubleSupportFall.set(false);
      existsAMinimumSwingTimeCaptureRegion.set(false);
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
