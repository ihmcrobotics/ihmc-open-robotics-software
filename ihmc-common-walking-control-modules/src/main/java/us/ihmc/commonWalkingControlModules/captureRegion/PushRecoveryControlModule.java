package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class PushRecoveryControlModule
{
   private static final boolean ENABLE = false;

   private static final double MINIMUM_TIME_TO_REPLAN = 0.1;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private final YoBoolean enablePushRecovery;

   private final YoBoolean recovering;
   private final YoBoolean recoveringFromDoubleSupportFall;
   private final YoBoolean footstepWasProjectedInCaptureRegion;

   private final SideDependentList<YoDouble> distanceICPToFeet = new SideDependentList<>();
   private final YoBoolean isICPOutside;
   private final YoBoolean isICPErrorTooLarge;
   private final YoDouble icpErrorThreshold;
   private final YoEnum<RobotSide> closestFootToICP;
   private final YoEnum<RobotSide> swingSideForDoubleSupportRecovery;

   private final GlitchFilteredYoBoolean isRobotBackToSafeState;
   private final YoBoolean isCaptureRegionEmpty;

   private final FootstepAdjustor footstepAdjustor;
   private final OneStepCaptureRegionCalculator captureRegionCalculator;

   private final BipedSupportPolygons bipedSupportPolygon;
   private final SideDependentList<? extends ContactablePlaneBody> feet;

   private final ReferenceFrame midFeetZUp;
   private final SideDependentList<MovingReferenceFrame> soleFrames;

   private final FrameConvexPolygon2d footPolygon = new FrameConvexPolygon2d();

   private double omega0;
   private final FramePoint2D desiredCapturePoint2d = new FramePoint2D();
   private final FramePoint2D capturePoint2d = new FramePoint2D();

   private final FramePoint3D projectedCapturePoint = new FramePoint3D();
   private final FramePoint2D projectedCapturePoint2d = new FramePoint2D();

   public PushRecoveryControlModule(BipedSupportPolygons bipedSupportPolygons, HighLevelHumanoidControllerToolbox controllerToolbox,
         WalkingControllerParameters walkingControllerParameters, YoVariableRegistry parentRegistry)
   {
      this.bipedSupportPolygon = bipedSupportPolygons;
      CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      feet = controllerToolbox.getContactableFeet();
      midFeetZUp = referenceFrames.getMidFeetZUpFrame();
      soleFrames = referenceFrames.getSoleFrames();

      enablePushRecovery = new YoBoolean("enablePushRecovery", registry);
      enablePushRecovery.set(ENABLE); // todo add some smartness on whether ot not to enable this if using the icp optimization

      yoGraphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();
      captureRegionCalculator = new OneStepCaptureRegionCalculator(referenceFrames, walkingControllerParameters, registry, yoGraphicsListRegistry);
      footstepAdjustor = new FootstepAdjustor(feet, registry, yoGraphicsListRegistry);

      footstepWasProjectedInCaptureRegion = new YoBoolean("footstepWasProjectedInCaptureRegion", registry);
      recovering = new YoBoolean("recovering", registry);
      recoveringFromDoubleSupportFall = new YoBoolean("recoveringFromDoubleSupportFall", registry);

      isICPOutside = new YoBoolean("isICPOutside", registry);
      isICPErrorTooLarge = new YoBoolean("isICPErrorTooLarge", registry);
      icpErrorThreshold = new YoDouble("icpErrorThreshold", registry);
      icpErrorThreshold.set(0.05);
      closestFootToICP = new YoEnum<>("ClosestFootToICP", registry, RobotSide.class, true);
      swingSideForDoubleSupportRecovery = new YoEnum<>("swingSideForDoubleSupportRecovery", registry, RobotSide.class, true);
      swingSideForDoubleSupportRecovery.set(null);

      isRobotBackToSafeState = new GlitchFilteredYoBoolean("isRobotBackToSafeState", registry, 100);
      isCaptureRegionEmpty = new YoBoolean("isCaptureRegionEmpty", registry);

      for (RobotSide robotSide : RobotSide.values)
      {
         String side = robotSide.getCamelCaseNameForMiddleOfExpression();
         YoDouble distanceICPToFoot = new YoDouble("DistanceICPTo" + side + "Foot", registry);
         distanceICPToFeet.put(robotSide, distanceICPToFoot);
      }

      footPolygon.setIncludingFrameAndUpdate(feet.get(RobotSide.LEFT).getContactPoints2d());

      parentRegistry.addChild(registry);

      reset();
   }

   public void updateCaptureRegion(double swingTimeRemaining, double omega0, RobotSide swingSide, FramePoint2D capturePoint2d)
   {
      footPolygon.setIncludingFrameAndUpdate(bipedSupportPolygon.getFootPolygonInAnkleZUp(swingSide.getOppositeSide()));
      captureRegionCalculator.calculateCaptureRegion(swingSide, swingTimeRemaining, capturePoint2d, omega0, footPolygon);
   }

   public FrameConvexPolygon2d getCaptureRegion()
   {
      return captureRegionCalculator.getCaptureRegion();
   }

   /**
    * Return null if the robot is not falling.
    * If the robot is falling, it returns the suggested swingSide to recover.
    */
   public RobotSide isRobotFallingFromDoubleSupport()
   {
      return swingSideForDoubleSupportRecovery.getEnumValue();
   }

   public void initializeParametersForDoubleSupportPushRecovery()
   {
      recoveringFromDoubleSupportFall.set(true);
   }

   public void updateForDoubleSupport(FramePoint2D desiredCapturePoint2d, FramePoint2D capturePoint2d, double omega0)
   {
      if (!isEnabled())
         return;

      this.omega0 = omega0;
      this.capturePoint2d.setIncludingFrame(capturePoint2d);
      this.desiredCapturePoint2d.setIncludingFrame(desiredCapturePoint2d);
      FrameConvexPolygon2d supportPolygonInMidFeetZUp = bipedSupportPolygon.getSupportPolygonInMidFeetZUp();

      // Initialize variables
      closestFootToICP.set(null);
      swingSideForDoubleSupportRecovery.set(null);

      for (RobotSide robotSide : RobotSide.values)
         distanceICPToFeet.get(robotSide).set(Double.NaN);

      this.capturePoint2d.changeFrame(midFeetZUp);
      this.desiredCapturePoint2d.changeFrame(midFeetZUp);

      isICPErrorTooLarge.set(this.desiredCapturePoint2d.distance(this.capturePoint2d) > icpErrorThreshold.getDoubleValue());

      isICPOutside.set(!supportPolygonInMidFeetZUp.isPointInside(this.capturePoint2d));

      if (!isICPOutside.getBooleanValue() || !isICPErrorTooLarge.getBooleanValue())
      {
         isRobotBackToSafeState.update(true);
         return;
      }

      projectedCapturePoint.setIncludingFrame(this.capturePoint2d, 0.0);

      for (RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame soleFrame = soleFrames.get(robotSide);
         projectedCapturePoint.changeFrame(soleFrame);
         footPolygon.setIncludingFrameAndUpdate(bipedSupportPolygon.getFootPolygonInSoleFrame(robotSide));
         projectedCapturePoint2d.setIncludingFrame(projectedCapturePoint);

         distanceICPToFeet.get(robotSide).set(projectedCapturePoint2d.distance(footPolygon.getCentroid()));
         isRobotBackToSafeState.set(false);
      }

      boolean isLeftFootCloser = distanceICPToFeet.get(RobotSide.LEFT).getDoubleValue() <= distanceICPToFeet.get(RobotSide.RIGHT).getDoubleValue();
      closestFootToICP.set(isLeftFootCloser ? RobotSide.LEFT : RobotSide.RIGHT);
      swingSideForDoubleSupportRecovery.set(closestFootToICP.getEnumValue().getOppositeSide());
   }

   public void updateForSingleSupport(FramePoint2D desiredCapturePoint2d, FramePoint2D capturePoint2d, double omega0)
   {
      if (!isEnabled())
         return;

      this.omega0 = omega0;
      this.capturePoint2d.setIncludingFrame(capturePoint2d);
      this.desiredCapturePoint2d.setIncludingFrame(desiredCapturePoint2d);

      isICPErrorTooLarge.set(this.desiredCapturePoint2d.distance(this.capturePoint2d) > icpErrorThreshold.getDoubleValue());
   }

   public double computePreferredSwingTimeForRecovering(double swingTimeRemaining, RobotSide swingSide)
   {
      RobotSide supportSide = swingSide.getOppositeSide();
      double preferredSwingTime = swingTimeRemaining;
      footPolygon.setIncludingFrameAndUpdate(bipedSupportPolygon.getFootPolygonInAnkleZUp(supportSide));
      captureRegionCalculator.calculateCaptureRegion(swingSide, preferredSwingTime, capturePoint2d, omega0, footPolygon);
      double captureRegionArea = captureRegionCalculator.getCaptureRegionArea();

      // If there is no capture region for the given swing time we reduce it.
      for (; preferredSwingTime >= 0.0; preferredSwingTime -= swingTimeRemaining / 10.0)
      {
         captureRegionCalculator.calculateCaptureRegion(swingSide, preferredSwingTime, capturePoint2d, omega0, footPolygon);

         captureRegionArea = captureRegionCalculator.getCaptureRegionArea();

         if (!Double.isNaN(captureRegionArea))
            break;
      }

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

      double preferredSwingTimeForRecovering = computePreferredSwingTimeForRecovering(swingTimeRemaining, swingSide);
      captureRegionCalculator.calculateCaptureRegion(swingSide, preferredSwingTimeForRecovering, capturePoint2d, omega0, footPolygon);

      if (!isICPErrorTooLarge.getBooleanValue())
      {
         isRobotBackToSafeState.update(true);
         return false;
      }

      if (swingTimeRemaining < MINIMUM_TIME_TO_REPLAN)
      {
         // do not re-plan if we are almost at touch-down
         return false;
      }

      FramePoint2D footCentroid = footPolygon.getCentroid();
      FrameConvexPolygon2d captureRegion = captureRegionCalculator.getCaptureRegion();
      isCaptureRegionEmpty.set(captureRegion.isEmpty());
      if (!recovering.getBooleanValue())
      {
         boolean hasFootstepBeenAdjusted = footstepAdjustor.adjustFootstep(nextFootstep, footCentroid, captureRegion);
         footstepWasProjectedInCaptureRegion.set(hasFootstepBeenAdjusted);
      }
      else
      {
         footstepWasProjectedInCaptureRegion.set(false);
      }

      if (footstepWasProjectedInCaptureRegion.getBooleanValue())
      {
         isRobotBackToSafeState.set(false);
         recovering.set(true);
      }

      return footstepWasProjectedInCaptureRegion.getBooleanValue();
   }

   public void packFootstepForRecoveringFromDisturbance(RobotSide swingSide, double swingTimeRemaining, Footstep footstepToPack)
   {
      footstepToPack.clear();

      if (!enablePushRecovery.getBooleanValue())
         return;

      packFootstepAtCurrentLocation(swingSide, footstepToPack);
      checkAndUpdateFootstep(swingTimeRemaining, footstepToPack);
   }

   public void reset()
   {
      footstepWasProjectedInCaptureRegion.set(false);
      recovering.set(false);
      captureRegionCalculator.hideCaptureRegion();

      recoveringFromDoubleSupportFall.set(false);
   }

   private void packFootstepAtCurrentLocation(RobotSide robotSide, Footstep footstepToPack)
   {
      footstepToPack.setRobotSide(robotSide);
      footstepToPack.setTrustHeight(true);
      footstepToPack.getFootstepPose().setToZero(soleFrames.get(robotSide));
      footstepToPack.getFootstepPose().changeFrame(worldFrame);
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
      return isEnabled() && recovering.getBooleanValue();
   }

   public boolean isRobotBackToSafeState()
   {
      return isRobotBackToSafeState.getBooleanValue();
   }

   public boolean isCaptureRegionEmpty()
   {
      return isCaptureRegionEmpty.getBooleanValue();
   }
}
