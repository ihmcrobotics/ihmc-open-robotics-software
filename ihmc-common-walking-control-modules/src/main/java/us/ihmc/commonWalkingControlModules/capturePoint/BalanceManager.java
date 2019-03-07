package us.ihmc.commonWalkingControlModules.capturePoint;

import static us.ihmc.graphicsDescription.appearance.YoAppearance.Beige;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.Black;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.BlueViolet;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.DarkViolet;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.Yellow;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.SmoothCMPBasedICPPlanner;
import us.ihmc.commonWalkingControlModules.captureRegion.PushRecoveryControlModule;
import us.ihmc.commonWalkingControlModules.configurations.ICPAngularMomentumModifierParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.PelvisICPBasedTranslationManager;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.dynamicReachability.DynamicReachabilityCalculator;
import us.ihmc.commonWalkingControlModules.messageHandlers.CenterOfMassTrajectoryHandler;
import us.ihmc.commonWalkingControlModules.messageHandlers.MomentumTrajectoryHandler;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector2D;

public class BalanceManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final boolean ENABLE_DYN_REACHABILITY = true;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BipedSupportPolygons bipedSupportPolygons;
   private final ICPPlannerWithAngularMomentumOffsetInterface icpPlanner;
   private final MomentumTrajectoryHandler momentumTrajectoryHandler;
   private final PrecomputedICPPlanner precomputedICPPlanner;
   private final LinearMomentumRateControlModule linearMomentumRateOfChangeControlModule;
   private final DynamicReachabilityCalculator dynamicReachabilityCalculator;
   private boolean updateTimings = false;

   private final PelvisICPBasedTranslationManager pelvisICPBasedTranslationManager;
   private final PushRecoveryControlModule pushRecoveryControlModule;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   private final YoFramePoint3D yoCenterOfMass = new YoFramePoint3D("centerOfMass", worldFrame, registry);
   private final YoFramePoint2D yoDesiredCapturePoint = new YoFramePoint2D("desiredICP", worldFrame, registry);
   private final YoFrameVector2D yoDesiredICPVelocity = new YoFrameVector2D("desiredICPVelocity", worldFrame, registry);
   private final YoFramePoint2D yoFinalDesiredICP = new YoFramePoint2D("finalDesiredICP", worldFrame, registry);

   /** CoP position according to the ICP planner */
   private final YoFramePoint2D yoPerfectCoP = new YoFramePoint2D("perfectCoP", worldFrame, registry);
   /** CMP position according to the ICP planner */
   private final YoFramePoint2D yoPerfectCMP = new YoFramePoint2D("perfectCMP", worldFrame, registry);

   private final YoBoolean editStepTimingForReachability = new YoBoolean("editStepTimingForReachability", registry);

   private final YoDouble yoTime;

   private final ReferenceFrame centerOfMassFrame;

   private final FramePoint3D centerOfMassPosition = new FramePoint3D();
   private final FramePoint2D centerOfMassPosition2d = new FramePoint2D();

   private final FramePoint2D capturePoint2d = new FramePoint2D();
   private final FrameVector2D capturePointVelocity2d = new FrameVector2D();
   private final FramePoint3D tempCapturePoint = new FramePoint3D();
   private final FramePoint2D desiredCapturePoint2d = new FramePoint2D();
   private final FrameVector2D desiredCapturePointVelocity2d = new FrameVector2D();
   private final FramePoint2D perfectCoP2d = new FramePoint2D();
   private final FramePoint2D finalDesiredCapturePoint2d = new FramePoint2D();

   private final YoBoolean blendICPTrajectories = new YoBoolean("blendICPTrajectories", registry);

   private final FramePoint2D adjustedDesiredCapturePoint2d = new FramePoint2D();
   private final YoFramePoint2D yoAdjustedDesiredCapturePoint = new YoFramePoint2D("adjustedDesiredICP", worldFrame, registry);

   private final FrameVector2D icpError2d = new FrameVector2D();

   private final ConvexPolygonScaler convexPolygonShrinker = new ConvexPolygonScaler();
   private final FrameConvexPolygon2D shrunkSupportPolygon = new FrameConvexPolygon2D();

   private final YoDouble safeDistanceFromSupportEdgesToStopCancelICPPlan = new YoDouble("safeDistanceFromSupportEdgesToStopCancelICPPlan", registry);
   private final YoDouble distanceToShrinkSupportPolygonWhenHoldingCurrent = new YoDouble("distanceToShrinkSupportPolygonWhenHoldingCurrent", registry);

   private final YoBoolean holdICPToCurrentCoMLocationInNextDoubleSupport = new YoBoolean("holdICPToCurrentCoMLocationInNextDoubleSupport", registry);
   private final YoBoolean controlHeightWithMomentum = new YoBoolean("controlHeightWithMomentum", registry);

   private final YoDouble normalizedICPError = new YoDouble("normalizedICPError", registry);
   private final DoubleProvider maxICPErrorBeforeSingleSupportX;
   private final DoubleProvider maxICPErrorBeforeSingleSupportY;

   private final CapturabilityBasedStatus capturabilityBasedStatus = new CapturabilityBasedStatus();

   private final BooleanParameter useCoPObjective = new BooleanParameter("UseCenterOfPressureObjectiveFromPlanner", registry, false);

   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();

   private final SmoothCMPBasedICPPlanner smoothCMPPlanner;
   private final YoBoolean icpPlannerDone = new YoBoolean("ICPPlannerDone", registry);

   private boolean initializeForStanding = false;
   private boolean initializeForSingleSupport = false;
   private boolean initializeForTransfer = false;
   private boolean footstepWasAdjusted = false;
   private boolean usingStepAdjustment = false;
   private final FixedFramePose3DBasics footstepSolution = new FramePose3D();

   public BalanceManager(HighLevelHumanoidControllerToolbox controllerToolbox, WalkingControllerParameters walkingControllerParameters,
                         ICPWithTimeFreezingPlannerParameters icpPlannerParameters, ICPAngularMomentumModifierParameters angularMomentumModifierParameters,
                         Vector3DReadOnly angularMomentumRateWeight, Vector3DReadOnly linearMomentumRateWeight, YoVariableRegistry parentRegistry)
   {
      CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();

      YoGraphicsListRegistry yoGraphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();
      SideDependentList<ContactableFoot> contactableFeet = controllerToolbox.getContactableFeet();

      double controlDT = controllerToolbox.getControlDT();
      double gravityZ = controllerToolbox.getGravityZ();
      double totalMass = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator());

      this.controllerToolbox = controllerToolbox;
      yoTime = controllerToolbox.getYoTime();

      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();

      bipedSupportPolygons = controllerToolbox.getBipedSupportPolygons();

      linearMomentumRateOfChangeControlModule = new LinearMomentumRateControlModule(referenceFrames, bipedSupportPolygons,
                                                                                                           controllerToolbox.getICPControlPolygons(),
                                                                                                           contactableFeet, walkingControllerParameters, yoTime,
                                                                                                           totalMass, gravityZ, controlDT,
                                                                                                           angularMomentumRateWeight, linearMomentumRateWeight,
                                                                                                           registry, yoGraphicsListRegistry);

      WalkingMessageHandler walkingMessageHandler = controllerToolbox.getWalkingMessageHandler();
      ICPPlannerInterface icpPlanner;
      //icpPlanner = new ICPPlannerWithTimeFreezer(bipedSupportPolygons, contactableFeet, icpPlannerParameters, registry, yoGraphicsListRegistry);
      if (!icpPlannerParameters.useSmoothCMPPlanner())
      {
         icpPlanner = new ContinuousCMPBasedICPPlanner(bipedSupportPolygons, contactableFeet, icpPlannerParameters.getNumberOfFootstepsToConsider(),
                                                                           registry, yoGraphicsListRegistry);
         smoothCMPPlanner = null;
      }
      else
      {
         MomentumTrajectoryHandler momentumTrajectoryHandler = walkingMessageHandler == null ? null : walkingMessageHandler.getMomentumTrajectoryHandler();
         smoothCMPPlanner = new SmoothCMPBasedICPPlanner(fullRobotModel, bipedSupportPolygons, contactableFeet, icpPlannerParameters.getNumberOfFootstepsToConsider(),
                                                                                  momentumTrajectoryHandler, yoTime, registry, yoGraphicsListRegistry, controllerToolbox.getGravityZ());
         smoothCMPPlanner.setDefaultPhaseTimes(walkingControllerParameters.getDefaultSwingTime(), walkingControllerParameters.getDefaultTransferTime());
         icpPlanner = smoothCMPPlanner;
      }

      ICPPlannerWithAngularMomentumOffsetWrapper icpWrapper = new ICPPlannerWithAngularMomentumOffsetWrapper(icpPlanner, bipedSupportPolygons.getSoleZUpFrames());
      parentRegistry.addChild(icpWrapper.getYoVariableRegistry());

      this.icpPlanner = icpWrapper;
      this.icpPlanner.initializeParameters(icpPlannerParameters, angularMomentumModifierParameters);
      this.icpPlanner.setOmega0(controllerToolbox.getOmega0());
      this.icpPlanner.setFinalTransferDuration(walkingControllerParameters.getDefaultTransferTime());

      if (walkingMessageHandler != null)
      {
         CenterOfMassTrajectoryHandler comTrajectoryHandler = walkingMessageHandler.getComTrajectoryHandler();
         double dt = controllerToolbox.getControlDT();
         momentumTrajectoryHandler = walkingMessageHandler.getMomentumTrajectoryHandler();
         precomputedICPPlanner = new PrecomputedICPPlanner(dt, comTrajectoryHandler, momentumTrajectoryHandler, registry, yoGraphicsListRegistry);
         precomputedICPPlanner.setOmega0(controllerToolbox.getOmega0());
         precomputedICPPlanner.setMass(totalMass);
         precomputedICPPlanner.setGravity(gravityZ);
      }
      else
      {
         momentumTrajectoryHandler = null;
         precomputedICPPlanner = null;
      }
      blendICPTrajectories.set(true);

      if (ENABLE_DYN_REACHABILITY)
      {
         dynamicReachabilityCalculator = new DynamicReachabilityCalculator(icpPlanner, fullRobotModel, centerOfMassFrame,
                                                                           walkingControllerParameters.getDynamicReachabilityParameters(), registry,
                                                                           yoGraphicsListRegistry);
      }
      else
      {
         dynamicReachabilityCalculator = null;
      }
      editStepTimingForReachability.set(walkingControllerParameters.editStepTimingForReachability());

      safeDistanceFromSupportEdgesToStopCancelICPPlan.set(0.05);
      distanceToShrinkSupportPolygonWhenHoldingCurrent.set(0.08);

      maxICPErrorBeforeSingleSupportX = new DoubleParameter("maxICPErrorBeforeSingleSupportX", registry, walkingControllerParameters.getMaxICPErrorBeforeSingleSupportX());
      maxICPErrorBeforeSingleSupportY = new DoubleParameter("maxICPErrorBeforeSingleSupportY", registry, walkingControllerParameters.getMaxICPErrorBeforeSingleSupportY());

      double pelvisTranslationICPSupportPolygonSafeMargin = walkingControllerParameters.getPelvisTranslationICPSupportPolygonSafeMargin();
      pelvisICPBasedTranslationManager = new PelvisICPBasedTranslationManager(controllerToolbox, pelvisTranslationICPSupportPolygonSafeMargin, bipedSupportPolygons, registry);


      pushRecoveryControlModule = new PushRecoveryControlModule(bipedSupportPolygons, controllerToolbox, walkingControllerParameters, registry);

      controlHeightWithMomentum.set(walkingControllerParameters.controlHeightWithMomentum());

      String graphicListName = getClass().getSimpleName();

      if (yoGraphicsListRegistry != null)
      {
         YoGraphicPosition centerOfMassViz = new YoGraphicPosition("Center Of Mass", yoCenterOfMass, 0.006, Black(), GraphicType.BALL_WITH_CROSS);
         YoGraphicPosition desiredCapturePointViz = new YoGraphicPosition("Desired Capture Point", yoDesiredCapturePoint, 0.01, Yellow(), GraphicType.BALL_WITH_ROTATED_CROSS);
         YoGraphicPosition finalDesiredCapturePointViz = new YoGraphicPosition("Final Desired Capture Point", yoFinalDesiredICP, 0.01, Beige(), GraphicType.BALL_WITH_ROTATED_CROSS);
         YoGraphicPosition perfectCMPViz = new YoGraphicPosition("Perfect CMP", yoPerfectCMP, 0.002, BlueViolet());
         YoGraphicPosition perfectCoPViz = new YoGraphicPosition("Perfect CoP", yoPerfectCoP, 0.002, DarkViolet(), GraphicType.BALL_WITH_CROSS);

         YoGraphicPosition adjustedDesiredCapturePointViz = new YoGraphicPosition("Adjusted Desired Capture Point", yoAdjustedDesiredCapturePoint, 0.005, Yellow(), GraphicType.DIAMOND);
         yoGraphicsListRegistry.registerArtifact(graphicListName, adjustedDesiredCapturePointViz.createArtifact());

         yoGraphicsListRegistry.registerArtifact(graphicListName, centerOfMassViz.createArtifact());
         yoGraphicsListRegistry.registerArtifact(graphicListName, desiredCapturePointViz.createArtifact());
         yoGraphicsListRegistry.registerArtifact(graphicListName, finalDesiredCapturePointViz.createArtifact());
         YoArtifactPosition perfectCMPArtifact = perfectCMPViz.createArtifact();
         perfectCMPArtifact.setVisible(false);
         yoGraphicsListRegistry.registerArtifact(graphicListName, perfectCMPArtifact);
         YoArtifactPosition perfectCoPArtifact = perfectCoPViz.createArtifact();
         perfectCoPArtifact.setVisible(false);
         yoGraphicsListRegistry.registerArtifact(graphicListName, perfectCoPArtifact);
      }
      yoCenterOfMass.setToNaN();
      yoDesiredCapturePoint.setToNaN();
      yoFinalDesiredICP.setToNaN();
      yoPerfectCMP.setToNaN();
      yoPerfectCoP.setToNaN();

      parentRegistry.addChild(registry);
   }

   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing)
   {
      icpPlanner.addFootstepToPlan(footstep, timing);
      linearMomentumRateOfChangeControlModule.addFootstepToPlan(footstep, timing);
   }

   /**
    * Sets the next footstep that the robot will take. Should be set at the beginning of transfer.
    * @param upcomingFootstep
    */
   public void setUpcomingFootstep(Footstep upcomingFootstep)
   {
      if (ENABLE_DYN_REACHABILITY)
         dynamicReachabilityCalculator.setUpcomingFootstep(upcomingFootstep);
   }

   /**
    * Sets the next footstep that the robot will take. Should be set at the beginning of swing. Modifies the momentum recovery control module, which checks
    * the stability of the robot.
    * @param nextFootstep
    */
   public void setNextFootstep(Footstep nextFootstep)
   {
      if (ENABLE_DYN_REACHABILITY)
         dynamicReachabilityCalculator.setUpcomingFootstep(nextFootstep);
   }

   public boolean wasTimingAdjustedForReachability()
   {
      if (ENABLE_DYN_REACHABILITY)
         return dynamicReachabilityCalculator.wasTimingAdjusted();
      else
         return false;
   }

   public double getCurrentTransferDurationAdjustedForReachability()
   {
      return icpPlanner.getTransferDuration(0);
   }

   public double getCurrentSwingDurationAdjustedForReachability()
   {
      return icpPlanner.getSwingDuration(0);
   }

   public double getNextTransferDurationAdjustedForReachability()
   {
      return icpPlanner.getTransferDuration(1);
   }

   public double getCurrentTouchdownDuration()
   {
      return icpPlanner.getTouchdownDuration(0);
   }

   public boolean checkAndUpdateFootstep(Footstep footstep)
   {
      return pushRecoveryControlModule.checkAndUpdateFootstep(getTimeRemainingInCurrentState(), footstep);
   }

   public boolean checkAndUpdateFootstepFromICPOptimization(Footstep footstep)
   {
      if (!usingStepAdjustment || initializeForSingleSupport || initializeForTransfer || initializeForStanding)
      {
         return false;
      }
      footstep.setPose(footstepSolution);
      return footstepWasAdjusted;
   }

   public void clearICPPlan()
   {
      icpPlanner.clearPlan();
   }

   public void setICPPlanSupportSide(RobotSide robotSide)
   {
      icpPlanner.setSupportLeg(robotSide);
      linearMomentumRateOfChangeControlModule.setSupportLeg(robotSide);
   }

   public void setICPPlanTransferToSide(RobotSide robotSide)
   {
      icpPlanner.setTransferToSide(robotSide);
      linearMomentumRateOfChangeControlModule.setTransferToSide(robotSide);
   }

   public void setICPPlanTransferFromSide(RobotSide robotSide)
   {
      icpPlanner.setTransferFromSide(robotSide);
      if (robotSide != null)
      {
         linearMomentumRateOfChangeControlModule.setTransferToSide(robotSide.getOppositeSide());
      }
   }

   public void endTick()
   {
      if (smoothCMPPlanner != null)
      {
         smoothCMPPlanner.endTick();
      }
   }

   private final FramePoint3D copEstimate = new FramePoint3D();
   private final FrameVector2D emptyVector = new FrameVector2D();
   public void compute(RobotSide supportLeg, double desiredCoMHeightAcceleration, boolean keepCMPInsideSupportPolygon, boolean controlHeightWithMomentum)
   {
      controllerToolbox.getCapturePointVelocity(capturePointVelocity2d);
      controllerToolbox.getCoP(copEstimate);

      if (icpPlanner instanceof ICPPlannerWithAngularMomentumOffsetInterface)
         icpPlanner.modifyDesiredICPForAngularMomentum(copEstimate, supportLeg);

      icpPlanner.getDesiredCapturePointPosition(desiredCapturePoint2d);
      icpPlanner.getDesiredCapturePointVelocity(desiredCapturePointVelocity2d);
      icpPlanner.getDesiredCenterOfPressurePosition(perfectCoP2d);

      pelvisICPBasedTranslationManager.compute(supportLeg, capturePoint2d);
      pelvisICPBasedTranslationManager.addICPOffset(desiredCapturePoint2d, desiredCapturePointVelocity2d, perfectCoP2d);

      double omega0 = controllerToolbox.getOmega0();
      if (Double.isNaN(omega0))
         throw new RuntimeException("omega0 is NaN");

      if (supportLeg == null)
         pushRecoveryControlModule.updateForDoubleSupport(desiredCapturePoint2d, capturePoint2d, omega0);
      else
         pushRecoveryControlModule.updateForSingleSupport(desiredCapturePoint2d, capturePoint2d, omega0);

      // --- compute adjusted desired capture point
      controllerToolbox.getAdjustedDesiredCapturePoint(desiredCapturePoint2d, adjustedDesiredCapturePoint2d);
      yoAdjustedDesiredCapturePoint.set(adjustedDesiredCapturePoint2d);
      desiredCapturePoint2d.setIncludingFrame(adjustedDesiredCapturePoint2d);
      // ---

      if (precomputedICPPlanner != null)
      {
         precomputedICPPlanner.setCoMZAcceleration(desiredCoMHeightAcceleration);
         if (blendICPTrajectories.getBooleanValue())
         {
            precomputedICPPlanner.computeAndBlend(yoTime.getDoubleValue(), desiredCapturePoint2d, desiredCapturePointVelocity2d, perfectCoP2d);
         }
         else
         {
            precomputedICPPlanner.compute(yoTime.getDoubleValue(), desiredCapturePoint2d, desiredCapturePointVelocity2d, perfectCoP2d);
         }
      }

      yoDesiredCapturePoint.set(desiredCapturePoint2d);
      yoDesiredICPVelocity.set(desiredCapturePointVelocity2d);
      yoPerfectCoP.set(perfectCoP2d);

      finalDesiredCapturePoint2d.setIncludingFrame(yoFinalDesiredICP);

      getICPError(icpError2d);

      if (initializeForStanding)
      {
         linearMomentumRateOfChangeControlModule.initializeForStanding();
         initializeForStanding = false;
      }
      if (initializeForTransfer)
      {
         linearMomentumRateOfChangeControlModule.initializeForTransfer();
         initializeForTransfer = false;
      }
      if (initializeForSingleSupport)
      {
         linearMomentumRateOfChangeControlModule.initializeForSingleSupport();
         initializeForSingleSupport = false;
      }

      if (ENABLE_DYN_REACHABILITY && updateTimings)
      {
         double adjustedFinalTransferDuration = dynamicReachabilityCalculator.getFinalTransferDuration();
         if (!Double.isNaN(adjustedFinalTransferDuration))
         {
            linearMomentumRateOfChangeControlModule.setFinalTransferDuration(adjustedFinalTransferDuration);
         }

         double nextTransferDuration = dynamicReachabilityCalculator.getNextTransferDuration();
         if (!Double.isNaN(nextTransferDuration))
         {
            linearMomentumRateOfChangeControlModule.setNextTransferDuration(nextTransferDuration);
         }

         linearMomentumRateOfChangeControlModule.setTransferDuration(dynamicReachabilityCalculator.getTransferDuration());
         linearMomentumRateOfChangeControlModule.setSwingDuration(dynamicReachabilityCalculator.getSwingDuration());

         updateTimings = false;
      }

      CapturePointTools.computeDesiredCentroidalMomentumPivot(desiredCapturePoint2d, desiredCapturePointVelocity2d, omega0, yoPerfectCMP);
      linearMomentumRateOfChangeControlModule.setKeepCoPInsideSupportPolygon(keepCMPInsideSupportPolygon);
      linearMomentumRateOfChangeControlModule.setControlHeightWithMomentum(this.controlHeightWithMomentum.getBooleanValue() && controlHeightWithMomentum);
      linearMomentumRateOfChangeControlModule.setDesiredCenterOfMassHeightAcceleration(desiredCoMHeightAcceleration);
      linearMomentumRateOfChangeControlModule.setCapturePoint(capturePoint2d, capturePointVelocity2d);
      linearMomentumRateOfChangeControlModule.setOmega0(omega0);
      linearMomentumRateOfChangeControlModule.setDesiredCapturePoint(desiredCapturePoint2d, desiredCapturePointVelocity2d);
      linearMomentumRateOfChangeControlModule.setFinalDesiredCapturePoint(finalDesiredCapturePoint2d);
      linearMomentumRateOfChangeControlModule.setPerfectCMP(yoPerfectCMP);
      linearMomentumRateOfChangeControlModule.setPerfectCoP(yoPerfectCoP);
      linearMomentumRateOfChangeControlModule.setSupportLeg(supportLeg);
      boolean success = linearMomentumRateOfChangeControlModule.compute();

      footstepSolution.set(linearMomentumRateOfChangeControlModule.getFootstepSolution());
      footstepWasAdjusted = linearMomentumRateOfChangeControlModule.getFootstepWasAdjusted();
      usingStepAdjustment = linearMomentumRateOfChangeControlModule.getUsingStepAdjustment();

      if (!success)
      {
         controllerToolbox.reportControllerFailureToListeners(emptyVector);
      }

      // This is for debugging such that the momentum trajectory handler YoVariables contain the current value:
      if (momentumTrajectoryHandler != null)
      {
         momentumTrajectoryHandler.packDesiredAngularMomentumAtTime(yoTime.getValue(), null, null);
      }
   }

   public void computeICPPlan()
   {
      controllerToolbox.getCapturePoint(capturePoint2d);
      controllerToolbox.getCoP(copEstimate);
      icpPlanner.compute(capturePoint2d, yoTime.getDoubleValue());
      icpPlannerDone.set(icpPlanner.isDone());
   }

   public void packFootstepForRecoveringFromDisturbance(RobotSide swingSide, double swingTimeRemaining, Footstep footstepToPack)
   {
      pushRecoveryControlModule.packFootstepForRecoveringFromDisturbance(swingSide, swingTimeRemaining, footstepToPack);
   }

   public void disablePelvisXYControl()
   {
      pelvisICPBasedTranslationManager.disable();
   }

   public void disablePushRecovery()
   {
      pushRecoveryControlModule.setIsEnabled(false);
   }

   public void enablePelvisXYControl()
   {
      pelvisICPBasedTranslationManager.enable();
   }

   public void enablePushRecovery()
   {
      pushRecoveryControlModule.setIsEnabled(true);
   }

   public double estimateTimeRemainingForSwingUnderDisturbance()
   {
      controllerToolbox.getCapturePoint(capturePoint2d);

      return icpPlanner.estimateTimeRemainingForStateUnderDisturbance(capturePoint2d);
   }

   public void freezePelvisXYControl()
   {
      pelvisICPBasedTranslationManager.freeze();
   }

   public void getDesiredCMP(FramePoint2D desiredCMPToPack)
   {
      desiredCMPToPack.setIncludingFrame(linearMomentumRateOfChangeControlModule.getDesiredCMP());
   }

   public void getDesiredICP(FramePoint2D desiredICPToPack)
   {
      desiredICPToPack.setIncludingFrame(yoDesiredCapturePoint);
   }

   public void getDesiredICPVelocity(FrameVector2D desiredICPVelocityToPack)
   {
      desiredICPVelocityToPack.setIncludingFrame(yoDesiredICPVelocity);
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      inverseDynamicsCommandList.clear();
      inverseDynamicsCommandList.addCommand(linearMomentumRateOfChangeControlModule.getMomentumRateCommand());
      if (useCoPObjective.getValue())
      {
         inverseDynamicsCommandList.addCommand(linearMomentumRateOfChangeControlModule.getCenterOfPressureCommand());
      }
      return inverseDynamicsCommandList;
   }

   public void getNextExitCMP(FramePoint3D entryCMPToPack)
   {
      icpPlanner.getNextExitCMP(entryCMPToPack);
   }

   public double getTimeRemainingInCurrentState()
   {
      return icpPlanner.getTimeInCurrentStateRemaining();
   }

   public void goHome()
   {
      if (pelvisICPBasedTranslationManager.isEnabled())
         pelvisICPBasedTranslationManager.goToHome();
   }

   public void handlePelvisTrajectoryCommand(PelvisTrajectoryCommand command)
   {
      pelvisICPBasedTranslationManager.handlePelvisTrajectoryCommand(command);
   }

   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      pelvisICPBasedTranslationManager.handleStopAllTrajectoryCommand(command);
   }

   public void initialize()
   {
      yoFinalDesiredICP.set(Double.NaN, Double.NaN);
      controllerToolbox.getCapturePoint(tempCapturePoint);
      yoDesiredCapturePoint.set(tempCapturePoint);

      icpPlanner.holdCurrentICP(tempCapturePoint);
      icpPlanner.initializeForStanding(yoTime.getDoubleValue());

      initializeForStanding = true;
   }

   public void prepareForDoubleSupportPushRecovery()
   {
      pushRecoveryControlModule.initializeParametersForDoubleSupportPushRecovery();
   }

   public void initializeICPPlanForSingleSupport(double swingTime, double transferTime, double finalTransferTime)
   {
      setFinalTransferTime(finalTransferTime);
      icpPlanner.initializeForSingleSupport(yoTime.getDoubleValue());
      initializeForSingleSupport = true;

      if (Double.isFinite(swingTime) && Double.isFinite(transferTime) && ENABLE_DYN_REACHABILITY)
      {
         dynamicReachabilityCalculator.setInSwing();

         if (editStepTimingForReachability.getBooleanValue())
         {
            dynamicReachabilityCalculator.verifyAndEnsureReachability();
            updateTimings = dynamicReachabilityCalculator.wasTimingAdjusted();
         }
         else
         {
            dynamicReachabilityCalculator.checkReachabilityOfStep();
         }
      }

      icpPlannerDone.set(false);
   }

   public void initializeICPPlanForStanding()
   {
      if (holdICPToCurrentCoMLocationInNextDoubleSupport.getBooleanValue())
      {
         requestICPPlannerToHoldCurrentCoM();
         holdICPToCurrentCoMLocationInNextDoubleSupport.set(false);
      }
      icpPlanner.initializeForStanding(yoTime.getDoubleValue());
      initializeForStanding = true;

      icpPlannerDone.set(false);
   }

   public void initializeICPPlanForTransferToStanding(double finalTransferTime)
   {
      if (holdICPToCurrentCoMLocationInNextDoubleSupport.getBooleanValue())
      {
         requestICPPlannerToHoldCurrentCoM();
         holdICPToCurrentCoMLocationInNextDoubleSupport.set(false);
      }
      setFinalTransferTime(finalTransferTime);
      icpPlanner.initializeForTransfer(yoTime.getDoubleValue());
      initializeForStanding = true;

      icpPlannerDone.set(false);
   }

   public void initializeICPPlanForTransfer(double swingTime, double transferTime, double finalTransferTime)
   {
      if (holdICPToCurrentCoMLocationInNextDoubleSupport.getBooleanValue())
      {
         requestICPPlannerToHoldCurrentCoM();
         holdICPToCurrentCoMLocationInNextDoubleSupport.set(false);
      }
      setFinalTransferTime(finalTransferTime);
      icpPlanner.initializeForTransfer(yoTime.getDoubleValue());

      initializeForTransfer = true;

      if (Double.isFinite(swingTime) && Double.isFinite(transferTime) && ENABLE_DYN_REACHABILITY)
      {
         dynamicReachabilityCalculator.setInTransfer();

         if (editStepTimingForReachability.getBooleanValue())
         {
            dynamicReachabilityCalculator.verifyAndEnsureReachability();
            updateTimings = dynamicReachabilityCalculator.wasTimingAdjusted();
         }
         else
         {
            dynamicReachabilityCalculator.checkReachabilityOfStep();
         }
      }

      icpPlannerDone.set(false);
   }

   public void computeNormalizedEllipticICPError(RobotSide transferToSide)
   {
      getICPError(icpError2d);
      ReferenceFrame leadingSoleZUpFrame = bipedSupportPolygons.getSoleZUpFrames().get(transferToSide);
      icpError2d.changeFrame(leadingSoleZUpFrame);
      normalizedICPError.set(MathTools.square(icpError2d.getX() / maxICPErrorBeforeSingleSupportX.getValue())
            + MathTools.square(icpError2d.getY() / maxICPErrorBeforeSingleSupportY.getValue()));
   }

   public double getNormalizedEllipticICPError()
   {
      return normalizedICPError.getValue();
   }

   public double getICPErrorMagnitude()
   {
      controllerToolbox.getCapturePoint(capturePoint2d);
      return capturePoint2d.distance(yoDesiredCapturePoint);
   }

   public void getICPError(FrameVector2D icpErrorToPack)
   {
      controllerToolbox.getCapturePoint(capturePoint2d);
      desiredCapturePoint2d.setIncludingFrame(yoDesiredCapturePoint);
      icpErrorToPack.setIncludingFrame(desiredCapturePoint2d);
      icpErrorToPack.sub(capturePoint2d);
   }

   public boolean isPrecomputedICPPlannerActive()
   {
      return precomputedICPPlanner.isWithinInterval(yoTime.getDoubleValue());
   }

   public boolean isICPPlanDone()
   {
      return icpPlannerDone.getValue();
   }

   public boolean isPushRecoveryEnabled()
   {
      return pushRecoveryControlModule.isEnabled();
   }

   public boolean isRecovering()
   {
      return pushRecoveryControlModule.isRecovering();
   }

   public boolean isRecoveringFromDoubleSupportFall()
   {
      return pushRecoveryControlModule.isEnabled() && pushRecoveryControlModule.isRecoveringFromDoubleSupportFall();
   }

   public boolean isRecoveryImpossible()
   {
      return pushRecoveryControlModule.isCaptureRegionEmpty();
   }

   public boolean isRobotBackToSafeState()
   {
      return pushRecoveryControlModule.isRobotBackToSafeState();
   }

   public RobotSide isRobotFallingFromDoubleSupport()
   {
      return pushRecoveryControlModule.isRobotFallingFromDoubleSupport();
   }

   public void resetPushRecovery()
   {
      pushRecoveryControlModule.reset();
   }

   public void requestICPPlannerToHoldCurrentCoMInNextDoubleSupport()
   {
      holdICPToCurrentCoMLocationInNextDoubleSupport.set(true);
   }

   public void requestICPPlannerToHoldCurrentCoM()
   {
      centerOfMassPosition.setToZero(centerOfMassFrame);

      FrameConvexPolygon2D supportPolygonInMidFeetZUp = bipedSupportPolygons.getSupportPolygonInMidFeetZUp();
      convexPolygonShrinker.scaleConvexPolygon(supportPolygonInMidFeetZUp, distanceToShrinkSupportPolygonWhenHoldingCurrent.getDoubleValue(), shrunkSupportPolygon);

      centerOfMassPosition.changeFrame(shrunkSupportPolygon.getReferenceFrame());
      centerOfMassPosition2d.setIncludingFrame(centerOfMassPosition);
      shrunkSupportPolygon.orthogonalProjection(centerOfMassPosition2d);
      centerOfMassPosition.set(centerOfMassPosition2d, 0.0);

      centerOfMassPosition.changeFrame(worldFrame);
      icpPlanner.holdCurrentICP(centerOfMassPosition);
   }

   public void setFinalTransferTime(double finalTransferTime)
   {
      icpPlanner.setFinalTransferDuration(finalTransferTime);
      linearMomentumRateOfChangeControlModule.setFinalTransferDuration(finalTransferTime);
   }

   /**
    * Update the basics: capture point, omega0, and the support polygons.
    */
   public void update()
   {
      centerOfMassPosition.setToZero(centerOfMassFrame);
      yoCenterOfMass.setMatchingFrame(centerOfMassPosition);
      computeICPPlan();
      icpPlanner.getFinalDesiredCapturePointPosition(yoFinalDesiredICP);
   }

   public void setAchievedLinearMomentumRate(FrameVector3DReadOnly achievedLinearMomentumRate)
   {
      linearMomentumRateOfChangeControlModule.setAchievedLinearMomentumRate(achievedLinearMomentumRate);
   }

   public CapturabilityBasedStatus updateAndReturnCapturabilityBasedStatus()
   {
      desiredCapturePoint2d.setIncludingFrame(yoDesiredCapturePoint);
      centerOfMassPosition.setToZero(centerOfMassFrame);
      centerOfMassPosition.changeFrame(worldFrame);

      controllerToolbox.getCapturePoint(capturePoint2d);
      capturePoint2d.checkReferenceFrameMatch(worldFrame);
      desiredCapturePoint2d.checkReferenceFrameMatch(worldFrame);

      SideDependentList<FrameConvexPolygon2D> footSupportPolygons = bipedSupportPolygons.getFootPolygonsInWorldFrame();

      capturabilityBasedStatus.getCapturePoint2d().set(capturePoint2d);
      capturabilityBasedStatus.getDesiredCapturePoint2d().set(desiredCapturePoint2d);
      capturabilityBasedStatus.getCenterOfMass3d().set(centerOfMassPosition);
      for (RobotSide robotSide : RobotSide.values)
      {
         HumanoidMessageTools.packFootSupportPolygon(robotSide, footSupportPolygons.get(robotSide), capturabilityBasedStatus);
      }

      return capturabilityBasedStatus;
   }

   public void submitCurrentPlanarRegions(RecyclingArrayList<PlanarRegion> planarRegions)
   {
      linearMomentumRateOfChangeControlModule.submitCurrentPlanarRegions(planarRegions);
   }

   public void updateCurrentICPPlan()
   {
      icpPlanner.updateCurrentPlan();
   }

   public void updateSwingTimeRemaining(double timeRemainingInSwing)
   {
      linearMomentumRateOfChangeControlModule.submitRemainingTimeInSwingUnderDisturbance(timeRemainingInSwing);
   }

   public FrameVector3DReadOnly getEffectiveICPAdjustment()
   {
      return linearMomentumRateOfChangeControlModule.getEffectiveICPAdjustment();
   }

   public void getCapturePoint(FramePoint2D capturePointToPack)
   {
      controllerToolbox.getCapturePoint(capturePointToPack);
   }

   public void minimizeAngularMomentumRateZ(boolean enable)
   {
      linearMomentumRateOfChangeControlModule.setMinimizeAngularMomentumRateZ(enable);
   }
}
