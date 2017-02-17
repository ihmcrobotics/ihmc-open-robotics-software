package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import static us.ihmc.graphicsDescription.appearance.YoAppearance.Beige;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.Black;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.BlueViolet;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.DarkRed;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.Purple;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.Yellow;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.captureRegion.PushRecoveryControlModule;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.PelvisICPBasedTranslationManager;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.GoHomeCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.YoPDGains;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.ConvexPolygonShrinker;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;

public class BalanceManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BipedSupportPolygons bipedSupportPolygons;
   private final ICPPlannerWithTimeFreezer icpPlanner;
   private final LinearMomentumRateOfChangeControlModule linearMomentumRateOfChangeControlModule;

   private final PelvisICPBasedTranslationManager pelvisICPBasedTranslationManager;
   private final PushRecoveryControlModule pushRecoveryControlModule;
   private final MomentumRecoveryControlModule momentumRecoveryControlModule;
   private final HighLevelHumanoidControllerToolbox momentumBasedController;

   private final YoFramePoint yoCenterOfMass = new YoFramePoint("centerOfMass", worldFrame, registry);
   private final YoFramePoint2d yoDesiredCapturePoint = new YoFramePoint2d("desiredICP", worldFrame, registry);
   private final YoFrameVector2d yoDesiredICPVelocity = new YoFrameVector2d("desiredICPVelocity", worldFrame, registry);
   private final YoFramePoint2d yoFinalDesiredICP = new YoFramePoint2d("finalDesiredICP", worldFrame, registry);

   private final YoFramePoint2d yoPerfectCMP = new YoFramePoint2d("perfectCMP", worldFrame, registry);
   private final YoFramePoint2d yoDesiredCMP = new YoFramePoint2d("desiredCMP", worldFrame, registry);
   // TODO It seems that the achieved CMP can be off sometimes.
   // Need to review the computation of the achieved linear momentum rate or of the achieved CMP. (Sylvain)
   private final YoFramePoint2d yoAchievedCMP = new YoFramePoint2d("achievedCMP", worldFrame, registry);

   private final DoubleYoVariable yoTime;

   private final ReferenceFrame centerOfMassFrame;

   private final FramePoint centerOfMassPosition = new FramePoint();
   private final FramePoint2d centerOfMassPosition2d = new FramePoint2d();

   private final FramePoint2d capturePoint2d = new FramePoint2d();
   private final FramePoint tempCapturePoint = new FramePoint();
   private final FramePoint2d desiredCapturePoint2d = new FramePoint2d();
   private final FrameVector2d desiredCapturePointVelocity2d = new FrameVector2d();
   private final FramePoint2d finalDesiredCapturePoint2d = new FramePoint2d();
   /** CMP position according to the ICP planner */
   private final FramePoint2d perfectCMP = new FramePoint2d();

   private final FramePoint2d adjustedDesiredCapturePoint2d = new FramePoint2d();
   private final YoFramePoint2d yoAdjustedDesiredCapturePoint = new YoFramePoint2d("adjustedDesiredICP", worldFrame, registry);

   private final FramePoint2d desiredCMP = new FramePoint2d();
   private final FramePoint2d achievedCMP = new FramePoint2d();

   private final FrameVector2d icpError2d = new FrameVector2d();

   private final ConvexPolygonShrinker convexPolygonShrinker = new ConvexPolygonShrinker();
   private final FrameConvexPolygon2d shrunkSupportPolygon = new FrameConvexPolygon2d();

   private final DoubleYoVariable safeDistanceFromSupportEdgesToStopCancelICPPlan = new DoubleYoVariable("safeDistanceFromSupportEdgesToStopCancelICPPlan", registry);
   private final DoubleYoVariable distanceToShrinkSupportPolygonWhenHoldingCurrent = new DoubleYoVariable("distanceToShrinkSupportPolygonWhenHoldingCurrent", registry);

   private final BooleanYoVariable holdICPToCurrentCoMLocationInNextDoubleSupport = new BooleanYoVariable("holdICPToCurrentCoMLocationInNextDoubleSupport", registry);
   private final BooleanYoVariable controlHeightWithMomentum = new BooleanYoVariable("controlHeightWithMomentum", registry);

   private final DoubleYoVariable maxICPErrorBeforeSingleSupportX = new DoubleYoVariable("maxICPErrorBeforeSingleSupportX", registry);
   private final DoubleYoVariable maxICPErrorBeforeSingleSupportY = new DoubleYoVariable("maxICPErrorBeforeSingleSupportY", registry);

   private final CapturabilityBasedStatus capturabilityBasedStatus = new CapturabilityBasedStatus();

   private final FrameConvexPolygon2d areaToProjectInto = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d safeArea = new FrameConvexPolygon2d();

   private final boolean useICPOptimizationControl;

   public BalanceManager(HighLevelHumanoidControllerToolbox momentumBasedController, WalkingControllerParameters walkingControllerParameters,
         CapturePointPlannerParameters capturePointPlannerParameters, ICPOptimizationParameters icpOptimizationParameters, YoVariableRegistry parentRegistry)
   {
      this(momentumBasedController, walkingControllerParameters, capturePointPlannerParameters, icpOptimizationParameters, parentRegistry, true);
   }

   public BalanceManager(HighLevelHumanoidControllerToolbox momentumBasedController, WalkingControllerParameters walkingControllerParameters,
         CapturePointPlannerParameters capturePointPlannerParameters, ICPOptimizationParameters icpOptimizationParameters, YoVariableRegistry parentRegistry,
         boolean use2DCMPProjection)
   {
      CommonHumanoidReferenceFrames referenceFrames = momentumBasedController.getReferenceFrames();
      FullHumanoidRobotModel fullRobotModel = momentumBasedController.getFullRobotModel();

      YoGraphicsListRegistry yoGraphicsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();
      SideDependentList<? extends ContactablePlaneBody> contactableFeet = momentumBasedController.getContactableFeet();

      ICPControlGains icpControlGains = walkingControllerParameters.createICPControlGains(registry);

      double controlDT = momentumBasedController.getControlDT();
      double gravityZ = momentumBasedController.getGravityZ();
      double totalMass = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator());
      double minimumSwingTimeForDisturbanceRecovery = walkingControllerParameters.getMinimumSwingTimeForDisturbanceRecovery();

      this.momentumBasedController = momentumBasedController;
      yoTime = momentumBasedController.getYoTime();

      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();

      bipedSupportPolygons = momentumBasedController.getBipedSupportPolygons();

      useICPOptimizationControl = walkingControllerParameters.useOptimizationBasedICPController() && (icpOptimizationParameters != null);

      if (useICPOptimizationControl)
      {
         linearMomentumRateOfChangeControlModule = new ICPOptimizationLinearMomentumRateOfChangeControlModule(referenceFrames, bipedSupportPolygons,
               contactableFeet, capturePointPlannerParameters, icpOptimizationParameters, walkingControllerParameters, yoTime, totalMass, gravityZ, controlDT,
               registry, yoGraphicsListRegistry);
      }
      else
      {
         linearMomentumRateOfChangeControlModule = new ICPBasedLinearMomentumRateOfChangeControlModule(referenceFrames, bipedSupportPolygons, controlDT,
               totalMass, gravityZ,icpControlGains, registry, yoGraphicsListRegistry, use2DCMPProjection);
      }
      linearMomentumRateOfChangeControlModule.setControlHeightWithMomentum(walkingControllerParameters.controlHeightWithMomentum());

      icpPlanner = new ICPPlannerWithTimeFreezer(bipedSupportPolygons, contactableFeet, capturePointPlannerParameters, registry, yoGraphicsListRegistry);
      icpPlanner.setMinimumSingleSupportTimeForDisturbanceRecovery(minimumSwingTimeForDisturbanceRecovery);
      icpPlanner.setOmega0(momentumBasedController.getOmega0());
      icpPlanner.setFinalTransferTime(walkingControllerParameters.getDefaultTransferTime());

      safeDistanceFromSupportEdgesToStopCancelICPPlan.set(0.05);
      distanceToShrinkSupportPolygonWhenHoldingCurrent.set(0.08);

      maxICPErrorBeforeSingleSupportX.set(walkingControllerParameters.getMaxICPErrorBeforeSingleSupportX());
      maxICPErrorBeforeSingleSupportY.set(walkingControllerParameters.getMaxICPErrorBeforeSingleSupportY());

      YoPDGains pelvisXYControlGains = walkingControllerParameters.createPelvisICPBasedXYControlGains(registry);
      pelvisICPBasedTranslationManager = new PelvisICPBasedTranslationManager(momentumBasedController, bipedSupportPolygons, pelvisXYControlGains, registry);

      pushRecoveryControlModule = new PushRecoveryControlModule(bipedSupportPolygons, momentumBasedController, walkingControllerParameters, registry);

      double maxAllowedDistanceCMPSupport = walkingControllerParameters.getMaxAllowedDistanceCMPSupport();
      SideDependentList<FrameConvexPolygon2d> defaultFootPolygons = momentumBasedController.getDefaultFootPolygons();
      momentumRecoveryControlModule = new MomentumRecoveryControlModule(defaultFootPolygons, maxAllowedDistanceCMPSupport, registry, yoGraphicsListRegistry);

      controlHeightWithMomentum.set(walkingControllerParameters.controlHeightWithMomentum());
      controlHeightWithMomentum.addVariableChangedListener(new VariableChangedListener()
      {
         @Override public void variableChanged(YoVariable<?> v)
         {
            linearMomentumRateOfChangeControlModule.setControlHeightWithMomentum(controlHeightWithMomentum.getBooleanValue());
         }
      });

      String graphicListName = getClass().getSimpleName();

      if (yoGraphicsListRegistry != null)
      {
         YoGraphicPosition centerOfMassViz = new YoGraphicPosition("Center Of Mass", yoCenterOfMass, 0.006, Black(), GraphicType.BALL_WITH_CROSS);
         YoGraphicPosition desiredCapturePointViz = new YoGraphicPosition("Desired Capture Point", yoDesiredCapturePoint, 0.01, Yellow(), GraphicType.BALL_WITH_ROTATED_CROSS);
         YoGraphicPosition finalDesiredCapturePointViz = new YoGraphicPosition("Final Desired Capture Point", yoFinalDesiredICP, 0.01, Beige(), GraphicType.BALL_WITH_ROTATED_CROSS);
         YoGraphicPosition desiredCMPViz = new YoGraphicPosition("Desired CMP", yoDesiredCMP, 0.012, Purple(), GraphicType.BALL_WITH_CROSS);
         YoGraphicPosition achievedCMPViz = new YoGraphicPosition("Achieved CMP", yoAchievedCMP, 0.005, DarkRed(), GraphicType.BALL_WITH_CROSS);
         YoGraphicPosition perfectCMPViz = new YoGraphicPosition("Perfect CMP", yoPerfectCMP, 0.002, BlueViolet());

         YoGraphicPosition adjustedDesiredCapturePointViz = new YoGraphicPosition("Adjusted Desired Capture Point", yoAdjustedDesiredCapturePoint, 0.005, Yellow(), GraphicType.DIAMOND);
         yoGraphicsListRegistry.registerArtifact(graphicListName, adjustedDesiredCapturePointViz.createArtifact());

         yoGraphicsListRegistry.registerArtifact(graphicListName, centerOfMassViz.createArtifact());
         yoGraphicsListRegistry.registerArtifact(graphicListName, desiredCapturePointViz.createArtifact());
         yoGraphicsListRegistry.registerArtifact(graphicListName, finalDesiredCapturePointViz.createArtifact());
         yoGraphicsListRegistry.registerArtifact(graphicListName, desiredCMPViz.createArtifact());
         yoGraphicsListRegistry.registerArtifact(graphicListName, achievedCMPViz.createArtifact());
         YoArtifactPosition perfectCMPArtifact = perfectCMPViz.createArtifact();
         perfectCMPArtifact.setVisible(false);
         yoGraphicsListRegistry.registerArtifact(graphicListName, perfectCMPArtifact);
      }
      yoCenterOfMass.setToNaN();
      yoDesiredCapturePoint.setToNaN();
      yoFinalDesiredICP.setToNaN();
      yoDesiredCMP.setToNaN();
      yoAchievedCMP.setToNaN();
      yoPerfectCMP.setToNaN();

      parentRegistry.addChild(registry);
   }

   public void setMomentumWeight(Vector3D linearWeight)
   {
      linearMomentumRateOfChangeControlModule.setMomentumWeight(linearWeight);
   }

   public void setMomentumWeight(Vector3D angularWeight, Vector3D linearWeight)
   {
      linearMomentumRateOfChangeControlModule.setMomentumWeight(angularWeight, linearWeight);
   }

   public void setHighMomentumWeightForRecovery(Vector3D highLinearWeight)
   {
      linearMomentumRateOfChangeControlModule.setHighMomentumWeightForRecovery(highLinearWeight);
   }

   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing)
   {
      icpPlanner.addFootstepToPlan(footstep, timing);
      linearMomentumRateOfChangeControlModule.addFootstepToPlan(footstep);
   }

   public void setNextFootstep(Footstep nextFootstep)
   {
      momentumRecoveryControlModule.setNextFootstep(nextFootstep);
   }

   public boolean checkAndUpdateFootstep(Footstep footstep)
   {
      return pushRecoveryControlModule.checkAndUpdateFootstep(getTimeRemainingInCurrentState(), footstep);
   }

   public boolean checkAndUpdateFootstepFromICPOptimization(Footstep footstep)
   {
      return linearMomentumRateOfChangeControlModule.getUpcomingFootstepSolution(footstep);
   }

   public void clearICPPlan()
   {
      icpPlanner.clearPlan();
      linearMomentumRateOfChangeControlModule.clearPlan();
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
      linearMomentumRateOfChangeControlModule.setTransferFromSide(robotSide);
   }

   public void compute(RobotSide supportLeg, double desiredCoMHeightAcceleration, boolean keepCMPInsideSupportPolygon)
   {
      momentumBasedController.getCapturePoint(capturePoint2d);

      icpPlanner.getDesiredCapturePointPositionAndVelocity(desiredCapturePoint2d, desiredCapturePointVelocity2d, capturePoint2d, yoTime.getDoubleValue());
      icpPlanner.getDesiredCentroidalMomentumPivotPosition(perfectCMP);

      pelvisICPBasedTranslationManager.compute(supportLeg, capturePoint2d);
      pelvisICPBasedTranslationManager.addICPOffset(desiredCapturePoint2d, desiredCapturePointVelocity2d);

      double omega0 = momentumBasedController.getOmega0();
      if (supportLeg == null)
         pushRecoveryControlModule.updateForDoubleSupport(desiredCapturePoint2d, capturePoint2d, omega0);
      else
         pushRecoveryControlModule.updateForSingleSupport(desiredCapturePoint2d, capturePoint2d, omega0);

      yoDesiredCapturePoint.set(desiredCapturePoint2d);
      yoDesiredICPVelocity.set(desiredCapturePointVelocity2d);

      yoFinalDesiredICP.getFrameTuple2dIncludingFrame(finalDesiredCapturePoint2d);

      // --- compute adjusted desired capture point
      momentumBasedController.getAdjustedDesiredCapturePoint(desiredCapturePoint2d, adjustedDesiredCapturePoint2d);
      yoAdjustedDesiredCapturePoint.set(adjustedDesiredCapturePoint2d);
      desiredCapturePoint2d.setIncludingFrame(adjustedDesiredCapturePoint2d);
      // ---

      getICPError(icpError2d);
      momentumRecoveryControlModule.setICPError(icpError2d);
      momentumRecoveryControlModule.setSupportSide(supportLeg);
      momentumRecoveryControlModule.setCapturePoint(capturePoint2d);
      momentumRecoveryControlModule.setSupportPolygon(bipedSupportPolygons.getSupportPolygonInWorld());
      momentumRecoveryControlModule.compute();

      momentumRecoveryControlModule.getCMPProjectionArea(areaToProjectInto, safeArea);
      if (!keepCMPInsideSupportPolygon)
         areaToProjectInto.clearAndUpdate(worldFrame);
      linearMomentumRateOfChangeControlModule.setCMPProjectionArea(areaToProjectInto, safeArea);

      if (momentumRecoveryControlModule.getUseHighMomentumWeight())
      {
         linearMomentumRateOfChangeControlModule.setHighMomentumWeight();
      }
      else
      {
         linearMomentumRateOfChangeControlModule.setDefaultMomentumWeight();
      }

         linearMomentumRateOfChangeControlModule.setDesiredCenterOfMassHeightAcceleration(desiredCoMHeightAcceleration);
         linearMomentumRateOfChangeControlModule.setCapturePoint(capturePoint2d);
         linearMomentumRateOfChangeControlModule.setOmega0(omega0);
         linearMomentumRateOfChangeControlModule.setDesiredCapturePoint(adjustedDesiredCapturePoint2d);
         linearMomentumRateOfChangeControlModule.setFinalDesiredCapturePoint(finalDesiredCapturePoint2d);
         linearMomentumRateOfChangeControlModule.setDesiredCapturePointVelocity(desiredCapturePointVelocity2d);
         linearMomentumRateOfChangeControlModule.setPerfectCMP(perfectCMP);
         linearMomentumRateOfChangeControlModule.setSupportLeg(supportLeg);
         yoDesiredCMP.getFrameTuple2d(desiredCMP);
         linearMomentumRateOfChangeControlModule.compute(desiredCMP, desiredCMP);
         yoDesiredCMP.set(desiredCMP);
   }

   public Footstep createFootstepForRecoveringFromDisturbance(RobotSide swingSide, double swingTimeRemaining)
   {
      return pushRecoveryControlModule.createFootstepForRecoveringFromDisturbance(swingSide, swingTimeRemaining);
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
      momentumBasedController.getCapturePoint(capturePoint2d);
      return icpPlanner.estimateTimeRemainingForStateUnderDisturbance(yoTime.getDoubleValue(), capturePoint2d);
   }

   public void freezePelvisXYControl()
   {
      pelvisICPBasedTranslationManager.freeze();
   }

   public void getDesiredCMP(FramePoint2d desiredCMPToPack)
   {
      yoDesiredCMP.getFrameTuple2dIncludingFrame(desiredCMPToPack);
   }

   public void getDesiredICP(FramePoint2d desiredICPToPack)
   {
      yoDesiredCapturePoint.getFrameTuple2dIncludingFrame(desiredICPToPack);
   }

   public void getDesiredICPVelocity(FrameVector2d desiredICPVelocityToPack)
   {
      yoDesiredICPVelocity.getFrameTuple2dIncludingFrame(desiredICPVelocityToPack);
   }

   public MomentumRateCommand getInverseDynamicsCommand()
   {
      return linearMomentumRateOfChangeControlModule.getMomentumRateCommand();
   }

   public void getNextExitCMP(FramePoint entryCMPToPack)
   {
      icpPlanner.getNextExitCMP(entryCMPToPack);
   }

   public double getTimeRemainingInCurrentState()
   {
      return icpPlanner.computeAndReturnTimeInCurrentState(yoTime.getDoubleValue());
   }

   public void handleGoHomeCommand(GoHomeCommand command)
   {
      pelvisICPBasedTranslationManager.handleGoHomeCommand(command);
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
      update();
      yoFinalDesiredICP.set(Double.NaN, Double.NaN);
      momentumBasedController.getCapturePoint(tempCapturePoint);
      yoDesiredCapturePoint.setByProjectionOntoXYPlane(tempCapturePoint);

      icpPlanner.holdCurrentICP(yoTime.getDoubleValue(), tempCapturePoint);
      icpPlanner.initializeForStanding(yoTime.getDoubleValue());

      linearMomentumRateOfChangeControlModule.initializeForStanding();
   }

   public void prepareForDoubleSupportPushRecovery()
   {
      pushRecoveryControlModule.initializeParametersForDoubleSupportPushRecovery();
   }

   public void initializeICPPlanForSingleSupport(double defaultSwingTime, double defaultTransferTime, double finalTransferTime)
   {
      setDefaultSingleSupportTime(defaultSwingTime);
      setDefaultDoubleSupportTime(defaultTransferTime);
      setFinalTransferTime(finalTransferTime);
      icpPlanner.initializeForSingleSupport(yoTime.getDoubleValue());
      linearMomentumRateOfChangeControlModule.initializeForSingleSupport();
   }

   public void initializeICPPlanForStanding(double defaultSwingTime, double defaultTransferTime, double finalTransferTime)
   {
      if (holdICPToCurrentCoMLocationInNextDoubleSupport.getBooleanValue())
      {
         requestICPPlannerToHoldCurrentCoM();
         holdICPToCurrentCoMLocationInNextDoubleSupport.set(false);
      }
      setDefaultSingleSupportTime(defaultSwingTime);
      setDefaultDoubleSupportTime(defaultTransferTime);
      setFinalTransferTime(finalTransferTime);
      icpPlanner.initializeForStanding(yoTime.getDoubleValue());
      linearMomentumRateOfChangeControlModule.initializeForStanding();
   }

   public void initializeICPPlanForTransfer(double defaultSwingTime, double defaultTransferTime, double finalTransferTime)
   {
      if (holdICPToCurrentCoMLocationInNextDoubleSupport.getBooleanValue())
      {
         requestICPPlannerToHoldCurrentCoM();
         holdICPToCurrentCoMLocationInNextDoubleSupport.set(false);
      }
      setDefaultSingleSupportTime(defaultSwingTime);
      setDefaultDoubleSupportTime(defaultTransferTime);
      setFinalTransferTime(finalTransferTime);
      icpPlanner.initializeForTransfer(yoTime.getDoubleValue());
      linearMomentumRateOfChangeControlModule.initializeForTransfer();
   }

   public boolean isTransitionToSingleSupportSafe(RobotSide transferToSide)
   {
      getICPError(icpError2d);
      ReferenceFrame leadingAnkleZUpFrame = bipedSupportPolygons.getAnkleZUpFrames().get(transferToSide);
      icpError2d.changeFrame(leadingAnkleZUpFrame);
      double ellipticErrorSquared = MathTools.square(icpError2d.getX() / maxICPErrorBeforeSingleSupportX.getDoubleValue())
            + MathTools.square(icpError2d.getY() / maxICPErrorBeforeSingleSupportY.getDoubleValue());
      boolean closeEnough = ellipticErrorSquared < 1.0;
      return closeEnough;
   }

   public boolean isTransitionToStandingSafe()
   {
      FrameConvexPolygon2d supportPolygonInWorld = bipedSupportPolygons.getSupportPolygonInWorld();
      momentumBasedController.getCapturePoint(capturePoint2d);

      // signed distance returns a negative number if the point is inside the polygon.
      return supportPolygonInWorld.getSignedDistance(capturePoint2d) < -safeDistanceFromSupportEdgesToStopCancelICPPlan.getDoubleValue();
   }

   public double getICPErrorMagnitude()
   {
      momentumBasedController.getCapturePoint(capturePoint2d);
      return capturePoint2d.distance(yoDesiredCapturePoint.getFrameTuple2d());
   }

   public void getICPError(FrameVector2d icpErrorToPack)
   {
      momentumBasedController.getCapturePoint(capturePoint2d);
      yoDesiredCapturePoint.getFrameTuple2dIncludingFrame(desiredCapturePoint2d);
      icpErrorToPack.setIncludingFrame(desiredCapturePoint2d);
      icpErrorToPack.sub(capturePoint2d);
   }

   public boolean isICPPlanDone()
   {
      return icpPlanner.isDone(yoTime.getDoubleValue());
   }

   public boolean isOnExitCMP()
   {
      return icpPlanner.isOnExitCMP();
   }

   public boolean isPushRecoveryEnabled()
   {
      return pushRecoveryControlModule.isEnabled();
   }

   public boolean useICPOptimization()
   {
      return useICPOptimizationControl;
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

      FrameConvexPolygon2d supportPolygonInMidFeetZUp = bipedSupportPolygons.getSupportPolygonInMidFeetZUp();
      convexPolygonShrinker.shrinkConstantDistanceInto(supportPolygonInMidFeetZUp, distanceToShrinkSupportPolygonWhenHoldingCurrent.getDoubleValue(), shrunkSupportPolygon);

      centerOfMassPosition.changeFrame(shrunkSupportPolygon.getReferenceFrame());
      centerOfMassPosition2d.setByProjectionOntoXYPlaneIncludingFrame(centerOfMassPosition);
      shrunkSupportPolygon.orthogonalProjection(centerOfMassPosition2d);
      centerOfMassPosition.setXY(centerOfMassPosition2d);

      centerOfMassPosition.changeFrame(worldFrame);
      icpPlanner.holdCurrentICP(yoTime.getDoubleValue(), centerOfMassPosition);
   }

   private void setDefaultDoubleSupportTime(double defaultDoubleSupportTime)
   {
      linearMomentumRateOfChangeControlModule.setDoubleSupportDuration(defaultDoubleSupportTime);
   }

   private void setDefaultSingleSupportTime(double defaultSingleSupportTime)
   {
      linearMomentumRateOfChangeControlModule.setSingleSupportDuration(defaultSingleSupportTime);
   }

   private void setFinalTransferTime(double finalTransferTime)
   {
      icpPlanner.setFinalTransferTime(finalTransferTime);
   }

   /**
    * Update the basics: capture point, omega0, and the support polygons.
    */
   public void update()
   {
      centerOfMassPosition.setToZero(centerOfMassFrame);
      yoCenterOfMass.setAndMatchFrame(centerOfMassPosition);
      double omega0 = momentumBasedController.getOmega0();
      CapturePointTools.computeDesiredCentroidalMomentumPivot(yoDesiredCapturePoint, yoDesiredICPVelocity, omega0, yoPerfectCMP);
      icpPlanner.getFinalDesiredCapturePointPosition(yoFinalDesiredICP);
   }

   public void computeAchievedCMP(FrameVector achievedLinearMomentumRate)
   {
      linearMomentumRateOfChangeControlModule.computeAchievedCMP(achievedLinearMomentumRate, achievedCMP);
      yoAchievedCMP.setAndMatchFrame(achievedCMP);
   }

   public CapturabilityBasedStatus updateAndReturnCapturabilityBasedStatus()
   {
      yoDesiredCapturePoint.getFrameTuple2dIncludingFrame(desiredCapturePoint2d);
      centerOfMassPosition.setToZero(centerOfMassFrame);
      centerOfMassPosition.changeFrame(worldFrame);

      momentumBasedController.getCapturePoint(capturePoint2d);
      capturePoint2d.checkReferenceFrameMatch(worldFrame);
      desiredCapturePoint2d.checkReferenceFrameMatch(worldFrame);

      SideDependentList<FrameConvexPolygon2d> footSupportPolygons = bipedSupportPolygons.getFootPolygonsInWorldFrame();

      capturePoint2d.get(capturabilityBasedStatus.capturePoint);
      desiredCapturePoint2d.get(capturabilityBasedStatus.desiredCapturePoint);
      centerOfMassPosition.get(capturabilityBasedStatus.centerOfMass);
      for (RobotSide robotSide : RobotSide.values)
      {
         capturabilityBasedStatus.setSupportPolygon(robotSide, footSupportPolygons.get(robotSide));
      }

      return capturabilityBasedStatus;
   }

   public void updateCurrentICPPlan()
   {
      icpPlanner.updateCurrentPlan();
   }

   public void updateICPPlanForSingleSupportDisturbances()
   {
      momentumBasedController.getCapturePoint(capturePoint2d);
      icpPlanner.updatePlanForSingleSupportDisturbances(yoTime.getDoubleValue(), capturePoint2d);
   }

   public void updateSwingTimeRemaining(double timeRemainingInSwing)
   {
      linearMomentumRateOfChangeControlModule.submitRemainingTimeInSwingUnderDisturbance(timeRemainingInSwing);
   }

   public void getCapturePoint(FramePoint2d capturePointToPack)
   {
      momentumBasedController.getCapturePoint(capturePointToPack);
   }

   public void minimizeAngularMomentumRateZ(boolean enable)
   {
      linearMomentumRateOfChangeControlModule.minimizeAngularMomentumRateZ(enable);
   }
}
