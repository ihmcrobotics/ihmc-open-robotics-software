package us.ihmc.commonWalkingControlModules.capturePoint;

import static us.ihmc.graphicsDescription.appearance.YoAppearance.*;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.SmoothCMPBasedICPPlanner;
import us.ihmc.commonWalkingControlModules.captureRegion.PushRecoveryControlModule;
import us.ihmc.commonWalkingControlModules.configurations.ICPAngularMomentumModifierParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.PelvisICPBasedTranslationManager;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CopTrajectory;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.SupportSequence;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlanner;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlannerInterface;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CornerPointViewer;
import us.ihmc.commonWalkingControlModules.dynamicReachability.DynamicReachabilityCalculator;
import us.ihmc.commonWalkingControlModules.messageHandlers.CenterOfMassTrajectoryHandler;
import us.ihmc.commonWalkingControlModules.messageHandlers.MomentumTrajectoryHandler;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepShiftFractions;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

public class BalanceManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final BipedSupportPolygons bipedSupportPolygons;
   private final MomentumTrajectoryHandler momentumTrajectoryHandler;
   private final PrecomputedICPPlanner precomputedICPPlanner;

   private final LinearMomentumRateControlModuleInput linearMomentumRateControlModuleInput = new LinearMomentumRateControlModuleInput();

   private final PelvisICPBasedTranslationManager pelvisICPBasedTranslationManager;
   private final PushRecoveryControlModule pushRecoveryControlModule;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   private final YoFramePoint2D yoDesiredCapturePoint = new YoFramePoint2D("desiredICP", worldFrame, registry);
   private final YoFrameVector2D yoDesiredICPVelocity = new YoFrameVector2D("desiredICPVelocity", worldFrame, registry);
   private final YoFramePoint3D yoDesiredCoMPosition = new YoFramePoint3D("desiredCoMPosition", worldFrame, registry);
   private final YoFrameVector3D yoDesiredCoMVelocity = new YoFrameVector3D("desiredCoMVelocity", worldFrame, registry);
   private final YoFramePoint2D yoFinalDesiredICP = new YoFramePoint2D("finalDesiredICP", worldFrame, registry);
   private final YoFramePoint3D yoFinalDesiredCoM = new YoFramePoint3D("finalDesiredCoM", worldFrame, registry);

   /** CoP position according to the ICP planner */
   private final YoFramePoint2D yoPerfectCoP = new YoFramePoint2D("perfectCoP", worldFrame, registry);
   /** CMP position according to the ICP planner */
   private final YoFramePoint2D yoPerfectCMP = new YoFramePoint2D("perfectCMP", worldFrame, registry);

   private final YoBoolean editStepTimingForReachability = new YoBoolean("editStepTimingForReachability", registry);

   private final YoBoolean useMomentumRecoveryModeForBalance = new YoBoolean("useMomentumRecoveryModeForBalance", registry);

   private final YoDouble yoTime;

   private final ReferenceFrame centerOfMassFrame;

   private final FramePoint3D centerOfMassPosition = new FramePoint3D();
   private final FramePoint2D centerOfMassPosition2d = new FramePoint2D();

   private final FramePoint2D capturePoint2d = new FramePoint2D();
   private final FramePoint3D tempCapturePoint = new FramePoint3D();
   private final FramePoint2D desiredCapturePoint2d = new FramePoint2D();
   private final FrameVector2D desiredCapturePointVelocity2d = new FrameVector2D();
   private final FramePoint2D perfectCoP2d = new FramePoint2D();

   private final YoBoolean blendICPTrajectories = new YoBoolean("blendICPTrajectories", registry);

   private final FramePoint2D adjustedDesiredCapturePoint2d = new FramePoint2D();
   private final YoFramePoint2D yoAdjustedDesiredCapturePoint = new YoFramePoint2D("adjustedDesiredICP", worldFrame, registry);

   private final FrameVector2D icpError2d = new FrameVector2D();

   private final ConvexPolygonScaler convexPolygonShrinker = new ConvexPolygonScaler();
   private final FrameConvexPolygon2D shrunkSupportPolygon = new FrameConvexPolygon2D();

   private final YoDouble safeDistanceFromSupportEdgesToStopCancelICPPlan = new YoDouble("safeDistanceFromSupportEdgesToStopCancelICPPlan", registry);
   private final YoDouble distanceToShrinkSupportPolygonWhenHoldingCurrent = new YoDouble("distanceToShrinkSupportPolygonWhenHoldingCurrent", registry);

   private final YoBoolean holdICPToCurrentCoMLocationInNextDoubleSupport = new YoBoolean("holdICPToCurrentCoMLocationInNextDoubleSupport", registry);

   private final YoDouble normalizedICPError = new YoDouble("normalizedICPError", registry);
   private final DoubleProvider maxICPErrorBeforeSingleSupportForwardX;
   private final DoubleProvider maxICPErrorBeforeSingleSupportBackwardX;
   private final DoubleProvider maxICPErrorBeforeSingleSupportInnerY;
   private final DoubleProvider maxICPErrorBeforeSingleSupportOuterY;

   private final DoubleProvider icpDistanceOutsideSupportForStep = new DoubleParameter("icpDistanceOutsideSupportForStep", registry, 0.03);
   private final DoubleProvider ellipticICPErrorForMomentumRecovery = new DoubleParameter("ellipticICPErrorForMomentumRecovery", registry, 2.0);

   private final CapturabilityBasedStatus capturabilityBasedStatus = new CapturabilityBasedStatus();

   private final YoBoolean icpPlannerDone = new YoBoolean("ICPPlannerDone", registry);

   private boolean initializeForStanding = false;
   private boolean initializeForSingleSupport = false;
   private boolean initializeForTransfer = false;
   private boolean minimizeAngularMomentumRateZ = false;
   private boolean footstepWasAdjusted = false;
   private boolean usingStepAdjustment = false;
   private double finalTransferDuration;
   private double timeRemainingInSwing = Double.NaN;
   private RobotSide supportSide;
   private RobotSide transferToSide;
   private final FixedFramePose3DBasics footstepSolution = new FramePose3D();
   private final FixedFramePoint2DBasics desiredCMP = new FramePoint2D();
   private final FixedFrameVector3DBasics effectiveICPAdjustment = new FrameVector3D();
   private final SideDependentList<PlaneContactStateCommand> contactStateCommands = new SideDependentList<>(new PlaneContactStateCommand(),
                                                                                                            new PlaneContactStateCommand());

   private final List<Footstep> footsteps = new ArrayList<>();
   private final List<FootstepTiming> footstepTimings = new ArrayList<>();
   private final List<FootstepShiftFractions> footstepShiftFractions = new ArrayList<>();

   private final YoBoolean inSingleSupport = new YoBoolean("InSingleSupport", registry);
   private final YoBoolean inFinalTransfer = new YoBoolean("InFinalTransfer", registry);
   private final FootstepTiming currentTiming = new FootstepTiming();
   private final YoDouble timeInSupportSequence = new YoDouble("TimeInSupportSequence", registry);
   private final CopTrajectory copTrajectory;
   private final SupportSequence supportSeqence;
   private final CoMTrajectoryPlannerInterface comTrajectoryPlanner;

   public BalanceManager(HighLevelHumanoidControllerToolbox controllerToolbox, WalkingControllerParameters walkingControllerParameters,
                         ICPWithTimeFreezingPlannerParameters icpPlannerParameters, ICPAngularMomentumModifierParameters angularMomentumModifierParameters,
                         YoRegistry parentRegistry)
   {
      CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();

      YoGraphicsListRegistry yoGraphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();

      double gravityZ = controllerToolbox.getGravityZ();
      double totalMass = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator());

      this.controllerToolbox = controllerToolbox;
      yoTime = controllerToolbox.getYoTime();

      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();

      bipedSupportPolygons = controllerToolbox.getBipedSupportPolygons();

      WalkingMessageHandler walkingMessageHandler = controllerToolbox.getWalkingMessageHandler();
      momentumTrajectoryHandler = walkingMessageHandler == null ? null : walkingMessageHandler.getMomentumTrajectoryHandler();

      if (walkingMessageHandler != null)
      {
         CenterOfMassTrajectoryHandler comTrajectoryHandler = walkingMessageHandler.getComTrajectoryHandler();
         double dt = controllerToolbox.getControlDT();
         precomputedICPPlanner = new PrecomputedICPPlanner(dt, comTrajectoryHandler, momentumTrajectoryHandler, registry, yoGraphicsListRegistry);
         precomputedICPPlanner.setOmega0(controllerToolbox.getOmega0());
         precomputedICPPlanner.setMass(totalMass);
         precomputedICPPlanner.setGravity(gravityZ);
      }
      else
      {
         precomputedICPPlanner = null;
      }
      blendICPTrajectories.set(true);

      editStepTimingForReachability.set(walkingControllerParameters.editStepTimingForReachability());

      safeDistanceFromSupportEdgesToStopCancelICPPlan.set(0.05);
      distanceToShrinkSupportPolygonWhenHoldingCurrent.set(0.08);

      maxICPErrorBeforeSingleSupportForwardX = new DoubleParameter("maxICPErrorBeforeSingleSupportForwardX", registry, walkingControllerParameters.getMaxICPErrorBeforeSingleSupportForwardX());
      maxICPErrorBeforeSingleSupportBackwardX = new DoubleParameter("maxICPErrorBeforeSingleSupportBackwardX", registry, walkingControllerParameters.getMaxICPErrorBeforeSingleSupportBackwardX());
      maxICPErrorBeforeSingleSupportInnerY = new DoubleParameter("maxICPErrorBeforeSingleSupportInnerY", registry, walkingControllerParameters.getMaxICPErrorBeforeSingleSupportInnerY());
      maxICPErrorBeforeSingleSupportOuterY = new DoubleParameter("maxICPErrorBeforeSingleSupportOuterY", registry, walkingControllerParameters.getMaxICPErrorBeforeSingleSupportOuterY());

      double pelvisTranslationICPSupportPolygonSafeMargin = walkingControllerParameters.getPelvisTranslationICPSupportPolygonSafeMargin();
      pelvisICPBasedTranslationManager = new PelvisICPBasedTranslationManager(controllerToolbox, pelvisTranslationICPSupportPolygonSafeMargin, bipedSupportPolygons, registry);

      FrameConvexPolygon2D defaultSupportPolygon = controllerToolbox.getDefaultFootPolygons().get(RobotSide.LEFT);
      SideDependentList<MovingReferenceFrame> soleFrames = controllerToolbox.getReferenceFrames().getSoleFrames();
      SideDependentList<MovingReferenceFrame> soleZUpFrames = controllerToolbox.getReferenceFrames().getSoleZUpFrames();
      supportSeqence = new SupportSequence(defaultSupportPolygon, soleFrames, soleZUpFrames, bipedSupportPolygons, registry, controllerToolbox.getYoGraphicsListRegistry());
      comTrajectoryPlanner = new CoMTrajectoryPlanner(gravityZ, 1.0, registry);
      ((CoMTrajectoryPlanner) comTrajectoryPlanner).setCornerPointViewer(new CornerPointViewer(registry, yoGraphicsListRegistry));
      copTrajectory = new CopTrajectory(registry, controllerToolbox.getYoGraphicsListRegistry());

      pushRecoveryControlModule = new PushRecoveryControlModule(bipedSupportPolygons, controllerToolbox, walkingControllerParameters, registry);

      String graphicListName = getClass().getSimpleName();

      if (yoGraphicsListRegistry != null)
      {
         YoGraphicPosition desiredCapturePointViz = new YoGraphicPosition("Desired Capture Point", yoDesiredCapturePoint, 0.01, Yellow(), GraphicType.BALL_WITH_ROTATED_CROSS);
         YoGraphicPosition finalDesiredCapturePointViz = new YoGraphicPosition("Final Desired Capture Point", yoFinalDesiredICP, 0.01, Beige(), GraphicType.BALL_WITH_ROTATED_CROSS);
         YoGraphicPosition finalDesiredCoMViz = new YoGraphicPosition("Final Desired CoM", yoFinalDesiredCoM, 0.01, Black(), GraphicType.BALL_WITH_ROTATED_CROSS);
         YoGraphicPosition perfectCMPViz = new YoGraphicPosition("Perfect CMP", yoPerfectCMP, 0.002, BlueViolet());
         YoGraphicPosition perfectCoPViz = new YoGraphicPosition("Perfect CoP", yoPerfectCoP, 0.002, DarkViolet(), GraphicType.BALL_WITH_CROSS);

         YoGraphicPosition adjustedDesiredCapturePointViz = new YoGraphicPosition("Adjusted Desired Capture Point", yoAdjustedDesiredCapturePoint, 0.005, Yellow(), GraphicType.DIAMOND);
         yoGraphicsListRegistry.registerArtifact(graphicListName, adjustedDesiredCapturePointViz.createArtifact());

         yoGraphicsListRegistry.registerArtifact(graphicListName, desiredCapturePointViz.createArtifact());
         yoGraphicsListRegistry.registerArtifact(graphicListName, finalDesiredCapturePointViz.createArtifact());
         yoGraphicsListRegistry.registerArtifact(graphicListName, finalDesiredCoMViz.createArtifact());
         YoArtifactPosition perfectCMPArtifact = perfectCMPViz.createArtifact();
         perfectCMPArtifact.setVisible(false);
         yoGraphicsListRegistry.registerArtifact(graphicListName, perfectCMPArtifact);
         YoArtifactPosition perfectCoPArtifact = perfectCoPViz.createArtifact();
         perfectCoPArtifact.setVisible(false);
         yoGraphicsListRegistry.registerArtifact(graphicListName, perfectCoPArtifact);
      }
      yoDesiredCapturePoint.setToNaN();
      yoFinalDesiredICP.setToNaN();
      yoPerfectCMP.setToNaN();
      yoPerfectCoP.setToNaN();

      parentRegistry.addChild(registry);
   }

   public void setUseMomentumRecoveryModeForBalance(boolean useMomentumRecoveryModeForBalance)
   {
      this.useMomentumRecoveryModeForBalance.set(useMomentumRecoveryModeForBalance);
   }

   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing, FootstepShiftFractions shiftFractions)
   {
      footsteps.add(footstep);
      footstepTimings.add(timing);
      footstepShiftFractions.add(shiftFractions);
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
      footsteps.clear();
      footstepTimings.clear();
      footstepShiftFractions.clear();
   }

   public void setICPPlanSupportSide(RobotSide supportSide)
   {
      this.supportSide = supportSide;
   }

   public void setICPPlanTransferToSide(RobotSide transferToSide)
   {
      this.transferToSide = transferToSide;
   }

   public void setICPPlanTransferFromSide(RobotSide robotSide)
   {
      this.transferToSide = robotSide != null ? robotSide.getOppositeSide() : null;
   }

   public void endTick()
   {
   }

   private final FramePoint3D copEstimate = new FramePoint3D();
   public void compute(RobotSide supportLeg, double desiredCoMHeightAcceleration, boolean keepCoPInsideSupportPolygon, boolean controlHeightWithMomentum)
   {
      controllerToolbox.getCoP(copEstimate);

      desiredCapturePoint2d.set(comTrajectoryPlanner.getDesiredDCMPosition());
      desiredCapturePointVelocity2d.set(comTrajectoryPlanner.getDesiredDCMVelocity());
      perfectCoP2d.set(comTrajectoryPlanner.getDesiredECMPPosition());
      yoDesiredCoMPosition.set(comTrajectoryPlanner.getDesiredCoMPosition());
      yoDesiredCoMVelocity.set(comTrajectoryPlanner.getDesiredCoMVelocity());

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

      getICPError(icpError2d);

      CapturePointTools.computeCentroidalMomentumPivot(desiredCapturePoint2d, desiredCapturePointVelocity2d, omega0, yoPerfectCMP);

      for (RobotSide robotSide : RobotSide.values)
      {
         YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotSide);
         contactState.getPlaneContactStateCommand(contactStateCommands.get(robotSide));
      }

      linearMomentumRateControlModuleInput.setDesiredCenterOfMassHeightAcceleration(desiredCoMHeightAcceleration);
      linearMomentumRateControlModuleInput.setInitializeForStanding(initializeForStanding);
      linearMomentumRateControlModuleInput.setInitializeForTransfer(initializeForTransfer);
      linearMomentumRateControlModuleInput.setInitializeForSingleSupport(initializeForSingleSupport);
      linearMomentumRateControlModuleInput.setFromFootsteps(footsteps);
      linearMomentumRateControlModuleInput.setFromFootstepTimings(footstepTimings);
      linearMomentumRateControlModuleInput.setFinalTransferDuration(finalTransferDuration);
      linearMomentumRateControlModuleInput.setKeepCoPInsideSupportPolygon(keepCoPInsideSupportPolygon);
      linearMomentumRateControlModuleInput.setControlHeightWithMomentum(controlHeightWithMomentum);
      linearMomentumRateControlModuleInput.setOmega0(omega0);
      linearMomentumRateControlModuleInput.setUseMomentumRecoveryMode(useMomentumRecoveryModeForBalance.getBooleanValue());
      linearMomentumRateControlModuleInput.setDesiredCapturePoint(desiredCapturePoint2d);
      linearMomentumRateControlModuleInput.setDesiredCapturePointVelocity(desiredCapturePointVelocity2d);
      linearMomentumRateControlModuleInput.setDesiredICPAtEndOfState(yoFinalDesiredICP);
      linearMomentumRateControlModuleInput.setPerfectCMP(yoPerfectCMP);
      linearMomentumRateControlModuleInput.setPerfectCoP(yoPerfectCoP);
      linearMomentumRateControlModuleInput.setSupportSide(supportSide);
      linearMomentumRateControlModuleInput.setTransferToSide(transferToSide);
      linearMomentumRateControlModuleInput.setMinimizeAngularMomentumRateZ(minimizeAngularMomentumRateZ);
      linearMomentumRateControlModuleInput.setRemainingTimeInSwingUnderDisturbance(timeRemainingInSwing);
      linearMomentumRateControlModuleInput.setContactStateCommand(contactStateCommands);

      initializeForStanding = false;
      initializeForTransfer = false;
      initializeForSingleSupport = false;
      supportSide = null;
      transferToSide = null;
      timeRemainingInSwing = Double.NaN;

      // This is for debugging such that the momentum trajectory handler YoVariables contain the current value:
      if (momentumTrajectoryHandler != null)
      {
         momentumTrajectoryHandler.packDesiredAngularMomentumAtTime(yoTime.getValue(), null, null);
      }
   }

   public void computeICPPlan()
   {
      supportSeqence.update(footsteps, footstepTimings);

      copTrajectory.update(supportSeqence, finalTransferDuration);

      comTrajectoryPlanner.setInitialCenterOfMassState(yoDesiredCoMPosition, yoDesiredCoMVelocity);
      comTrajectoryPlanner.solveForTrajectory(copTrajectory.getContactStates());
      int segment = inSingleSupport.getBooleanValue() ? 1 : 0;
      comTrajectoryPlanner.compute(segment, timeInSupportSequence.getDoubleValue());

      if (footstepTimings.isEmpty())
         yoFinalDesiredICP.setToNaN();

      // If this condition is false we are experiencing a late touchdown or a delayed liftoff. Do not advance the time in support sequence! // FIXME is this right
      if (footsteps.isEmpty() || !icpPlannerDone.getValue())
         timeInSupportSequence.add(controllerToolbox.getControlDT());

      if (inFinalTransfer.getValue())
         icpPlannerDone.set(timeInSupportSequence.getValue() >= finalTransferDuration);
      else if (footstepTimings.isEmpty())
         icpPlannerDone.set(true);
      else if (inSingleSupport.getValue())
         icpPlannerDone.set(timeInSupportSequence.getValue() >= currentTiming.getStepTime());
      else
         icpPlannerDone.set(timeInSupportSequence.getValue() >= currentTiming.getTransferTime());
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


      // FIXME
//      return icpPlanner.estimateTimeRemainingForStateUnderDisturbance(capturePoint2d);
      return 0.0;
   }

   public void freezePelvisXYControl()
   {
      pelvisICPBasedTranslationManager.freeze();
   }

   public void getDesiredCMP(FramePoint2D desiredCMPToPack)
   {
      desiredCMPToPack.setIncludingFrame(desiredCMP);
   }

   public void getDesiredICP(FramePoint2D desiredICPToPack)
   {
      desiredICPToPack.setIncludingFrame(yoDesiredCapturePoint);
   }

   public void getDesiredICPVelocity(FrameVector2D desiredICPVelocityToPack)
   {
      desiredICPVelocityToPack.setIncludingFrame(yoDesiredICPVelocity);
   }

   public void getFinalDesiredCoMPosition(FixedFramePoint3DBasics desiredCoMPositionToPack)
   {
      desiredCoMPositionToPack.set(yoFinalDesiredCoM);
   }

   public void getDesiredCoMPosition(FixedFramePoint3DBasics desiredCoMPositionToPack)
   {
      desiredCoMPositionToPack.set(yoDesiredCoMPosition);
   }

   public void getDesiredCoMVelocity(FixedFrameVector2DBasics desiredCoMVelocityToPack)
   {
      desiredCoMVelocityToPack.set(yoDesiredCoMVelocity);
   }

   public void getNextExitCMP(FramePoint3D entryCMPToPack)
   {
      // TODO:
      entryCMPToPack.setToNaN();
   }

   public double getTimeRemainingInCurrentState()
   {
      if (footstepTimings.isEmpty())
         return 0.0;
      return currentTiming.getStepTime() - timeInSupportSequence.getValue();
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

      desiredCapturePoint2d.set(tempCapturePoint);
      supportSeqence.initializeStance();
      timeInSupportSequence.set(0.0);
      inSingleSupport.set(false);
      inFinalTransfer.set(false);

      initializeForStanding = true;

      endTick();
   }

   public void prepareForDoubleSupportPushRecovery()
   {
      pushRecoveryControlModule.initializeParametersForDoubleSupportPushRecovery();
   }

   public void initializeICPPlanForSingleSupport()
   {
      inSingleSupport.set(true);
      inFinalTransfer.set(false);
      supportSeqence.changeFootFrame(footsteps.get(0).getRobotSide(), worldFrame);
      timeInSupportSequence.set(0.0);

      initializeForSingleSupport = true;

      // FIXME
//      icpPlanner.getFinalDesiredCapturePointPosition(yoFinalDesiredICP);
//      icpPlanner.getFinalDesiredCenterOfMassPosition(yoFinalDesiredCoM);

      icpPlannerDone.set(false);
   }


   public void initializeICPPlanForStanding()
   {
      if (holdICPToCurrentCoMLocationInNextDoubleSupport.getBooleanValue())
      {
         requestICPPlannerToHoldCurrentCoM();
         holdICPToCurrentCoMLocationInNextDoubleSupport.set(false);
      }
      supportSeqence.initializeStance();
      timeInSupportSequence.set(0.0);
      inSingleSupport.set(false);
      inFinalTransfer.set(false);
      initializeForStanding = true;

      icpPlannerDone.set(false);
   }

   public void initializeICPPlanForTransferToStanding()
   {
      if (holdICPToCurrentCoMLocationInNextDoubleSupport.getBooleanValue())
      {
         requestICPPlannerToHoldCurrentCoM();
         holdICPToCurrentCoMLocationInNextDoubleSupport.set(false);
      }
      supportSeqence.initializeStance();
      timeInSupportSequence.set(0.0);
      inSingleSupport.set(false);
      inFinalTransfer.set(true);
      initializeForStanding = true;

      icpPlannerDone.set(false);
   }

   public void initializeICPPlanForTransfer()
   {
      if (holdICPToCurrentCoMLocationInNextDoubleSupport.getBooleanValue())
      {
         requestICPPlannerToHoldCurrentCoM();
         holdICPToCurrentCoMLocationInNextDoubleSupport.set(false);
      }

      supportSeqence.initializeStance();
      currentTiming.set(footstepTimings.get(0));
      timeInSupportSequence.set(0.0);
      inFinalTransfer.set(false);
      inSingleSupport.set(false);

      initializeForTransfer = true;

//      icpPlanner.getFinalDesiredCapturePointPosition(yoFinalDesiredICP);
//      icpPlanner.getFinalDesiredCenterOfMassPosition(yoFinalDesiredCoM);

      icpPlannerDone.set(false);
   }

   public void computeNormalizedEllipticICPError(RobotSide transferToSide)
   {
      getICPError(icpError2d);
      ReferenceFrame leadingSoleZUpFrame = controllerToolbox.getReferenceFrames().getSoleZUpFrame(transferToSide);
      icpError2d.changeFrame(leadingSoleZUpFrame);
      boolean isICPErrorToTheInside = transferToSide == RobotSide.RIGHT ? icpError2d.getY() > 0.0 : icpError2d.getY() < 0.0;
      double maxICPErrorBeforeSingleSupportX = icpError2d.getX() > 0.0 ? maxICPErrorBeforeSingleSupportForwardX.getValue() : maxICPErrorBeforeSingleSupportBackwardX.getValue();
      double maxICPErrorBeforeSingleSupportY = isICPErrorToTheInside ? maxICPErrorBeforeSingleSupportInnerY.getValue() : maxICPErrorBeforeSingleSupportOuterY.getValue();
      normalizedICPError.set(MathTools.square(icpError2d.getX() / maxICPErrorBeforeSingleSupportX) + MathTools.square(icpError2d.getY() / maxICPErrorBeforeSingleSupportY));
   }

   public double getNormalizedEllipticICPError()
   {
      return normalizedICPError.getValue();
   }

   public double getEllipticICPErrorForMomentumRecovery()
   {
      return ellipticICPErrorForMomentumRecovery.getValue();
   }

   public double getICPDistanceOutsideSupportForStep()
   {
      return icpDistanceOutsideSupportForStep.getValue();
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

      FrameConvexPolygon2DReadOnly supportPolygonInMidFeetZUp = bipedSupportPolygons.getSupportPolygonInMidFeetZUp();
      convexPolygonShrinker.scaleConvexPolygon(supportPolygonInMidFeetZUp, distanceToShrinkSupportPolygonWhenHoldingCurrent.getDoubleValue(), shrunkSupportPolygon);

      centerOfMassPosition.changeFrame(shrunkSupportPolygon.getReferenceFrame());
      centerOfMassPosition2d.setIncludingFrame(centerOfMassPosition);
      shrunkSupportPolygon.orthogonalProjection(centerOfMassPosition2d);
      centerOfMassPosition.set(centerOfMassPosition2d, 0.0);

      centerOfMassPosition.changeFrame(worldFrame);
      // TODO
      throw new RuntimeException("Reimplement me.");
   }

   public void setFinalTransferWeightDistribution(double weightDistribution)
   {
      // TODO
//      icpPlanner.setFinalTransferWeightDistribution(weightDistribution);
   }

   public void setFinalTransferSplitFraction(double finalTransferSplitFraction)
   {
      // TODO
//      icpPlanner.setFinalTransferDurationAlpha(finalTransferSplitFraction);
   }

   public void setFinalTransferTime(double finalTransferDuration)
   {
      // todo
//      icpPlanner.setFinalTransferDuration(finalTransferDuration);
      this.finalTransferDuration = finalTransferDuration;
   }

   /**
    * Update the basics: capture point, omega0, and the support polygons.
    */
   public void update()
   {
      computeICPPlan();
      // TODO
//      icpPlanner.getFinalDesiredCapturePointPosition(yoFinalDesiredICP);
//      icpPlanner.getFinalDesiredCenterOfMassPosition(yoFinalDesiredCoM);
   }

   public CapturabilityBasedStatus updateAndReturnCapturabilityBasedStatus()
   {
      desiredCapturePoint2d.setIncludingFrame(yoDesiredCapturePoint);
      centerOfMassPosition.setToZero(centerOfMassFrame);
      centerOfMassPosition.changeFrame(worldFrame);

      controllerToolbox.getCapturePoint(capturePoint2d);
      capturePoint2d.checkReferenceFrameMatch(worldFrame);
      desiredCapturePoint2d.checkReferenceFrameMatch(worldFrame);

      capturabilityBasedStatus.setOmega(controllerToolbox.getOmega0());
      capturabilityBasedStatus.getCapturePoint2d().set(capturePoint2d);
      capturabilityBasedStatus.getDesiredCapturePoint2d().set(desiredCapturePoint2d);
      capturabilityBasedStatus.getCenterOfMass3d().set(centerOfMassPosition);
      for (RobotSide robotSide : RobotSide.values)
      {
         HumanoidMessageTools.packFootSupportPolygon(robotSide, bipedSupportPolygons.getFootPolygonInSoleFrame(robotSide), capturabilityBasedStatus);
      }

      return capturabilityBasedStatus;
   }

   public void updateCurrentICPPlan()
   {
   }

   public void updateSwingTimeRemaining(double timeRemainingInSwing)
   {
      this.timeRemainingInSwing = timeRemainingInSwing;
   }

   public FrameVector3DReadOnly getEffectiveICPAdjustment()
   {
      return effectiveICPAdjustment;
   }

   public void getCapturePoint(FramePoint2D capturePointToPack)
   {
      controllerToolbox.getCapturePoint(capturePointToPack);
   }

   public void minimizeAngularMomentumRateZ(boolean minimizeAngularMomentumRateZ)
   {
      this.minimizeAngularMomentumRateZ = minimizeAngularMomentumRateZ;
   }

   public LinearMomentumRateControlModuleInput getLinearMomentumRateControlModuleInput()
   {
      return linearMomentumRateControlModuleInput;
   }

   public void setLinearMomentumRateControlModuleOutput(LinearMomentumRateControlModuleOutput output)
   {
      desiredCMP.set(output.getDesiredCMP());
      effectiveICPAdjustment.set(output.getEffectiveICPAdjustment());
      footstepSolution.set(output.getFootstepSolution());
      footstepWasAdjusted = output.getFootstepWasAdjusted();
      usingStepAdjustment = output.getUsingStepAdjustment();
   }

   public TaskspaceTrajectoryStatusMessage pollPelvisXYTranslationStatusToReport()
   {
      return pelvisICPBasedTranslationManager.pollStatusToReport();
   }
}
