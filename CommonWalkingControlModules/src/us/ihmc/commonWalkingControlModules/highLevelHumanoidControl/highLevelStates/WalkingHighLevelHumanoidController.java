package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.SdfLoader.partNames.LimbName;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.captureRegion.PushRecoveryControlModule;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerCommandInputManager;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableAbortWalkingMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableArmDesiredAccelerationsMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableArmTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableAutomaticManipulationAbortMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableChestTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableEndEffectorLoadBearingMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableFootTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableFootstepDataListMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableGoHomeMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableHandComplianceControlParametersMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableHandTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableHeadTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiablePauseWalkingMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiablePelvisHeightTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiablePelvisOrientationTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiablePelvisTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableStopAllTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.output.ControllerStatusOutputManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.desiredFootStep.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingManagers;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.BalanceManager;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.ControllerCoreOuput;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.JointspaceFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.PlaneContactStateCommandPool;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage.EndEffector;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage.LoadBearingRequest;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.ConvexPolygonShrinker;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachines.State;
import us.ihmc.robotics.stateMachines.StateChangedListener;
import us.ihmc.robotics.stateMachines.StateMachine;
import us.ihmc.robotics.stateMachines.StateTransition;
import us.ihmc.robotics.stateMachines.StateTransitionAction;
import us.ihmc.robotics.stateMachines.StateTransitionCondition;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.tools.io.printing.PrintTools;

public class WalkingHighLevelHumanoidController extends AbstractHighLevelHumanoidControlPattern
{
   private final static HighLevelState controllerState = HighLevelState.WALKING;

   private final PushRecoveryControlModule pushRecoveryModule;

   private final static boolean DEBUG = false;

   private final StateMachine<WalkingState> stateMachine;

   private final BalanceManager balanceManager;
   private final CenterOfMassHeightManager comHeightManager;

   private final BooleanYoVariable resetIntegratorsAfterSwing = new BooleanYoVariable("resetIntegratorsAfterSwing", registry);
   private final BooleanYoVariable alwaysIntegrateAnkleAcceleration = new BooleanYoVariable("alwaysIntegrateAnkleAcceleration", registry);

   private final BooleanYoVariable hasMinimumTimePassed = new BooleanYoVariable("hasMinimumTimePassed", registry);
   private final DoubleYoVariable minimumSwingFraction = new DoubleYoVariable("minimumSwingFraction", registry);

   private final SideDependentList<FootSwitchInterface> footSwitches;

   private final DoubleYoVariable maxICPErrorBeforeSingleSupportX = new DoubleYoVariable("maxICPErrorBeforeSingleSupportX", registry);
   private final DoubleYoVariable maxICPErrorBeforeSingleSupportY = new DoubleYoVariable("maxICPErrorBeforeSingleSupportY", registry);

   private final WalkingMessageHandler walkingMessageHandler;
   private final BooleanYoVariable abortWalkingRequested = new BooleanYoVariable("requestAbortWalking", registry);

   private final EnumYoVariable<RobotSide> supportLeg;
   private final EnumYoVariable<RobotSide> trailingLeg = new EnumYoVariable<RobotSide>("trailingLeg", "", registry, RobotSide.class, true);
   private final EnumYoVariable<RobotSide> lastPlantedLeg = new EnumYoVariable<RobotSide>("lastPlantedLeg", "", registry, RobotSide.class, true);

   private final BipedSupportPolygons bipedSupportPolygons;

   private final FramePoint tmpFramePoint = new FramePoint(worldFrame);
   private final FramePoint2d tempFramePoint2d = new FramePoint2d();

   private final DoubleYoVariable controlledCoMHeightAcceleration;

   private final BooleanYoVariable isPerformingToeOff = new BooleanYoVariable("isPerformingToeOff", registry);

   private final BooleanYoVariable preparingForLocomotion = new BooleanYoVariable("preparingForLocomotion", registry);
   private final DoubleYoVariable timeToGetPreparedForLocomotion = new DoubleYoVariable("timeToGetPreparedForLocomotion", registry);
   private final DoubleYoVariable preparingForLocomotionStartTime = new DoubleYoVariable("preparingForLocomotionStartTime", registry);
   private final BooleanYoVariable doPrepareManipulationForLocomotion = new BooleanYoVariable("doPrepareManipulationForLocomotion", registry);
   private final BooleanYoVariable doPreparePelvisForLocomotion = new BooleanYoVariable("doPreparePelvisForLocomotion", registry);

   private final BooleanYoVariable isInFlamingoStance = new BooleanYoVariable("isInFlamingoStance", registry);

   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final BooleanYoVariable hasWalkingControllerBeenInitialized = new BooleanYoVariable("hasWalkingControllerBeenInitialized", registry);

   private final DoubleYoVariable remainingSwingTimeAccordingToPlan = new DoubleYoVariable("remainingSwingTimeAccordingToPlan", registry);
   private final DoubleYoVariable estimatedRemainingSwingTimeUnderDisturbance = new DoubleYoVariable("estimatedRemainingSwingTimeUnderDisturbance", registry);
   private final DoubleYoVariable icpErrorThresholdToSpeedUpSwing = new DoubleYoVariable("icpErrorThresholdToSpeedUpSwing", registry);

   private final BooleanYoVariable loadFoot = new BooleanYoVariable("loadFoot", registry);
   private final DoubleYoVariable loadFootStartTime = new DoubleYoVariable("loadFootStartTime", registry);
   private final DoubleYoVariable loadFootDuration = new DoubleYoVariable("loadFootDuration", registry);
   private final DoubleYoVariable loadFootTransferDuration = new DoubleYoVariable("loadFootTransferDuration", registry);

   private final BooleanYoVariable hasICPPlannerBeenInitializedAtStart = new BooleanYoVariable("hasICPPlannerBeenInitializedAtStart", registry);

   private final ConvexPolygonShrinker convexPolygonShrinker = new ConvexPolygonShrinker();
   private final FrameConvexPolygon2d shrunkSupportPolygon = new FrameConvexPolygon2d();
   private final DoubleYoVariable distanceToShrinkSupportPolygonWhenHoldingCurrent = new DoubleYoVariable("distanceToShrinkSupportPolygonWhenHoldingCurrent",
         registry);
   private final BooleanYoVariable holdICPToCurrentCoMLocationInNextDoubleSupport = new BooleanYoVariable("holdICPToCurrentCoMLocationInNextDoubleSupport",
         registry);

   private final BooleanYoVariable enablePushRecoveryOnFailure = new BooleanYoVariable("enablePushRecoveryOnFailure", registry);

   private final BooleanYoVariable isAutomaticManipulationAbortEnabled = new BooleanYoVariable("isAutomaticManipulationAbortEnabled", registry);
   private final BooleanYoVariable hasManipulationBeenAborted = new BooleanYoVariable("hasManipulationBeenAborted", registry);
   private final DoubleYoVariable icpErrorThresholdToAbortManipulation = new DoubleYoVariable("icpErrorThresholdToAbortManipulation", registry);
   private final DoubleYoVariable minimumDurationBetweenTwoManipulationAborts = new DoubleYoVariable("minimumDurationBetweenTwoManipulationAborts", registry);
   private final DoubleYoVariable timeOfLastManipulationAbortRequest = new DoubleYoVariable("timeOfLastManipulationAbortRequest", registry);
   private final DoubleYoVariable manipulationIgnoreInputsDurationAfterAbort = new DoubleYoVariable("manipulationIgnoreInputsDurationAfterAbort", registry);

   private final ControllerCommandInputManager commandInputManager;
   private final ControllerStatusOutputManager statusOutputManager;
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(true);
   private ControllerCoreOuput controllerCoreOuput;

   public WalkingHighLevelHumanoidController(ControllerCommandInputManager commandInputManager, ControllerStatusOutputManager statusOutputManager,
         VariousWalkingManagers variousWalkingManagers, WalkingControllerParameters walkingControllerParameters,
         MomentumBasedController momentumBasedController)
   {
      super(variousWalkingManagers, momentumBasedController, walkingControllerParameters, controllerState);

      balanceManager = variousWalkingManagers.getBalanceManager();
      comHeightManager = variousWalkingManagers.getCenterOfMassHeightManager();

      this.commandInputManager = commandInputManager;
      this.statusOutputManager = statusOutputManager;

      hasWalkingControllerBeenInitialized.set(false);

      timeToGetPreparedForLocomotion.set(walkingControllerParameters.getTimeToGetPreparedForLocomotion());
      icpErrorThresholdToSpeedUpSwing.set(walkingControllerParameters.getICPErrorThresholdToSpeedUpSwing());

      doPrepareManipulationForLocomotion.set(walkingControllerParameters.doPrepareManipulationForLocomotion());
      doPreparePelvisForLocomotion.set(true);

      isAutomaticManipulationAbortEnabled.set(walkingControllerParameters.allowAutomaticManipulationAbort());
      icpErrorThresholdToAbortManipulation.set(0.04);
      minimumDurationBetweenTwoManipulationAborts.set(5.0);
      manipulationIgnoreInputsDurationAfterAbort.set(2.0);
      timeOfLastManipulationAbortRequest.set(Double.NEGATIVE_INFINITY);

      failureDetectionControlModule = new WalkingFailureDetectionControlModule(momentumBasedController.getContactableFeet(), registry);

      supportLeg = balanceManager.getYoSupportLeg();
      bipedSupportPolygons = balanceManager.getBipedSupportPolygons();
      controlledCoMHeightAcceleration = balanceManager.getControlledCoMHeightAcceleration();

      this.footSwitches = momentumBasedController.getFootSwitches();

      double defaultTransferTime = walkingControllerParameters.getDefaultTransferTime();
      double defaultSwingTime = walkingControllerParameters.getDefaultSwingTime();
      walkingMessageHandler = new WalkingMessageHandler(defaultTransferTime, defaultSwingTime, feet, statusOutputManager, yoGraphicsListRegistry, registry);

      String namePrefix = "walking";

      this.stateMachine = new StateMachine<WalkingState>(namePrefix + "State", namePrefix + "SwitchTime", WalkingState.class, yoTime, registry); // this is used by name, and it is ugly.

      pushRecoveryModule = new PushRecoveryControlModule(bipedSupportPolygons, momentumBasedController, walkingControllerParameters, registry);

      setupStateMachine();

      loadFoot.set(false);
      loadFootDuration.set(1.2);
      loadFootTransferDuration.set(0.8);

      maxICPErrorBeforeSingleSupportX.set(walkingControllerParameters.getMaxICPErrorBeforeSingleSupportX());
      maxICPErrorBeforeSingleSupportY.set(walkingControllerParameters.getMaxICPErrorBeforeSingleSupportY());

      minimumSwingFraction.set(0.5); // 0.8);

      resetIntegratorsAfterSwing.set(true);
      alwaysIntegrateAnkleAcceleration.set(true);

      distanceToShrinkSupportPolygonWhenHoldingCurrent.set(0.08);
   }

   private void setupStateMachine()
   {
      DoubleSupportState doubleSupportState = new DoubleSupportState(null);
      SideDependentList<State<WalkingState>> transferStates = new SideDependentList<>();
      SideDependentList<StateTransition<WalkingState>> fromFallingToSingleSupportsTransitions = new SideDependentList<>();

      stateMachine.addState(doubleSupportState);

      ResetICPTrajectoryAction resetICPTrajectoryAction = new ResetICPTrajectoryAction();
      for (RobotSide robotSide : RobotSide.values)
      {
         WalkingState doubleSupportStateEnum = doubleSupportState.getStateEnum();
         WalkingState singleSupportStateEnum = WalkingState.getSingleSupportState(robotSide);
         WalkingState transferStateEnum = WalkingState.getTransferState(robotSide);
         WalkingState oppTransferStateEnum = WalkingState.getTransferState(robotSide.getOppositeSide());

         State<WalkingState> transferState = new DoubleSupportState(robotSide);
         State<WalkingState> singleSupportState = new SingleSupportState(robotSide);

         transferStates.put(robotSide, transferState);

         StopWalkingCondition stopWalkingCondition = new StopWalkingCondition(robotSide);
         DoneWithTransferCondition doneWithTransferCondition = new DoneWithTransferCondition(robotSide);
         SingleSupportToTransferToCondition singleSupportToTransferToOppositeSideCondition = new SingleSupportToTransferToCondition(robotSide);
         SingleSupportToTransferToCondition singleSupportToTransferToSameSideCondition = new SingleSupportToTransferToCondition(robotSide.getOppositeSide());
         StartWalkingCondition startWalkingCondition = new StartWalkingCondition(robotSide);
         FlamingoStanceCondition flamingoStanceCondition = new FlamingoStanceCondition(robotSide);

         StateTransition<WalkingState> toDoubleSupport = new StateTransition<WalkingState>(doubleSupportStateEnum, stopWalkingCondition,
               resetICPTrajectoryAction);
         StateTransition<WalkingState> toSingleSupport = new StateTransition<WalkingState>(singleSupportStateEnum, doneWithTransferCondition);
         //         StateTransition<WalkingState> toDoubleSupport2 = new StateTransition<WalkingState>(doubleSupportStateEnum, stopWalkingCondition, resetICPTrajectoryAction);
         StateTransition<WalkingState> toTransferOppositeSide = new StateTransition<WalkingState>(oppTransferStateEnum,
               singleSupportToTransferToOppositeSideCondition);
         StateTransition<WalkingState> toTransferSameSide = new StateTransition<WalkingState>(transferStateEnum, singleSupportToTransferToSameSideCondition);
         StateTransition<WalkingState> toTransfer = new StateTransition<WalkingState>(transferStateEnum, startWalkingCondition);
         StateTransition<WalkingState> toTransfer2 = new StateTransition<WalkingState>(transferStateEnum, flamingoStanceCondition);

         transferState.addStateTransition(toDoubleSupport);
         transferState.addStateTransition(toSingleSupport);
         //         singleSupportState.addStateTransition(toDoubleSupport2);
         singleSupportState.addStateTransition(toTransferOppositeSide);
         singleSupportState.addStateTransition(toTransferSameSide);
         doubleSupportState.addStateTransition(toTransfer);
         doubleSupportState.addStateTransition(toTransfer2);

         stateMachine.addState(transferState);
         stateMachine.addState(singleSupportState);

         DoubleSupportToSingleSupportConditionForDisturbanceRecovery isFallingFromDoubleSupportCondition = new DoubleSupportToSingleSupportConditionForDisturbanceRecovery(
               robotSide);
         StateTransition<WalkingState> fromFallingToSingleSupport = new StateTransition<WalkingState>(singleSupportStateEnum,
               isFallingFromDoubleSupportCondition);
         fromFallingToSingleSupportsTransitions.put(robotSide, fromFallingToSingleSupport);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         doubleSupportState.addStateTransition(fromFallingToSingleSupportsTransitions.get(robotSide));

         for (RobotSide transferSide : RobotSide.values)
         {
            transferStates.get(transferSide).addStateTransition(fromFallingToSingleSupportsTransitions.get(robotSide));
         }
      }

      stateMachine.attachStateChangedListener(new StateChangedListener<WalkingState>()
      {
         @Override
         public void stateChanged(State<WalkingState> oldState, State<WalkingState> newState, double time)
         {
            momentumBasedController.reportControllerStateChangeToListeners(oldState.getStateEnum(), newState.getStateEnum());
         }
      });
   }

   @Override
   public void setControllerCoreOuput(ControllerCoreOuput controllerCoreOuput)
   {
      this.controllerCoreOuput = controllerCoreOuput;
   }

   public void initialize()
   {
      super.initialize();

      commandInputManager.flushBuffers();

      initializeContacts();

      for (RobotSide robotSide : RobotSide.values)
      {
         controllerCoreOuput.getDesiredCenterOfPressure(footDesiredCoPs.get(robotSide), feet.get(robotSide).getRigidBody());
         momentumBasedController.setDesiredCenterOfPressure(feet.get(robotSide), footDesiredCoPs.get(robotSide));
      }

      balanceManager.disablePelvisXYControl();

      double stepTime = walkingMessageHandler.getStepTime();
      pelvisOrientationManager.setTrajectoryTime(stepTime);

      if (!hasWalkingControllerBeenInitialized.getBooleanValue())
      {
         pelvisOrientationManager.resetOrientationOffset();
         pelvisOrientationManager.setToZeroInMidFeetZUpFrame();
         hasWalkingControllerBeenInitialized.set(true);
      }
      else
      {
         pelvisOrientationManager.resetOrientationOffset();
         pelvisOrientationManager.setToHoldCurrentInWorldFrame();
      }

      if (manipulationControlModule != null)
      {
         manipulationControlModule.holdCurrentArmConfiguration();
      }
      chestOrientationManager.holdCurrentOrientation();

      balanceManager.initialize();
      //      requestICPPlannerToHoldCurrent(); // Not sure if we want to do this. Might cause robot to fall. Might just be better to recenter ICP whenever switching to walking.

      // Need to reset it so the planner will be initialized even when restarting the walking controller.
      hasICPPlannerBeenInitializedAtStart.set(false);
      stateMachine.setCurrentState(WalkingState.DOUBLE_SUPPORT);

      hasWalkingControllerBeenInitialized.set(true);
   }

   public void initializeDesiredHeightToCurrent()
   {
      comHeightManager.initializeDesiredHeightToCurrent();
      feetManager.resetHeightCorrectionParametersForSingularityAvoidance();
   }

   public void requestICPPlannerToHoldCurrentCoM()
   {
      tmpFramePoint.setToZero(referenceFrames.getCenterOfMassFrame());

      balanceManager.updateBipedSupportPolygons();
      FrameConvexPolygon2d supportPolygonInMidFeetZUp = bipedSupportPolygons.getSupportPolygonInMidFeetZUp();
      convexPolygonShrinker.shrinkConstantDistanceInto(supportPolygonInMidFeetZUp, distanceToShrinkSupportPolygonWhenHoldingCurrent.getDoubleValue(),
            shrunkSupportPolygon);

      tmpFramePoint.changeFrame(shrunkSupportPolygon.getReferenceFrame());
      tempFramePoint2d.setByProjectionOntoXYPlaneIncludingFrame(tmpFramePoint);
      shrunkSupportPolygon.orthogonalProjection(tempFramePoint2d);
      tmpFramePoint.setXY(tempFramePoint2d);

      tmpFramePoint.changeFrame(worldFrame);
      balanceManager.holdCurrentICP(yoTime.getDoubleValue(), tmpFramePoint);
   }

   private void initializeContacts()
   {
      momentumBasedController.clearContacts();

      for (RobotSide robotSide : RobotSide.values)
      {
         feetManager.setFlatFootContactState(robotSide);
      }
   }

   private class DoubleSupportState extends State<WalkingState>
   {
      private final RobotSide transferToSide;
      private final FramePoint2d desiredICPLocal = new FramePoint2d();
      private final FrameVector2d desiredICPVelocityLocal = new FrameVector2d();
      private final FramePoint2d capturePoint2d = new FramePoint2d();
      private final FramePoint2d desiredCMP = new FramePoint2d();

      public DoubleSupportState(RobotSide transferToSide)
      {
         super((transferToSide == null) ? WalkingState.DOUBLE_SUPPORT : WalkingState.getTransferState(transferToSide));
         this.transferToSide = transferToSide;
      }

      @Override
      public void doAction()
      {
         //abort walk and clear if should abort
         if (abortWalkingRequested.getBooleanValue())
         {
            walkingMessageHandler.clearFootsteps();
            walkingMessageHandler.reportWalkingAbortRequested();
            abortWalkingRequested.set(false);
         }

         if (!alwaysIntegrateAnkleAcceleration.getBooleanValue())
            doNotIntegrateAnkleAccelerations();

         feetManager.updateContactStatesInDoubleSupport(transferToSide);

         if (transferToSide == null)
         {
            consumeManipulationMessages();
            consumePelvisMessages();
         }

         desiredICPLocal.setToZero(worldFrame);
         desiredICPVelocityLocal.setToZero(worldFrame);

         balanceManager.getDesiredCapturePointPositionAndVelocity(desiredICPLocal, desiredICPVelocityLocal, yoTime.getDoubleValue());

         if (transferToSide == null)
            balanceManager.computePelvisXY(desiredICPLocal, desiredICPVelocityLocal);

         balanceManager.setDesiredICP(desiredICPLocal);
         balanceManager.setDesiredICPVelocity(desiredICPVelocityLocal);

         switchToToeOffIfPossible();

         if (pushRecoveryModule.isEnabled())
         {
            balanceManager.getCapturePoint(capturePoint2d);
            pushRecoveryModule.updateForDoubleSupport(desiredICPLocal, capturePoint2d, balanceManager.getOmega0());
         }

         // Always do this so that when a foot slips or is loaded in the air, the height
         // gets adjusted.
         if (transferToSide != null)
            comHeightManager.setSupportLeg(transferToSide);
         else
            comHeightManager.setSupportLeg(lastPlantedLeg.getEnumValue());

         // Do it only when standing
         if (transferToSide == null)
            handleAutomaticManipulationAbortOnICPError();
      }

      private void handlePelvisTrajectoryMessage(ModifiablePelvisTrajectoryMessage message)
      {
         if (transferToSide != null)
            return;

         pelvisOrientationManager.handlePelvisTrajectoryMessage(message);
         balanceManager.handlePelvisTrajectoryMessage(message);
         comHeightManager.handlePelvisTrajectoryMessage(message);
      }

      private void handleAutomaticManipulationAbortOnICPError()
      {
         if (manipulationControlModule == null)
         {
            return;
         }

         if (commandInputManager.isNewMessageAvailable(ModifiableAutomaticManipulationAbortMessage.class))
         {
            ModifiableAutomaticManipulationAbortMessage message = commandInputManager.pollNewestMessage(ModifiableAutomaticManipulationAbortMessage.class);
            isAutomaticManipulationAbortEnabled.set(message.isEnable());
         }

         if (!isAutomaticManipulationAbortEnabled.getBooleanValue())
         {
            return;
         }

         if (yoTime.getDoubleValue() - timeOfLastManipulationAbortRequest.getDoubleValue() < minimumDurationBetweenTwoManipulationAborts.getDoubleValue())
            return;

         if (capturePoint2d.distance(desiredICPLocal) > icpErrorThresholdToAbortManipulation.getDoubleValue())
         {
            hasManipulationBeenAborted.set(true);
            manipulationControlModule.freeze();
            timeOfLastManipulationAbortRequest.set(yoTime.getDoubleValue());

            statusOutputManager.reportManipulationAborted();
         }
         else
         {
            hasManipulationBeenAborted.set(false);
         }
      }

      public void switchToToeOffIfPossible()
      {
         // the only case left for determining the contact state of the trailing foot
         if (!isPerformingToeOff.getBooleanValue() && transferToSide != null)
         {
            RobotSide trailingLeg = transferToSide.getOppositeSide();

            double predictedToeOffDuration = balanceManager.computeAndReturnTimeRemaining(yoTime.getDoubleValue());

            balanceManager.getDesiredCMP(desiredCMP);
            balanceManager.getDesiredICP(desiredICPLocal);
            balanceManager.getCapturePoint(capturePoint2d);
            boolean doToeOff = feetManager.checkIfToeOffSafe(trailingLeg, desiredCMP, desiredICPLocal, capturePoint2d);

            if (doToeOff)
            {
               feetManager.requestToeOff(trailingLeg, predictedToeOffDuration);
               balanceManager.updateBipedSupportPolygons(); // need to always update biped support polygons after a change to the contact states
               isPerformingToeOff.set(true);
            }
         }
      }

      private TransferToAndNextFootstepsData createTransferToAndNextFootstepDataForDoubleSupport(RobotSide transferToSide)
      {
         Footstep transferFromFootstep = walkingMessageHandler.getFootstepAtCurrentLocation(transferToSide.getOppositeSide());
         Footstep transferToFootstep = walkingMessageHandler.getFootstepAtCurrentLocation(transferToSide);

         Footstep nextFootstep;

         nextFootstep = walkingMessageHandler.peek(0);

         TransferToAndNextFootstepsData transferToAndNextFootstepsData = new TransferToAndNextFootstepsData();
         transferToAndNextFootstepsData.setTransferFromFootstep(transferFromFootstep);
         transferToAndNextFootstepsData.setTransferToFootstep(transferToFootstep);
         transferToAndNextFootstepsData.setTransferToSide(transferToSide);
         transferToAndNextFootstepsData.setNextFootstep(nextFootstep);

         return transferToAndNextFootstepsData;
      }

      @Override
      public void doTransitionIntoAction()
      {
         preparingForLocomotion.set(false);

         balanceManager.clearPlan();
         supportLeg.set(null);
         isPerformingToeOff.set(false);
         feetManager.initializeContactStatesForDoubleSupport(transferToSide);
         balanceManager.updateBipedSupportPolygons(); // need to always update biped support polygons after a change to the contact states

         commandInputManager.flushMessages(ModifiablePelvisTrajectoryMessage.class);

         boolean isInDoubleSupport = supportLeg.getEnumValue() == null;
         if (isInDoubleSupport && !walkingMessageHandler.hasUpcomingFootsteps() && !walkingMessageHandler.isWalkingPaused())
         {
            PrintTools.debug(DEBUG, this, "WALKING COMPLETE");
            walkingMessageHandler.reportWalkingComplete();
            balanceManager.enablePelvisXYControl();
         }

         if (transferToSide != null)
            trailingLeg.set(transferToSide.getOppositeSide());
         else
            trailingLeg.set(null);


         if (transferToSide == null)
         {
            failureDetectionControlModule.setNextFootstep(null);
            momentumBasedController.reportChangeOfRobotMotionStatus(RobotMotionStatus.STANDING);
         }
         else
            failureDetectionControlModule.setNextFootstep(walkingMessageHandler.peek(0));


         RobotSide transferToSideToUseInFootstepData = transferToSide;
         if (transferToSideToUseInFootstepData == null)
            transferToSideToUseInFootstepData = lastPlantedLeg.getEnumValue();

         boolean hasFootTrajectoryMessage = walkingMessageHandler.hasFootTrajectoryForFlamingoStance();
         if (!hasFootTrajectoryMessage && !comHeightManager.hasBeenInitializedWithNextStep())
         {
            TransferToAndNextFootstepsData transferToAndNextFootstepsDataForDoubleSupport = createTransferToAndNextFootstepDataForDoubleSupport(
                  transferToSideToUseInFootstepData);
            double extraToeOffHeight = 0.0;
            if (transferToSide != null && feetManager.willDoToeOff(null, transferToSide))
               extraToeOffHeight = feetManager.getWalkOnTheEdgesManager().getExtraCoMMaxHeightWithToes();
            comHeightManager.initialize(transferToAndNextFootstepsDataForDoubleSupport, extraToeOffHeight);
         }

         if (pushRecoveryModule.isEnabled())
         {
            pushRecoveryModule.reset();
         }

         double transferTime;

         boolean isPreviousStateDoubleSupport = getPreviousState().getStateEnum() == WalkingState.DOUBLE_SUPPORT;
         if (isPreviousStateDoubleSupport)
         {
            transferTime = balanceManager.getInitialTransferDuration();
         }
         else
         {
            transferTime = walkingMessageHandler.getTransferTime();
         }

         pelvisOrientationManager.setTrajectoryTime(transferTime);

         // Just standing in double support, do nothing
         if (transferToSide == null)
            pelvisOrientationManager.setToHoldCurrentDesiredInMidFeetZUpFrame();
         // Transferring to execute a foot pose, hold current desired in upcoming support foot in case it slips
         else if (hasFootTrajectoryMessage)
            pelvisOrientationManager.setToHoldCurrentDesiredInSupportFoot(transferToSide);
         // Transfer for taking the first step, need to ensure a safe pelvis orientation
         else if (walkingMessageHandler.hasUpcomingFootsteps() && isPreviousStateDoubleSupport)
            pelvisOrientationManager.moveToAverageInSupportFoot(transferToSide);
         // In middle of walking or leaving foot pose, pelvis is good leave it like that.
         else
            pelvisOrientationManager.setToHoldCurrentDesiredInSupportFoot(transferToSide);

         if (walkingMessageHandler.hasUpcomingFootsteps())
         {
            for (int i = 0; i < 3; i++)
               balanceManager.addFootstepToPlan(walkingMessageHandler.peek(i));
            balanceManager.initializeDoubleSupport(yoTime.getDoubleValue(), transferToSide);
         }
         else if (hasFootTrajectoryMessage)
         {
            balanceManager.setSingleSupportTime(Double.POSITIVE_INFINITY);
            balanceManager.addFootstepToPlan(walkingMessageHandler.getFootstepAtCurrentLocation(transferToSide.getOppositeSide()));
         }
         else
         {
            balanceManager.reset(yoTime.getDoubleValue());
         }
      }

      @Override
      public void doTransitionOutOfAction()
      {
         preparingForLocomotion.set(false);
         balanceManager.disablePelvisXYControl();
         feetManager.reset();
         isPerformingToeOff.set(false);

         commandInputManager.flushMessages(ModifiablePelvisTrajectoryMessage.class);

         if (transferToSide == null)
            momentumBasedController.reportChangeOfRobotMotionStatus(RobotMotionStatus.IN_MOTION);
      }
   }

   private Footstep previousDesiredFootstep;

   private class SingleSupportState extends State<WalkingState>
   {
      private final RobotSide swingSide;

      private final FramePoint2d desiredICPLocal = new FramePoint2d();
      private final FrameVector2d desiredICPVelocityLocal = new FrameVector2d();
      private final FramePoint2d capturePoint2d = new FramePoint2d();

      private Footstep nextFootstep;
      private double captureTime;

      private final FramePose actualFootPoseInWorld;

      public SingleSupportState(RobotSide supportSide)
      {
         super(WalkingState.getSingleSupportState(supportSide));
         this.swingSide = supportSide.getOppositeSide();
         actualFootPoseInWorld = new FramePose(worldFrame);
      }

      @Override
      public void doAction()
      {
         integrateAnkleAccelerationsOnSwingLeg(swingSide);

         desiredICPLocal.setToZero(worldFrame);
         desiredICPVelocityLocal.setToZero(worldFrame);

         balanceManager.getCapturePoint(capturePoint2d);

         balanceManager.getDesiredCapturePointPositionAndVelocity(desiredICPLocal, desiredICPVelocityLocal, yoTime.getDoubleValue());

         if (isInFlamingoStance.getBooleanValue())
         {
            feetManager.handleFootTrajectoryMessage(walkingMessageHandler.pollFootTrajectoryForFlamingoStance(swingSide));
         }

         RobotSide supportSide = swingSide.getOppositeSide();

         boolean icpErrorIsTooLarge = capturePoint2d.distance(desiredICPLocal) > icpErrorThresholdToSpeedUpSwing.getDoubleValue();

         if (isInFlamingoStance.getBooleanValue())
         {
            if (icpErrorIsTooLarge)
            {
               FrameConvexPolygon2d supportPolygonInWorld = bipedSupportPolygons.getSupportPolygonInWorld();
               FrameConvexPolygon2d combinedFootPolygon = failureDetectionControlModule.getCombinedFootPolygon();
               if (!supportPolygonInWorld.isPointInside(capturePoint2d, 2.0e-2) && combinedFootPolygon.isPointInside(capturePoint2d))
               {
                  feetManager.requestMoveStraightTouchdownForDisturbanceRecovery(swingSide);
                  initiateFootLoadingProcedure(swingSide);
                  holdICPToCurrentCoMLocationInNextDoubleSupport.set(true);
               }
            }
         }
         else if (pushRecoveryModule.isEnabled())
         {
            balanceManager.getCapturePoint(capturePoint2d);
            pushRecoveryModule.updateForSingleSupport(desiredICPLocal, capturePoint2d, balanceManager.getOmega0());

            double estimatedTimeRemaining = balanceManager.computeAndReturnTimeRemaining(yoTime.getDoubleValue());
            boolean footstepHasBeenAdjusted = pushRecoveryModule.checkAndUpdateFootstep(estimatedTimeRemaining, nextFootstep);
            if (footstepHasBeenAdjusted)
            {
               failureDetectionControlModule.setNextFootstep(nextFootstep);
               updateFootstepParameters();

               captureTime = stateMachine.timeInCurrentState();
               feetManager.replanSwingTrajectory(swingSide, nextFootstep, estimatedTimeRemaining);

               walkingMessageHandler.reportWalkingAbortRequested();
               walkingMessageHandler.clearFootsteps();

               balanceManager.clearPlan();
               balanceManager.addFootstepToPlan(nextFootstep);
               balanceManager.updatePlanForSingleSupportDisturbances(yoTime.getDoubleValue());
               holdICPToCurrentCoMLocationInNextDoubleSupport.set(true);
            }
         }

         if (icpErrorIsTooLarge || pushRecoveryModule.isRecovering())
         {
            requestSwingSpeedUpIfNeeded();
         }

         if (isInFlamingoStance.getBooleanValue())
         {
            balanceManager.computePelvisXY(desiredICPLocal, desiredICPVelocityLocal);
            consumePelvisMessages();
         }
         else
         {
            walkingMessageHandler.clearFootTrajectory();
         }

         balanceManager.setDesiredICP(desiredICPLocal);
         balanceManager.setDesiredICPVelocity(desiredICPVelocityLocal);

         if ((stateMachine.timeInCurrentState() - captureTime < 0.5 * walkingMessageHandler.getSwingTime())
               && feetManager.isInSingularityNeighborhood(swingSide))
         {
            feetManager.doSingularityEscape(swingSide);
         }

         if (feetManager.doToeOffIfPossibleInSingleSupport())
         {
            boolean willDoToeOff = feetManager.willDoToeOff(nextFootstep, swingSide);

            if (feetManager.isInFlatSupportState(supportSide) && willDoToeOff && balanceManager.isOnExitCMP())
            {
               balanceManager.getNextExitCMP(nextExitCMP);
               nextExitCMP.changeFrame(feet.get(supportSide).getSoleFrame());
               toeOffContactPoint.setByProjectionOntoXYPlaneIncludingFrame(nextExitCMP);
               feetManager.registerDesiredContactPointForToeOff(supportSide, toeOffContactPoint);
               double predictedToeOffDuration = balanceManager.computeAndReturnTimeRemaining(yoTime.getDoubleValue()) + walkingMessageHandler.getTransferTime();
               feetManager.requestToeOff(supportSide, predictedToeOffDuration);
            }
         }
      }

      private void handlePelvisTrajectoryMessage(ModifiablePelvisTrajectoryMessage message)
      {
         if (!isInFlamingoStance.getBooleanValue() || loadFoot.getBooleanValue())
            return;

         pelvisOrientationManager.handlePelvisTrajectoryMessage(message);
         balanceManager.handlePelvisTrajectoryMessage(message);
      }

      private void handleEndEffectorLoadBearingRequest(ModifiableEndEffectorLoadBearingMessage message)
      {
         if (isInFlamingoStance.getBooleanValue() && message.getRequest(swingSide, EndEffector.FOOT) == LoadBearingRequest.LOAD)
            initiateFootLoadingProcedure(swingSide);
      }

      private final FramePoint nextExitCMP = new FramePoint();
      private final FramePoint2d toeOffContactPoint = new FramePoint2d();

      private void requestSwingSpeedUpIfNeeded()
      {
         remainingSwingTimeAccordingToPlan.set(balanceManager.computeAndReturnTimeRemaining(yoTime.getDoubleValue()));
         estimatedRemainingSwingTimeUnderDisturbance.set(balanceManager.estimateTimeRemainingForStateUnderDisturbance(yoTime.getDoubleValue()));

         if (estimatedRemainingSwingTimeUnderDisturbance.getDoubleValue() > 1.0e-3)
         {
            double swingSpeedUpFactor = remainingSwingTimeAccordingToPlan.getDoubleValue() / estimatedRemainingSwingTimeUnderDisturbance.getDoubleValue();
            feetManager.requestSwingSpeedUp(swingSide, swingSpeedUpFactor);
         }
         else if (remainingSwingTimeAccordingToPlan.getDoubleValue() > 1.0e-3)
         {
            feetManager.requestSwingSpeedUp(swingSide, Double.POSITIVE_INFINITY);
         }
      }

      @Override
      public void doTransitionIntoAction()
      {
         lastPlantedLeg.set(swingSide.getOppositeSide());
         captureTime = 0.0;
         trailingLeg.set(null);

         RobotSide supportSide = swingSide.getOppositeSide();
         supportLeg.set(supportSide);
         balanceManager.clearPlan();

         footSwitches.get(swingSide).reset();

         if (pushRecoveryModule.isEnabled() && pushRecoveryModule.isRecoveringFromDoubleSupportFall())
         {
            nextFootstep = pushRecoveryModule.createFootstepForRecoveringFromDisturbance(swingSide, walkingMessageHandler.getSwingTime());
            nextFootstep.setTrajectoryType(TrajectoryType.PUSH_RECOVERY);
            walkingMessageHandler.reportWalkingAbortRequested();
            walkingMessageHandler.clearFootsteps();
         }
         else
         {
            nextFootstep = walkingMessageHandler.poll();
         }

         if (nextFootstep != null)
         {
            feetManager.requestSwing(swingSide, nextFootstep, walkingMessageHandler.getSwingTime());
            walkingMessageHandler.reportFootstepStarted(swingSide);
         }
         else if (walkingMessageHandler.hasFootTrajectoryForFlamingoStance(swingSide))
         {
            feetManager.handleFootTrajectoryMessage(walkingMessageHandler.pollFootTrajectoryForFlamingoStance(swingSide));
            isInFlamingoStance.set(true);
            balanceManager.enablePelvisXYControl();
         }

         if (DEBUG)
            System.out.println("WalkingHighLevelHumanoidController: enteringSingleSupportState");

         if (nextFootstep != null)
         {
            updateFootstepParameters();

            updateICPPlannerTimesAndOmega0();
            balanceManager.addFootstepToPlan(nextFootstep);
            balanceManager.addFootstepToPlan(walkingMessageHandler.peek(0));
            balanceManager.addFootstepToPlan(walkingMessageHandler.peek(1));
            balanceManager.initializeSingleSupport(yoTime.getDoubleValue(), supportSide);

            if (pushRecoveryModule.isEnabled() && pushRecoveryModule.isRecoveringFromDoubleSupportFall())
            {
               balanceManager.updatePlanForSingleSupportDisturbances(yoTime.getDoubleValue());
               holdICPToCurrentCoMLocationInNextDoubleSupport.set(true);
            }
         }
         else
         {
            pelvisOrientationManager.setToHoldCurrentDesiredInSupportFoot(supportSide);
            comHeightManager.setSupportLeg(supportSide);
         }

         loadFoot.set(false);
      }

      private void updateFootstepParameters()
      {
         pelvisOrientationManager.setTrajectoryTime(walkingMessageHandler.getSwingTime());
         pelvisOrientationManager.setWithUpcomingFootstep(nextFootstep);

         TransferToAndNextFootstepsData transferToAndNextFootstepsData = createTransferToAndNextFootstepDataForSingleSupport(nextFootstep, swingSide);
         transferToAndNextFootstepsData.setTransferFromDesiredFootstep(previousDesiredFootstep);
         double extraToeOffHeight = 0.0;
         if (feetManager.willDoToeOff(nextFootstep, swingSide))
            extraToeOffHeight = feetManager.getWalkOnTheEdgesManager().getExtraCoMMaxHeightWithToes();
         comHeightManager.initialize(transferToAndNextFootstepsData, extraToeOffHeight);

         // Update the contact states based on the footstep. If the footstep doesn't have any predicted contact points, then use the default ones in the ContactablePlaneBodys.
         momentumBasedController.updateContactPointsForUpcomingFootstep(nextFootstep);
         balanceManager.updateBipedSupportPolygons();

      }

      @Override
      public void doTransitionOutOfAction()
      {
         if (DEBUG)
            System.out.println("WalkingHighLevelController: leavingDoubleSupportState");

         if (!isInFlamingoStance.getBooleanValue())
         {
            actualFootPoseInWorld.setToZero(fullRobotModel.getEndEffectorFrame(swingSide, LimbName.LEG)); // changed Here Nicolas
            actualFootPoseInWorld.changeFrame(worldFrame);
            walkingMessageHandler.reportFootstepCompleted(swingSide, actualFootPoseInWorld);
         }
         else
         {
            balanceManager.disablePelvisXYControl();
            isInFlamingoStance.set(false);
         }

         if (pushRecoveryModule.isEnabled())
         {
            captureTime = 0.0;
            pushRecoveryModule.reset();
         }

         resetLoadedLegIntegrators(swingSide);

         previousDesiredFootstep = nextFootstep;
      }
   }

   private void initiateFootLoadingProcedure(RobotSide swingSide)
   {
      loadFoot.set(true);
      loadFootStartTime.set(yoTime.getDoubleValue());
      balanceManager.setSingleSupportTime(loadFootDuration.getDoubleValue());
      balanceManager.setDoubleSupportTime(loadFootTransferDuration.getDoubleValue());
      balanceManager.clearPlan();
      balanceManager.addFootstepToPlan(walkingMessageHandler.getFootstepAtCurrentLocation(swingSide));
      balanceManager.initializeSingleSupport(yoTime.getDoubleValue(), supportLeg.getEnumValue());

      balanceManager.freezePelvisXYControl();
   }

   private class DoubleSupportToSingleSupportConditionForDisturbanceRecovery implements StateTransitionCondition
   {
      private final RobotSide transferToSide;

      public DoubleSupportToSingleSupportConditionForDisturbanceRecovery(RobotSide robotSide)
      {
         this.transferToSide = robotSide;
      }

      @Override
      public boolean checkCondition()
      {
         if (!pushRecoveryModule.isEnabled())
            return false;

         RobotSide suggestedSwingSide = pushRecoveryModule.isRobotFallingFromDoubleSupport(stateMachine.timeInCurrentState());
         boolean isRobotFalling = suggestedSwingSide != null;

         if (!isRobotFalling)
            return false;

         boolean switchToSingleSupport = transferToSide != suggestedSwingSide;

         if (switchToSingleSupport)
            pushRecoveryModule.initializeParametersForDoubleSupportPushRecovery();

         return switchToSingleSupport;
      }
   }

   public class StartWalkingCondition implements StateTransitionCondition
   {
      private final RobotSide transferToSide;

      public StartWalkingCondition(RobotSide robotSide)
      {
         this.transferToSide = robotSide;
      }

      public boolean checkCondition()
      {
         boolean doubleSupportTimeHasPassed = stateMachine.timeInCurrentState() > walkingMessageHandler.getTransferTime();
         boolean transferringToThisRobotSide;
         if (walkingMessageHandler.hasUpcomingFootsteps())
            transferringToThisRobotSide = transferToSide == walkingMessageHandler.peek(0).getRobotSide().getOppositeSide();
         else
            transferringToThisRobotSide = false;

         boolean shouldStartWalking = transferringToThisRobotSide && doubleSupportTimeHasPassed;
         boolean isPreparedToWalk = false;

         if (shouldStartWalking)
         {
            if (!preparingForLocomotion.getBooleanValue())
            {
               preparingForLocomotion.set(true);
               preparingForLocomotionStartTime.set(yoTime.getDoubleValue());

               if (manipulationControlModule != null && doPrepareManipulationForLocomotion.getBooleanValue())
                  manipulationControlModule.prepareForLocomotion();

               if (pelvisOrientationManager != null && doPreparePelvisForLocomotion.getBooleanValue())
                  pelvisOrientationManager.prepareForLocomotion();
            }

            isPreparedToWalk = yoTime.getDoubleValue() - preparingForLocomotionStartTime.getDoubleValue() > timeToGetPreparedForLocomotion.getDoubleValue();
         }

         return shouldStartWalking && isPreparedToWalk;
      }
   }

   public class FlamingoStanceCondition implements StateTransitionCondition
   {
      private final RobotSide transferToSide;

      public FlamingoStanceCondition(RobotSide robotSide)
      {
         this.transferToSide = robotSide;
      }

      public boolean checkCondition()
      {
         boolean doubleSupportTimeHasPassed = stateMachine.timeInCurrentState() > walkingMessageHandler.getTransferTime();
         boolean hasNewFootTrajectoryMessage = walkingMessageHandler.hasFootTrajectoryForFlamingoStance(transferToSide.getOppositeSide());
         boolean transferringToThisRobotSide = hasNewFootTrajectoryMessage;

         return transferringToThisRobotSide && doubleSupportTimeHasPassed;
      }
   }

   public class DoneWithTransferCondition implements StateTransitionCondition
   {
      private final RobotSide robotSide;
      private final FramePoint2d capturePoint2d = new FramePoint2d();
      private final FramePoint2d desiredICP2d = new FramePoint2d();
      private final FramePoint2d icpError2d = new FramePoint2d();

      public DoneWithTransferCondition(RobotSide robotSide)
      {
         this.robotSide = robotSide;
      }

      public boolean checkCondition()
      {
         boolean icpTrajectoryIsDone = balanceManager.isDone(yoTime.getDoubleValue());

         if (!icpTrajectoryIsDone)
            return false;

         balanceManager.getCapturePoint(capturePoint2d);
         balanceManager.getDesiredICP(desiredICP2d);

         capturePoint2d.changeFrame(referenceFrames.getAnkleZUpFrame(robotSide));
         desiredICP2d.changeFrame(referenceFrames.getAnkleZUpFrame(robotSide));

         icpError2d.setIncludingFrame(desiredICP2d);
         icpError2d.sub(capturePoint2d);

         double ellipticErrorSquared = MathTools.square(icpError2d.getX() / maxICPErrorBeforeSingleSupportX.getDoubleValue())
               + MathTools.square(icpError2d.getY() / maxICPErrorBeforeSingleSupportY.getDoubleValue());
         boolean closeEnough = ellipticErrorSquared < 1.0;

         return closeEnough;
      }
   }

   private class SingleSupportToTransferToCondition extends DoneWithSingleSupportCondition
   {
      private final RobotSide robotSide;

      public SingleSupportToTransferToCondition(RobotSide robotSide)
      {
         super();

         this.robotSide = robotSide;
      }

      public boolean checkCondition()
      {
         Footstep nextFootstep = walkingMessageHandler.peek(0);
         if (nextFootstep == null)
            return super.checkCondition();

         if (this.robotSide != nextFootstep.getRobotSide())
            return false;

         boolean condition = super.checkCondition();

         return condition;
      }
   }

   private class DoneWithSingleSupportCondition implements StateTransitionCondition
   {
      public DoneWithSingleSupportCondition()
      {
      }

      public boolean checkCondition()
      {
         RobotSide swingSide = supportLeg.getEnumValue().getOppositeSide();
         hasMinimumTimePassed.set(hasMinimumTimePassed());

         if (!hasMinimumTimePassed.getBooleanValue())
            return false;

         boolean finishSingleSupportWhenICPPlannerIsDone = walkingControllerParameters.finishSingleSupportWhenICPPlannerIsDone();// || pushRecoveryModule.isRecovering();

         if (finishSingleSupportWhenICPPlannerIsDone && !isInFlamingoStance.getBooleanValue())
         {
            if (balanceManager.isDone(yoTime.getDoubleValue()))
               return true;
         }

         if (loadFoot.getBooleanValue() && (yoTime.getDoubleValue() > loadFootStartTime.getDoubleValue() + loadFootDuration.getDoubleValue()))
         {
            loadFoot.set(false);
            return true;
         }

         return hasMinimumTimePassed.getBooleanValue() && footSwitches.get(swingSide).hasFootHitGround();
      }

      private boolean hasMinimumTimePassed()
      {
         double minimumSwingTime;
         if (pushRecoveryModule.isEnabled() && pushRecoveryModule.isRecoveringFromDoubleSupportFall())
            minimumSwingTime = 0.15;
         else
            minimumSwingTime = walkingMessageHandler.getSwingTime() * minimumSwingFraction.getDoubleValue();

         return stateMachine.timeInCurrentState() > minimumSwingTime;
      }
   }

   private class StopWalkingCondition extends DoneWithSingleSupportCondition
   {
      private final RobotSide robotSide;

      public StopWalkingCondition(RobotSide robotSide)
      {
         super();

         this.robotSide = robotSide;
      }

      public boolean checkCondition()
      {
         if (abortWalkingRequested.getBooleanValue())
         {
            return true;
         }

         boolean isInDoubleSupport = supportLeg.getEnumValue() == null;
         if (isInDoubleSupport)
            System.out.println();
         boolean noMoreFootstepsForThisSide = !walkingMessageHandler.isNextFootstepFor(robotSide.getOppositeSide());
         boolean noMoreFootTrajectoryMessages = !walkingMessageHandler.hasFootTrajectoryForFlamingoStance(robotSide.getOppositeSide());
         boolean readyToStopWalking = noMoreFootstepsForThisSide && noMoreFootTrajectoryMessages && (isInDoubleSupport || super.checkCondition());
         return readyToStopWalking;
      }
   }

   public class ResetICPTrajectoryAction implements StateTransitionAction
   {
      public void doTransitionAction()
      {
         if (holdICPToCurrentCoMLocationInNextDoubleSupport.getBooleanValue())
         {
            requestICPPlannerToHoldCurrentCoM();
            holdICPToCurrentCoMLocationInNextDoubleSupport.set(false);
         }

         balanceManager.reset(yoTime.getDoubleValue());
      }
   }

   private void updateICPPlannerTimesAndOmega0()
   {
      balanceManager.setDoubleSupportTime(walkingMessageHandler.getTransferTime());
      balanceManager.setSingleSupportTime(walkingMessageHandler.getSwingTime());
   }

   private TransferToAndNextFootstepsData createTransferToAndNextFootstepDataForSingleSupport(Footstep transferToFootstep, RobotSide swingSide)
   {
      TransferToAndNextFootstepsData transferToAndNextFootstepsData = new TransferToAndNextFootstepsData();

      Footstep transferFromFootstep = walkingMessageHandler.getFootstepAtCurrentLocation(swingSide.getOppositeSide());

      transferToAndNextFootstepsData.setTransferFromFootstep(transferFromFootstep);
      transferToAndNextFootstepsData.setTransferToFootstep(transferToFootstep);

      transferToAndNextFootstepsData.setTransferToSide(swingSide);
      transferToAndNextFootstepsData.setNextFootstep(walkingMessageHandler.peek(0));

      return transferToAndNextFootstepsData;
   }

   private final FrameVector2d desiredICPVelocityAsFrameVector = new FrameVector2d();

   private final SideDependentList<FramePoint2d> footDesiredCoPs = new SideDependentList<FramePoint2d>(new FramePoint2d(), new FramePoint2d());
   private final PlaneContactStateCommandPool planeContactStateCommandPool = new PlaneContactStateCommandPool();
   private final FramePoint2d capturePoint2d = new FramePoint2d();
   private final FramePoint2d desiredCapturePoint2d = new FramePoint2d();

   public void doMotionControl()
   {
      consumeHeadMessages();
      consumeChestMessages();
      consumePelvisHeightMessages();
      consumeGoHomeMessages();
      consumeEndEffectorLoadBearingMessages();
      consumeStopAllTrajectoryMessages();
      consumeFootTrajectoryMessages();
      consumeAbortWalkingMessages();

      balanceManager.getCapturePoint(capturePoint2d);
      balanceManager.getDesiredICP(desiredCapturePoint2d);
      failureDetectionControlModule.checkIfRobotIsFalling(capturePoint2d, desiredCapturePoint2d);
      if (failureDetectionControlModule.isRobotFalling())
      {
         if (enablePushRecoveryOnFailure.getBooleanValue() && !pushRecoveryModule.isEnabled())
         {
            pushRecoveryModule.setIsEnabled(true);
         }
         else if (!pushRecoveryModule.isEnabled() || pushRecoveryModule.isCaptureRegionEmpty())
         {
            momentumBasedController.reportControllerFailureToListeners(failureDetectionControlModule.getFallingDirection());
         }
      }

      if (enablePushRecoveryOnFailure.getBooleanValue())
      {
         if (pushRecoveryModule.isEnabled() && pushRecoveryModule.isRobotBackToSafeState())
            pushRecoveryModule.setIsEnabled(false);
      }

      momentumBasedController.doPrioritaryControl();
      super.callUpdatables();

      balanceManager.update();

      stateMachine.checkTransitionConditions();
      stateMachine.doAction();

      balanceManager.getDesiredICPVelocity(desiredICPVelocityAsFrameVector);
      boolean isInDoubleSupport = supportLeg.getEnumValue() == null;
      double omega0 = balanceManager.getOmega0();
      boolean isRecoveringFromPush = pushRecoveryModule.isRecovering();
      controlledCoMHeightAcceleration.set(comHeightManager.computeDesiredCoMHeightAcceleration(desiredICPVelocityAsFrameVector, isInDoubleSupport, omega0,
            trailingLeg.getEnumValue(), isRecoveringFromPush, feetManager));

      doFootControl();
      doArmControl();
      doHeadControl();

      //    doCoMControl(); //TODO: Should we be doing this too?
      doChestControl();
      doCapturePointBasedControl();
      doPelvisControl();
      JointspaceFeedbackControlCommand unconstrainedJointCommand = doUnconstrainedJointControl();

      submitControllerCoreCommands(unconstrainedJointCommand);

      for (RobotSide robotSide : RobotSide.values)
      {
         controllerCoreOuput.getDesiredCenterOfPressure(footDesiredCoPs.get(robotSide), feet.get(robotSide).getRigidBody());
         momentumBasedController.setDesiredCenterOfPressure(feet.get(robotSide), footDesiredCoPs.get(robotSide));
      }

      momentumBasedController.doSecondaryControl();

      momentumBasedController.doPassiveKneeControl();
      momentumBasedController.doProportionalControlOnCoP(footDesiredCoPs);
   }

   public void submitControllerCoreCommands(JointspaceFeedbackControlCommand unconstrainedJointCommand)
   {
      planeContactStateCommandPool.clear();
      double wRhoSmoother = momentumBasedController.smoothDesiredCoPIfNeeded(footDesiredCoPs);

      controllerCoreCommand.clear();

      for (RobotSide robotSide : RobotSide.values)
      {
         InverseDynamicsCommand<?> footInverseDynamicsCommand = feetManager.getInverseDynamicsCommand(robotSide);
         if (footInverseDynamicsCommand != null)
            controllerCoreCommand.addInverseDynamicsCommand(footInverseDynamicsCommand);

         FeedbackControlCommand<?> footFeedbackControlCommand = feetManager.getFeedbackControlCommand(robotSide);
         if (footFeedbackControlCommand != null)
            controllerCoreCommand.addFeedbackControlCommand(footFeedbackControlCommand);

         InverseDynamicsCommand<?> handInverseDynamicsCommand = manipulationControlModule.getInverseDynamicsCommand(robotSide);
         if (handInverseDynamicsCommand != null)
            controllerCoreCommand.addInverseDynamicsCommand(handInverseDynamicsCommand);

         FeedbackControlCommand<?> handFeedbackControlCommand = manipulationControlModule.getFeedbackControlCommand(robotSide);
         if (handFeedbackControlCommand != null)
            controllerCoreCommand.addFeedbackControlCommand(handFeedbackControlCommand);

         YoPlaneContactState contactState = momentumBasedController.getContactState(feet.get(robotSide));
         PlaneContactStateCommand planeContactStateCommand = planeContactStateCommandPool.createCommand();
         contactState.getPlaneContactStateCommand(planeContactStateCommand);
         planeContactStateCommand.setWRhoSmoother(wRhoSmoother);
      }

      controllerCoreCommand.addInverseDynamicsCommand(planeContactStateCommandPool);
      controllerCoreCommand.addFeedbackControlCommand(headOrientationManager.getFeedbackControlCommand());
      controllerCoreCommand.addFeedbackControlCommand(chestOrientationManager.getFeedbackControlCommand());
      controllerCoreCommand.addFeedbackControlCommand(pelvisOrientationManager.getFeedbackControlCommand());

      controllerCoreCommand.addInverseDynamicsCommand(getUncontrolledJointCommand());
      controllerCoreCommand.addFeedbackControlCommand(unconstrainedJointCommand);

      controllerCoreCommand.addInverseDynamicsCommand(balanceManager.getInverseDynamicsCommand());
   }

   private void doCapturePointBasedControl()
   {
      boolean keepCMPInsideSupportPolygon = true;
      if ((manipulationControlModule != null) && (manipulationControlModule.isAtLeastOneHandLoadBearing()))
         keepCMPInsideSupportPolygon = false;

      balanceManager.compute(keepCMPInsideSupportPolygon);
   }

   private void integrateAnkleAccelerationsOnSwingLeg(RobotSide swingSide)
   {
      fullRobotModel.getLegJoint(swingSide, LegJointName.ANKLE_PITCH).setIntegrateDesiredAccelerations(true);
      fullRobotModel.getLegJoint(swingSide, LegJointName.ANKLE_ROLL).setIntegrateDesiredAccelerations(true);
      fullRobotModel.getLegJoint(swingSide.getOppositeSide(), LegJointName.ANKLE_PITCH).setIntegrateDesiredAccelerations(false);
      fullRobotModel.getLegJoint(swingSide.getOppositeSide(), LegJointName.ANKLE_ROLL).setIntegrateDesiredAccelerations(false);
   }

   private void doNotIntegrateAnkleAccelerations()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         fullRobotModel.getLegJoint(robotSide, LegJointName.ANKLE_PITCH).setIntegrateDesiredAccelerations(false);
         fullRobotModel.getLegJoint(robotSide, LegJointName.ANKLE_ROLL).setIntegrateDesiredAccelerations(false);
      }
   }

   private void resetLoadedLegIntegrators(RobotSide robotSide)
   {
      if (resetIntegratorsAfterSwing.getBooleanValue())
      {
         for (LegJointName jointName : fullRobotModel.getRobotSpecificJointNames().getLegJointNames())
            fullRobotModel.getLegJoint(robotSide, jointName).resetDesiredAccelerationIntegrator();
      }
   }

   public void reinitializePelvisOrientation(boolean reinitialize)
   {
      hasWalkingControllerBeenInitialized.set(!reinitialize);
   }

   private void consumeHeadMessages()
   {
      if (commandInputManager.isNewMessageAvailable(ModifiableHeadTrajectoryMessage.class))
         headOrientationManager.handleHeadTrajectoryMessage(commandInputManager.pollNewestMessage(ModifiableHeadTrajectoryMessage.class));
   }

   private void consumeChestMessages()
   {
      if (commandInputManager.isNewMessageAvailable(ModifiableChestTrajectoryMessage.class))
         chestOrientationManager.handleChestTrajectoryMessage(commandInputManager.pollNewestMessage(ModifiableChestTrajectoryMessage.class));
   }

   private void consumePelvisMessages()
   {
      if (commandInputManager.isNewMessageAvailable(ModifiablePelvisOrientationTrajectoryMessage.class))
         pelvisOrientationManager
               .handlePelvisOrientationTrajectoryMessages(commandInputManager.pollNewestMessage(ModifiablePelvisOrientationTrajectoryMessage.class));

      if (commandInputManager.isNewMessageAvailable(ModifiablePelvisTrajectoryMessage.class))
      {
         ModifiablePelvisTrajectoryMessage message = commandInputManager.pollNewestMessage(ModifiablePelvisTrajectoryMessage.class);
         State<WalkingState> currentState = stateMachine.getCurrentState();
         if (currentState instanceof DoubleSupportState)
            ((DoubleSupportState) currentState).handlePelvisTrajectoryMessage(message);
         else if (currentState instanceof SingleSupportState)
            ((SingleSupportState) currentState).handlePelvisTrajectoryMessage(message);
      }
   }

   private void consumePelvisHeightMessages()
   {
      if (commandInputManager.isNewMessageAvailable(ModifiablePelvisHeightTrajectoryMessage.class))
         comHeightManager.handlePelvisHeightTrajectoryMessage(commandInputManager.pollNewestMessage(ModifiablePelvisHeightTrajectoryMessage.class));
   }

   private void consumeManipulationMessages()
   {
      if (yoTime.getDoubleValue() - timeOfLastManipulationAbortRequest.getDoubleValue() < manipulationIgnoreInputsDurationAfterAbort.getDoubleValue())
      {
         commandInputManager.flushManipulationBuffers();
         return;
      }

      manipulationControlModule.handleHandTrajectoryMessages(commandInputManager.pollNewMessages(ModifiableHandTrajectoryMessage.class));
      manipulationControlModule.handleArmTrajectoryMessages(commandInputManager.pollNewMessages(ModifiableArmTrajectoryMessage.class));
      manipulationControlModule.handleArmDesiredAccelerationsMessages(commandInputManager.pollNewMessages(ModifiableArmDesiredAccelerationsMessage.class));
      manipulationControlModule
            .handleHandComplianceControlParametersMessages(commandInputManager.pollNewMessages(ModifiableHandComplianceControlParametersMessage.class));
   }

   private void consumeGoHomeMessages()
   {
      if (!commandInputManager.isNewMessageAvailable(ModifiableGoHomeMessage.class))
         return;

      ModifiableGoHomeMessage message = commandInputManager.pollAndCompileGoHomeMessages();
      manipulationControlModule.handleGoHomeMessage(message);

      pelvisOrientationManager.handleGoHomeMessage(message);
      balanceManager.handleGoHomeMessage(message);
      chestOrientationManager.handleGoHomeMessage(message);
   }

   private void consumeEndEffectorLoadBearingMessages()
   {
      if (!commandInputManager.isNewMessageAvailable(ModifiableEndEffectorLoadBearingMessage.class))
         return;

      ModifiableEndEffectorLoadBearingMessage message = commandInputManager.pollAndCompileEndEffectorLoadBearingMessages();
      manipulationControlModule.handleEndEffectorLoadBearingMessage(message);
      State<WalkingState> currentState = stateMachine.getCurrentState();
      if (currentState instanceof SingleSupportState)
         ((SingleSupportState) currentState).handleEndEffectorLoadBearingRequest(message);
   }

   private void consumeStopAllTrajectoryMessages()
   {
      if (!commandInputManager.isNewMessageAvailable(ModifiableStopAllTrajectoryMessage.class))
         return;

      ModifiableStopAllTrajectoryMessage message = commandInputManager.pollNewestMessage(ModifiableStopAllTrajectoryMessage.class);
      chestOrientationManager.handleStopAllTrajectoryMessage(message);
      feetManager.handleStopAllTrajectoryMessage(message);
      comHeightManager.handleStopAllTrajectoryMessage(message);
      balanceManager.handleStopAllTrajectoryMessage(message);
      pelvisOrientationManager.handleStopAllTrajectoryMessage(message);
   }

   private void consumeFootTrajectoryMessages()
   {
      if (commandInputManager.isNewMessageAvailable(ModifiableFootTrajectoryMessage.class))
         walkingMessageHandler.handleFootTrajectoryMessage(commandInputManager.pollNewestMessage(ModifiableFootTrajectoryMessage.class));

      if (commandInputManager.isNewMessageAvailable(ModifiablePauseWalkingMessage.class))
         walkingMessageHandler.handlePauseWalkingMessage(commandInputManager.pollNewestMessage(ModifiablePauseWalkingMessage.class));

      if (commandInputManager.isNewMessageAvailable(ModifiableFootstepDataListMessage.class))
         walkingMessageHandler.handleFootstepDataListMessage(commandInputManager.pollNewestMessage(ModifiableFootstepDataListMessage.class));

      updateICPPlannerTimesAndOmega0();
   }

   private void consumeAbortWalkingMessages()
   {
      if (!commandInputManager.isNewMessageAvailable(ModifiableAbortWalkingMessage.class))
         return;
      abortWalkingRequested.set(commandInputManager.pollNewestMessage(ModifiableAbortWalkingMessage.class).isAbortWalkingRequested());
   }

   @Override
   public ControllerCoreCommand getControllerCoreCommand()
   {
      return controllerCoreCommand;
   }
}
