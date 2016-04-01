package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.List;

import us.ihmc.SdfLoader.partNames.ArmJointName;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.SdfLoader.partNames.LimbName;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.JointspaceFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.desiredFootStep.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingManagers;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.stateTransitionConditions.DoneWithStateCondition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.stateTransitionConditions.DoubleSupportToSingleSupportConditionForDisturbanceRecovery;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.stateTransitionConditions.SingleSupportToTransferToCondition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.stateTransitionConditions.StartFlamingoCondition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.stateTransitionConditions.StopFlamingoCondition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.stateTransitionConditions.StopWalkingFromSingleSupportCondition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.stateTransitionConditions.StopWalkingFromTransferCondition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.BalanceManager;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.AbortWalkingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ArmDesiredAccelerationsCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ArmTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.AutomaticManipulationAbortCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ChestTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EndEffectorLoadBearingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.GoHomeCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandComplianceControlParametersCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HeadTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.NeckDesiredAccelerationsCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.NeckTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PauseWalkingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisHeightTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisOrientationTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage.EndEffector;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage.LoadBearingRequest;
import us.ihmc.humanoidRobotics.communication.packets.walking.ManipulationAbortedStatus;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.stateMachines.GenericStateMachine;
import us.ihmc.robotics.stateMachines.State;
import us.ihmc.robotics.stateMachines.StateChangedListener;
import us.ihmc.robotics.stateMachines.StateTransition;
import us.ihmc.robotics.stateMachines.StateTransitionCondition;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;

public class WalkingHighLevelHumanoidController extends AbstractHighLevelHumanoidControlPattern
{
   private final static HighLevelState controllerState = HighLevelState.WALKING;

   private final GenericStateMachine<WalkingStateEnum, WalkingState> stateMachine;

   private final BalanceManager balanceManager;
   private final CenterOfMassHeightManager comHeightManager;

   private final SideDependentList<FootSwitchInterface> footSwitches;

   private final DoubleYoVariable maxICPErrorBeforeSingleSupportX = new DoubleYoVariable("maxICPErrorBeforeSingleSupportX", registry);
   private final DoubleYoVariable maxICPErrorBeforeSingleSupportY = new DoubleYoVariable("maxICPErrorBeforeSingleSupportY", registry);

   private final WalkingMessageHandler walkingMessageHandler;
   private final BooleanYoVariable abortWalkingRequested = new BooleanYoVariable("requestAbortWalking", registry);

   private final BipedSupportPolygons bipedSupportPolygons;

   private final DoubleYoVariable controlledCoMHeightAcceleration = new DoubleYoVariable("controlledCoMHeightAcceleration", registry);

   private final BooleanYoVariable preparingForLocomotion = new BooleanYoVariable("preparingForLocomotion", registry);
   private final DoubleYoVariable timeToGetPreparedForLocomotion = new DoubleYoVariable("timeToGetPreparedForLocomotion", registry);
   private final DoubleYoVariable preparingForLocomotionStartTime = new DoubleYoVariable("preparingForLocomotionStartTime", registry);
   private final BooleanYoVariable doPrepareManipulationForLocomotion = new BooleanYoVariable("doPrepareManipulationForLocomotion", registry);
   private final BooleanYoVariable doPreparePelvisForLocomotion = new BooleanYoVariable("doPreparePelvisForLocomotion", registry);

   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final BooleanYoVariable hasWalkingControllerBeenInitialized = new BooleanYoVariable("hasWalkingControllerBeenInitialized", registry);

   private final DoubleYoVariable remainingSwingTimeAccordingToPlan = new DoubleYoVariable("remainingSwingTimeAccordingToPlan", registry);
   private final DoubleYoVariable estimatedRemainingSwingTimeUnderDisturbance = new DoubleYoVariable("estimatedRemainingSwingTimeUnderDisturbance", registry);
   private final DoubleYoVariable icpErrorThresholdToSpeedUpSwing = new DoubleYoVariable("icpErrorThresholdToSpeedUpSwing", registry);

   private final BooleanYoVariable hasICPPlannerBeenInitializedAtStart = new BooleanYoVariable("hasICPPlannerBeenInitializedAtStart", registry);

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

   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusOutputManager;
   private final PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);
   private ControllerCoreOutputReadOnly controllerCoreOutput;

   public WalkingHighLevelHumanoidController(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
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

      bipedSupportPolygons = momentumBasedController.getBipedSupportPolygons();

      footSwitches = momentumBasedController.getFootSwitches();

      double defaultTransferTime = walkingControllerParameters.getDefaultTransferTime();
      double defaultSwingTime = walkingControllerParameters.getDefaultSwingTime();
      walkingMessageHandler = new WalkingMessageHandler(defaultTransferTime, defaultSwingTime, feet, statusOutputManager, yoGraphicsListRegistry, registry);

      String namePrefix = "walking";

      stateMachine = new GenericStateMachine<>(namePrefix + "State", namePrefix + "SwitchTime", WalkingStateEnum.class, yoTime, registry); // this is used by name, and it is ugly.

      setupStateMachine();

      maxICPErrorBeforeSingleSupportX.set(walkingControllerParameters.getMaxICPErrorBeforeSingleSupportX());
      maxICPErrorBeforeSingleSupportY.set(walkingControllerParameters.getMaxICPErrorBeforeSingleSupportY());

      distanceToShrinkSupportPolygonWhenHoldingCurrent.set(0.08);

      double highCoPDampingDuration = walkingControllerParameters.getHighCoPDampingDurationToPreventFootShakies();
      double coPErrorThreshold = walkingControllerParameters.getCoPErrorThresholdForHighCoPDamping();
      boolean enableHighCoPDamping = highCoPDampingDuration > 0.0 && !Double.isInfinite(coPErrorThreshold);
      momentumBasedController.setHighCoPDampingParameters(enableHighCoPDamping, highCoPDampingDuration, coPErrorThreshold);
   }

   private void setupStateMachine()
   {
      StandingState standingState = new StandingState(registry);
      TransferToStandingState toStandingState = new TransferToStandingState(registry);
      WalkingStateEnum toStandingStateEnum = toStandingState.getStateEnum();

      stateMachine.addState(toStandingState);
      stateMachine.addState(standingState);

      SideDependentList<TransferToWalkingSingleSupportState> walkingTransferStates = new SideDependentList<>();

      for (RobotSide transferToSide : RobotSide.values)
      {
         TransferToWalkingSingleSupportState transferState = new TransferToWalkingSingleSupportState(transferToSide, registry);
         walkingTransferStates.put(transferToSide, transferState);
         stateMachine.addState(transferState);
      }

      SideDependentList<WalkingSingleSupportState> walkingSingleSupportStates = new SideDependentList<>();

      for (RobotSide supportSide : RobotSide.values)
      {
         WalkingSingleSupportState singleSupportState = new WalkingSingleSupportState(supportSide, registry);
         walkingSingleSupportStates.put(supportSide, singleSupportState);
         stateMachine.addState(singleSupportState);
      }

      SideDependentList<TransferToFlamingoStanceState> flamingoTransferStates = new SideDependentList<>();

      for (RobotSide transferToSide : RobotSide.values)
      {
         TransferToFlamingoStanceState transferState = new TransferToFlamingoStanceState(transferToSide, registry);
         flamingoTransferStates.put(transferToSide, transferState);
         stateMachine.addState(transferState);
      }

      SideDependentList<FlamingoStanceState> flamingoSingleSupportStates = new SideDependentList<>();

      for (RobotSide supportSide : RobotSide.values)
      {
         FlamingoStanceState singleSupportState = new FlamingoStanceState(supportSide, registry);
         flamingoSingleSupportStates.put(supportSide, singleSupportState);
         stateMachine.addState(singleSupportState);
      }

      // Encapsulate toStandingTransition to make sure it is not used afterwards
      {
         DoneWithStateCondition toStandingDoneCondition = new DoneWithStateCondition(toStandingState);
         StateTransition<WalkingStateEnum> toStandingTransition = new StateTransition<>(WalkingStateEnum.STANDING, toStandingDoneCondition);
         toStandingState.addStateTransition(toStandingTransition);
      }

      // Setup start/stop walking conditions
      for (RobotSide robotSide : RobotSide.values)
      {
         TransferToWalkingSingleSupportState transferState = walkingTransferStates.get(robotSide);
         SingleSupportState singleSupportState = walkingSingleSupportStates.get(robotSide);

         WalkingStateEnum transferStateEnum = transferState.getStateEnum();

         // Start walking
         StartWalkingCondition startWalkingCondition = new StartWalkingCondition(transferState.getTransferToSide());
         StateTransition<WalkingStateEnum> toTransfer = new StateTransition<WalkingStateEnum>(transferStateEnum, startWalkingCondition);
         standingState.addStateTransition(toTransfer);
         toStandingState.addStateTransition(toTransfer);

         // Stop walking when in transfer
         StopWalkingFromTransferCondition stopWalkingFromTranferCondition = new StopWalkingFromTransferCondition(transferState, walkingMessageHandler);
         StateTransition<WalkingStateEnum> fromTransferToStanding = new StateTransition<WalkingStateEnum>(toStandingStateEnum, stopWalkingFromTranferCondition);
         transferState.addStateTransition(fromTransferToStanding);

         // Stop walking when in single support
         StopWalkingFromSingleSupportCondition stopWalkingFromSingleSupportCondition = new StopWalkingFromSingleSupportCondition(singleSupportState, walkingMessageHandler);
         StateTransition<WalkingStateEnum> fromSingleSupportToStanding = new StateTransition<WalkingStateEnum>(toStandingStateEnum, stopWalkingFromSingleSupportCondition);
         singleSupportState.addStateTransition(fromSingleSupportToStanding);
      }

      // Setup walking transfer to single support conditions
      for (RobotSide robotSide : RobotSide.values)
      {
         TransferToWalkingSingleSupportState transferState = walkingTransferStates.get(robotSide);
         SingleSupportState singleSupportState = walkingSingleSupportStates.get(robotSide);
         WalkingStateEnum singleSupportStateEnum = singleSupportState.getStateEnum();

         DoneWithStateCondition doneWithTransferCondition = new DoneWithStateCondition(transferState);
         StateTransition<WalkingStateEnum> toSingleSupport = new StateTransition<WalkingStateEnum>(singleSupportStateEnum, doneWithTransferCondition);
         transferState.addStateTransition(toSingleSupport);
      }

      // Setup walking single support to transfer conditions
      for (RobotSide robotSide : RobotSide.values)
      {
         SingleSupportState singleSupportState = walkingSingleSupportStates.get(robotSide);

         // Single support to transfer with same side
         {
            TransferToWalkingSingleSupportState transferState = walkingTransferStates.get(robotSide);
            WalkingStateEnum transferStateEnum = transferState.getStateEnum();

            SingleSupportToTransferToCondition singleSupportToTransferToCondition = new SingleSupportToTransferToCondition(singleSupportState, transferState, walkingMessageHandler);
            StateTransition<WalkingStateEnum> toTransfer = new StateTransition<>(transferStateEnum, singleSupportToTransferToCondition);
            singleSupportState.addStateTransition(toTransfer);
         }

         // Single support to transfer with opposite side
         {
            TransferToWalkingSingleSupportState transferState = walkingTransferStates.get(robotSide.getOppositeSide());
            WalkingStateEnum transferStateEnum = transferState.getStateEnum();

            SingleSupportToTransferToCondition singleSupportToTransferToCondition = new SingleSupportToTransferToCondition(singleSupportState, transferState, walkingMessageHandler);
            StateTransition<WalkingStateEnum> toTransfer = new StateTransition<>(transferStateEnum, singleSupportToTransferToCondition);
            singleSupportState.addStateTransition(toTransfer);
         }
      }

      // Setup start/stop flamingo conditions
      for (RobotSide robotSide : RobotSide.values)
      {
         TransferToFlamingoStanceState transferState = flamingoTransferStates.get(robotSide);
         FlamingoStanceState singleSupportState = flamingoSingleSupportStates.get(robotSide);

         WalkingStateEnum transferStateEnum = transferState.getStateEnum();

         // Start flamingo
         StartFlamingoCondition startFlamingoCondition = new StartFlamingoCondition(transferState.getTransferToSide(), walkingMessageHandler);
         StateTransition<WalkingStateEnum> toTransfer = new StateTransition<>(transferStateEnum, startFlamingoCondition);
         standingState.addStateTransition(toTransfer);
         toStandingState.addStateTransition(toTransfer);

         // Stop flamingo
         StopFlamingoCondition stopFlamingoCondition = new StopFlamingoCondition(singleSupportState, walkingMessageHandler);
         StateTransition<WalkingStateEnum> toStanding = new StateTransition<>(toStandingStateEnum, stopFlamingoCondition);
         singleSupportState.addStateTransition(toStanding);
      }

      // Setup tranfer to flamingo stance condition
      for (RobotSide robotSide : RobotSide.values)
      {
         TransferToFlamingoStanceState transferState = flamingoTransferStates.get(robotSide);
         FlamingoStanceState singleSupportState = flamingoSingleSupportStates.get(robotSide);

         WalkingStateEnum singleSupportStateEnum = singleSupportState.getStateEnum();

         DoneWithStateCondition doneWithTransferCondition = new DoneWithStateCondition(transferState);
         StateTransition<WalkingStateEnum> toSingleSupport = new StateTransition<WalkingStateEnum>(singleSupportStateEnum, doneWithTransferCondition);
         transferState.addStateTransition(toSingleSupport);
      }

      // Setup the abort condition from all states to the toStandingState
      for (RobotSide robotSide : RobotSide.values)
      {
         AbortCondition abortCondition = new AbortCondition();
         StateTransition<WalkingStateEnum> abortTransition = new StateTransition<>(toStandingStateEnum, abortCondition);

         walkingSingleSupportStates.get(robotSide).addStateTransition(abortTransition);
         walkingTransferStates.get(robotSide).addStateTransition(abortTransition);
         flamingoSingleSupportStates.get(robotSide).addStateTransition(abortTransition);
         flamingoTransferStates.get(robotSide).addStateTransition(abortTransition);
      }

      // Setup transition condition for push recovery, when recovering from double support.
      SideDependentList<StateTransition<WalkingStateEnum>> fromFallingToSingleSupportsTransitions = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         WalkingSingleSupportState walkingSingleSupportState = walkingSingleSupportStates.get(robotSide);
         WalkingStateEnum walkingSingleSupportStateEnum = walkingSingleSupportState.getStateEnum();
         RobotSide supportSide = walkingSingleSupportState.getSupportSide();

         DoubleSupportToSingleSupportConditionForDisturbanceRecovery isFallingFromDoubleSupportCondition = new DoubleSupportToSingleSupportConditionForDisturbanceRecovery(supportSide, balanceManager);
         StateTransition<WalkingStateEnum> fromFallingToSingleSupport = new StateTransition<WalkingStateEnum>(walkingSingleSupportStateEnum, isFallingFromDoubleSupportCondition);
         fromFallingToSingleSupportsTransitions.put(robotSide, fromFallingToSingleSupport);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         toStandingState.addStateTransition(fromFallingToSingleSupportsTransitions.get(robotSide));
         standingState.addStateTransition(fromFallingToSingleSupportsTransitions.get(robotSide));

         for (RobotSide transferSide : RobotSide.values)
         {
            walkingTransferStates.get(transferSide).addStateTransition(fromFallingToSingleSupportsTransitions.get(robotSide));
         }
      }

      stateMachine.attachStateChangedListener(new StateChangedListener<WalkingStateEnum>()
      {
         @Override
         public void stateChanged(State<WalkingStateEnum> oldState, State<WalkingStateEnum> newState, double time)
         {
            momentumBasedController.reportControllerStateChangeToListeners(oldState.getStateEnum(), newState.getStateEnum());
         }
      });
   }

   @Override
   public void setControllerCoreOutput(ControllerCoreOutputReadOnly controllerCoreOutput)
   {
      this.controllerCoreOutput = controllerCoreOutput;
   }

   @Override
   public void initialize()
   {
      super.initialize();

      commandInputManager.flushAllCommands();

      privilegedConfigurationCommand.setPrivilegedConfigurationOption(PrivilegedConfigurationOption.AT_ZERO);

      for (RobotSide robotSide : RobotSide.values)
      {
         ArmJointName[] armJointNames = fullRobotModel.getRobotSpecificJointNames().getArmJointNames();
         for (int i = 0; i < armJointNames.length; i++)
            privilegedConfigurationCommand.addJoint(fullRobotModel.getArmJoint(robotSide, armJointNames[i]), PrivilegedConfigurationOption.AT_MID_RANGE);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         privilegedConfigurationCommand.addJoint(fullRobotModel.getLegJoint(robotSide, LegJointName.KNEE), 1.0);

         RigidBody pelvis = fullRobotModel.getPelvis();
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         privilegedConfigurationCommand.applyPrivilegedConfigurationToSubChain(pelvis, foot);
      }

      initializeContacts();

      for (RobotSide robotSide : RobotSide.values)
      {
         controllerCoreOutput.getDesiredCenterOfPressure(footDesiredCoPs.get(robotSide), feet.get(robotSide).getRigidBody());
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
      stateMachine.setCurrentState(WalkingStateEnum.TO_STANDING);

      hasWalkingControllerBeenInitialized.set(true);
   }

   public void initializeDesiredHeightToCurrent()
   {
      comHeightManager.initializeDesiredHeightToCurrent();
      feetManager.resetHeightCorrectionParametersForSingularityAvoidance();
   }

   private void initializeContacts()
   {
      momentumBasedController.clearContacts();

      for (RobotSide robotSide : RobotSide.values)
      {
         feetManager.setFlatFootContactState(robotSide);
      }
   }

   public class StandingState extends WalkingState
   {

      public StandingState(YoVariableRegistry parentRegistry)
      {
         super(WalkingStateEnum.STANDING, parentRegistry);
      }

      @Override
      public void doAction()
      {
         comHeightManager.setSupportLeg(RobotSide.LEFT);
      }

      @Override
      public void doTransitionIntoAction()
      {
         commandInputManager.flushAllCommands();

         balanceManager.clearICPPlan();
         balanceManager.setDoubleSupportTime(walkingMessageHandler.getTransferTime());
         balanceManager.setSingleSupportTime(walkingMessageHandler.getSwingTime());
         balanceManager.resetPushRecovery();
         balanceManager.enablePelvisXYControl();

         momentumBasedController.updateBipedSupportPolygons(); // need to always update biped support polygons after a change to the contact states


         walkingMessageHandler.reportWalkingComplete();

         failureDetectionControlModule.setNextFootstep(null);
         momentumBasedController.reportChangeOfRobotMotionStatus(RobotMotionStatus.STANDING);


         pelvisOrientationManager.setToHoldCurrentDesiredInMidFeetZUpFrame();
      }

      @Override
      public void doTransitionOutOfAction()
      {
         balanceManager.disablePelvisXYControl();
         feetManager.reset();

         momentumBasedController.reportChangeOfRobotMotionStatus(RobotMotionStatus.IN_MOTION);
      }

      @Override
      public boolean isStateSafeToConsumePelvisTrajectoryCommand()
      {
         return true;
      }

      @Override
      public boolean isStateSafeToConsumeManipulationCommands()
      {
         return true;
      }
   }

   public class TransferToStandingState extends WalkingState
   {
      private final DoubleYoVariable maxICPErrorToSwitchToStanding = new DoubleYoVariable("maxICPErrorToSwitchToStanding", registry);

      public TransferToStandingState(YoVariableRegistry parentRegistry)
      {
         super(WalkingStateEnum.TO_STANDING, parentRegistry);
         maxICPErrorToSwitchToStanding.set(0.025);
      }

      @Override
      public void doAction()
      {
         // Always do this so that when a foot slips or is loaded in the air, the height gets adjusted.
         comHeightManager.setSupportLeg(RobotSide.LEFT);
      }

      @Override
      public boolean isDone()
      {
         if (!balanceManager.isICPPlanDone())
            return false;

         return balanceManager.getICPErrorMagnitude() < maxICPErrorToSwitchToStanding.getDoubleValue();
      }

      @Override
      public void doTransitionIntoAction()
      {
         balanceManager.clearICPPlan();
         balanceManager.resetPushRecovery();

         feetManager.initializeContactStatesForDoubleSupport(null);
         momentumBasedController.updateBipedSupportPolygons(); // need to always update biped support polygons after a change to the contact states

         failureDetectionControlModule.setNextFootstep(null);

         TransferToAndNextFootstepsData transferToAndNextFootstepsDataForDoubleSupport = createTransferToAndNextFootstepDataForDoubleSupport(RobotSide.LEFT);
         double extraToeOffHeight = 0.0;
         comHeightManager.initialize(transferToAndNextFootstepsDataForDoubleSupport, extraToeOffHeight);


         // Just standing in double support, do nothing
         pelvisOrientationManager.setToHoldCurrentDesiredInMidFeetZUpFrame();

         if (holdICPToCurrentCoMLocationInNextDoubleSupport.getBooleanValue())
         {
            balanceManager.requestICPPlannerToHoldCurrentCoM(distanceToShrinkSupportPolygonWhenHoldingCurrent.getDoubleValue());
            holdICPToCurrentCoMLocationInNextDoubleSupport.set(false);
         }

         balanceManager.initializeICPPlanForStanding();
      }

      @Override
      public void doTransitionOutOfAction()
      {
      }
   }

   public abstract class TransferState extends WalkingState
   {
      private final RobotSide transferToSide;
      private final FramePoint2d desiredICPLocal = new FramePoint2d();
      private final FramePoint2d capturePoint2d = new FramePoint2d();
      private final FramePoint2d desiredCMP = new FramePoint2d();

      public TransferState(RobotSide transferToSide, WalkingStateEnum transferStateEnum, YoVariableRegistry parentRegistry)
      {
         super(transferStateEnum, parentRegistry);
         this.transferToSide = transferToSide;
      }

      public RobotSide getTransferToSide()
      {
         return transferToSide;
      }

      @Override
      public void doAction()
      {
         feetManager.updateContactStatesInDoubleSupport(transferToSide);

         switchToToeOffIfPossible();

         // Always do this so that when a foot slips or is loaded in the air, the height gets adjusted.
         comHeightManager.setSupportLeg(transferToSide);
      }

      private final FrameVector2d icpError2d = new FrameVector2d();

      @Override
      public boolean isDone()
      {
         if (!balanceManager.isICPPlanDone())
            return false;

         balanceManager.getICPError(icpError2d);
         icpError2d.changeFrame(referenceFrames.getAnkleZUpFrame(transferToSide));

         double ellipticErrorSquared = MathTools.square(icpError2d.getX() / maxICPErrorBeforeSingleSupportX.getDoubleValue())
               + MathTools.square(icpError2d.getY() / maxICPErrorBeforeSingleSupportY.getDoubleValue());
         boolean closeEnough = ellipticErrorSquared < 1.0;

         return closeEnough;
      }

      public boolean isStopWalkingSafe()
      {
         balanceManager.getCapturePoint(capturePoint2d);
         FrameConvexPolygon2d supportPolygonInWorld = bipedSupportPolygons.getSupportPolygonInWorld();
         if (supportPolygonInWorld.getDistanceInside(capturePoint2d) < distanceToShrinkSupportPolygonWhenHoldingCurrent.getDoubleValue())
            return false;
         return true;
      }

      public void switchToToeOffIfPossible()
      {
         RobotSide trailingLeg = transferToSide.getOppositeSide();
         // the only case left for determining the contact state of the trailing foot
         if (feetManager.getCurrentConstraintType(trailingLeg) != ConstraintType.TOES)
         {

            double predictedToeOffDuration = balanceManager.getTimeRemainingInCurrentState();

            balanceManager.getDesiredCMP(desiredCMP);
            balanceManager.getDesiredICP(desiredICPLocal);
            balanceManager.getCapturePoint(capturePoint2d);

            boolean doToeOff = feetManager.checkIfToeOffSafe(trailingLeg, desiredCMP, desiredICPLocal, capturePoint2d);

            if (doToeOff)
            {
               feetManager.requestToeOff(trailingLeg, predictedToeOffDuration);
               momentumBasedController.updateBipedSupportPolygons(); // need to always update biped support polygons after a change to the contact states
            }
         }
      }

      @Override
      public void doTransitionIntoAction()
      {
         balanceManager.clearICPPlan();

         feetManager.initializeContactStatesForDoubleSupport(transferToSide);
         momentumBasedController.updateBipedSupportPolygons(); // need to always update biped support polygons after a change to the contact states

         failureDetectionControlModule.setNextFootstep(walkingMessageHandler.peek(0));


         boolean hasFootTrajectoryMessage = walkingMessageHandler.hasFootTrajectoryForFlamingoStance();
         if (!hasFootTrajectoryMessage && !comHeightManager.hasBeenInitializedWithNextStep())
         {
            TransferToAndNextFootstepsData transferToAndNextFootstepsDataForDoubleSupport = createTransferToAndNextFootstepDataForDoubleSupport(transferToSide);
            double extraToeOffHeight = 0.0;
            if (feetManager.willDoToeOff(null, transferToSide))
               extraToeOffHeight = feetManager.getWalkOnTheEdgesManager().getExtraCoMMaxHeightWithToes();
            comHeightManager.initialize(transferToAndNextFootstepsDataForDoubleSupport, extraToeOffHeight);
         }

         balanceManager.resetPushRecovery();

         double transferTime;

         boolean isPreviousStateDoubleSupport = getPreviousState().getStateEnum() == WalkingStateEnum.STANDING;
         if (isPreviousStateDoubleSupport)
         {
            transferTime = balanceManager.getInitialTransferDuration();
         }
         else
         {
            transferTime = walkingMessageHandler.getTransferTime();
         }

         pelvisOrientationManager.setTrajectoryTime(transferTime);

         // Transferring to execute a foot pose, hold current desired in upcoming support foot in case it slips
         if (hasFootTrajectoryMessage)
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
            balanceManager.initializeICPPlanForTransfer(transferToSide);
         }
         else if (hasFootTrajectoryMessage)
         {
            balanceManager.setSingleSupportTime(Double.POSITIVE_INFINITY);
            balanceManager.addFootstepToPlan(walkingMessageHandler.getFootstepAtCurrentLocation(transferToSide.getOppositeSide()));
            balanceManager.initializeICPPlanForTransfer(transferToSide);
         }
      }

      @Override
      public void doTransitionOutOfAction()
      {
         balanceManager.disablePelvisXYControl();
         feetManager.reset();
      }
   }

   public class TransferToWalkingSingleSupportState extends TransferState
   {
      public TransferToWalkingSingleSupportState(RobotSide transferToSide, YoVariableRegistry parentRegistry)
      {
         super(transferToSide, WalkingStateEnum.getWalkingTransferState(transferToSide), parentRegistry);
      }
   }

   public class TransferToFlamingoStanceState extends TransferState
   {
      public TransferToFlamingoStanceState(RobotSide transferToSide, YoVariableRegistry parentRegistry)
      {
         super(transferToSide, WalkingStateEnum.getFlamingoTransferState(transferToSide), parentRegistry);
      }
   }

   private Footstep previousDesiredFootstep;

   public abstract class SingleSupportState extends WalkingState
   {
      private final BooleanYoVariable resetIntegratorsAfterSwing = new BooleanYoVariable("resetIntegratorsAfterSwing", registry);
      private final BooleanYoVariable hasMinimumTimePassed = new BooleanYoVariable("hasMinimumTimePassed", registry);
      private final DoubleYoVariable minimumSwingFraction = new DoubleYoVariable("minimumSwingFraction", registry);

      protected final RobotSide swingSide;
      protected final RobotSide supportSide;

      public SingleSupportState(RobotSide supportSide, WalkingStateEnum singleSupportStateEnum, YoVariableRegistry parentRegistry)
      {
         super(singleSupportStateEnum, parentRegistry);
         this.supportSide = supportSide;
         swingSide = supportSide.getOppositeSide();
         resetIntegratorsAfterSwing.set(true);

         minimumSwingFraction.set(0.5);
      }

      public RobotSide getSwingSide()
      {
         return swingSide;
      }

      public RobotSide getSupportSide()
      {
         return supportSide;
      }

      @Override
      public void doAction()
      {
      }

      @Override
      public boolean isDone()
      {
         hasMinimumTimePassed.set(hasMinimumTimePassed());

         if (!hasMinimumTimePassed.getBooleanValue())
            return false;

         return hasMinimumTimePassed.getBooleanValue() && footSwitches.get(swingSide).hasFootHitGround();
      }

      private boolean hasMinimumTimePassed()
      {
         double minimumSwingTime;
         if (balanceManager.isRecoveringFromDoubleSupportFall())
            minimumSwingTime = 0.15;
         else
            minimumSwingTime = walkingMessageHandler.getSwingTime() * minimumSwingFraction.getDoubleValue();

         return stateMachine.timeInCurrentState() > minimumSwingTime;
      }

      @Override
      public void doTransitionIntoAction()
      {
         balanceManager.clearICPPlan();
         footSwitches.get(swingSide).reset();
         integrateAnkleAccelerationsOnSwingLeg(swingSide);
      }

      @Override
      public void doTransitionOutOfAction()
      {
         balanceManager.resetPushRecovery();
         resetLoadedLegIntegrators(swingSide);
      }

      private void resetLoadedLegIntegrators(RobotSide robotSide)
      {
         if (resetIntegratorsAfterSwing.getBooleanValue())
         {
            for (LegJointName jointName : fullRobotModel.getRobotSpecificJointNames().getLegJointNames())
               fullRobotModel.getLegJoint(robotSide, jointName).resetDesiredAccelerationIntegrator();
         }
      }
   }

   public class WalkingSingleSupportState extends SingleSupportState
   {
      private Footstep nextFootstep;
      private final FramePose actualFootPoseInWorld = new FramePose(worldFrame);
      private final FramePoint nextExitCMP = new FramePoint();
      private final FramePoint2d toeOffContactPoint = new FramePoint2d();

      public WalkingSingleSupportState(RobotSide supportSide, YoVariableRegistry parentRegistry)
      {
         super(supportSide, WalkingStateEnum.getWalkingSingleSupportState(supportSide), parentRegistry);
      }

      @Override
      public void doAction()
      {
         super.doAction();

         boolean icpErrorIsTooLarge = balanceManager.getICPErrorMagnitude() > icpErrorThresholdToSpeedUpSwing.getDoubleValue();

         if (balanceManager.isPushRecoveryEnabled())
         {
            double estimatedTimeRemaining = balanceManager.getTimeRemainingInCurrentState();
            boolean footstepHasBeenAdjusted = balanceManager.checkAndUpdateFootstep(nextFootstep);
            if (footstepHasBeenAdjusted)
            {
               failureDetectionControlModule.setNextFootstep(nextFootstep);
               updateFootstepParameters();

               feetManager.replanSwingTrajectory(swingSide, nextFootstep, estimatedTimeRemaining);

               walkingMessageHandler.reportWalkingAbortRequested();
               walkingMessageHandler.clearFootsteps();

               balanceManager.clearICPPlan();
               balanceManager.addFootstepToPlan(nextFootstep);
               balanceManager.updateICPPlanForSingleSupportDisturbances();
               holdICPToCurrentCoMLocationInNextDoubleSupport.set(true);
            }
         }

         if (icpErrorIsTooLarge || balanceManager.isRecovering())
         {
            requestSwingSpeedUpIfNeeded();
         }

         walkingMessageHandler.clearFootTrajectory();

         switchToToeOffIfPossible(supportSide);
      }

      @Override
      public boolean isDone()
      {
         if (super.isDone())
            return true;

         return walkingControllerParameters.finishSingleSupportWhenICPPlannerIsDone() && balanceManager.isICPPlanDone();
      }

      @Override
      public void doTransitionIntoAction()
      {
         super.doTransitionIntoAction();

         if (balanceManager.isRecoveringFromDoubleSupportFall())
         {
            nextFootstep = balanceManager.createFootstepForRecoveringFromDisturbance(swingSide, walkingMessageHandler.getSwingTime());
            nextFootstep.setTrajectoryType(TrajectoryType.PUSH_RECOVERY);
            walkingMessageHandler.reportWalkingAbortRequested();
            walkingMessageHandler.clearFootsteps();
         }
         else
         {
            nextFootstep = walkingMessageHandler.poll();
         }

         updateFootstepParameters();

         balanceManager.addFootstepToPlan(nextFootstep);
         balanceManager.addFootstepToPlan(walkingMessageHandler.peek(0));
         balanceManager.addFootstepToPlan(walkingMessageHandler.peek(1));
         balanceManager.initializeICPPlanForSingleSupport(supportSide);

         if (balanceManager.isRecoveringFromDoubleSupportFall())
         {
            balanceManager.updateICPPlanForSingleSupportDisturbances();
            holdICPToCurrentCoMLocationInNextDoubleSupport.set(true);
         }

         feetManager.requestSwing(swingSide, nextFootstep, walkingMessageHandler.getSwingTime());
         walkingMessageHandler.reportFootstepStarted(swingSide);
      }

      @Override
      public void doTransitionOutOfAction()
      {
         super.doTransitionOutOfAction();

         actualFootPoseInWorld.setToZero(fullRobotModel.getEndEffectorFrame(swingSide, LimbName.LEG)); // changed Here Nicolas
         actualFootPoseInWorld.changeFrame(worldFrame);
         walkingMessageHandler.reportFootstepCompleted(swingSide, actualFootPoseInWorld);

         previousDesiredFootstep = nextFootstep;
      }

      public void switchToToeOffIfPossible(RobotSide supportSide)
      {
         if (feetManager.doToeOffIfPossibleInSingleSupport())
         {
            boolean willDoToeOff = feetManager.willDoToeOff(nextFootstep, swingSide);

            if (feetManager.isInFlatSupportState(supportSide) && willDoToeOff && balanceManager.isOnExitCMP())
            {
               balanceManager.getNextExitCMP(nextExitCMP);
               nextExitCMP.changeFrame(feet.get(supportSide).getSoleFrame());
               toeOffContactPoint.setByProjectionOntoXYPlaneIncludingFrame(nextExitCMP);
               feetManager.registerDesiredContactPointForToeOff(supportSide, toeOffContactPoint);
               double predictedToeOffDuration = balanceManager.getTimeRemainingInCurrentState() + walkingMessageHandler.getTransferTime();
               feetManager.requestToeOff(supportSide, predictedToeOffDuration);
            }
         }
      }

      private void requestSwingSpeedUpIfNeeded()
      {
         remainingSwingTimeAccordingToPlan.set(balanceManager.getTimeRemainingInCurrentState());
         estimatedRemainingSwingTimeUnderDisturbance.set(balanceManager.estimateTimeRemainingForSwingUnderDisturbance());

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
         momentumBasedController.updateBipedSupportPolygons();

      }
   }

   public class FlamingoStanceState extends SingleSupportState
   {
      private final FramePoint2d capturePoint2d = new FramePoint2d();
      private final BooleanYoVariable loadFoot = new BooleanYoVariable("loadFoot", registry);
      private final DoubleYoVariable loadFootStartTime = new DoubleYoVariable("loadFootStartTime", registry);
      private final DoubleYoVariable loadFootDuration = new DoubleYoVariable("loadFootDuration", registry);
      private final DoubleYoVariable loadFootTransferDuration = new DoubleYoVariable("loadFootTransferDuration", registry);

      public FlamingoStanceState(RobotSide supportSide, YoVariableRegistry parentRegistry)
      {
         super(supportSide, WalkingStateEnum.getFlamingoSingleSupportState(supportSide), parentRegistry);

         loadFoot.set(false);
         loadFootDuration.set(1.2);
         loadFootTransferDuration.set(0.8);
      }

      @Override
      public void doAction()
      {
         super.doAction();

         if (walkingMessageHandler.hasFootTrajectoryForFlamingoStance(swingSide))
         {
            while(walkingMessageHandler.hasFootTrajectoryForFlamingoStance(swingSide))
               feetManager.handleFootTrajectoryCommand(walkingMessageHandler.pollFootTrajectoryForFlamingoStance(swingSide));
         }

         balanceManager.getCapturePoint(capturePoint2d);

         boolean icpErrorIsTooLarge = balanceManager.getICPErrorMagnitude() > 0.05;

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

         walkingMessageHandler.clearFootTrajectory(supportSide);
      }

      @Override
      public boolean isDone()
      {
         if (super.isDone())
            return true;

         if (loadFoot.getBooleanValue() && yoTime.getDoubleValue() > loadFootStartTime.getDoubleValue() + loadFootDuration.getDoubleValue())
         {
            loadFoot.set(false);
            return true;
         }

         return false;
      }

      @Override
      public void doTransitionIntoAction()
      {
         super.doTransitionIntoAction();

         balanceManager.enablePelvisXYControl();
         feetManager.handleFootTrajectoryCommand(walkingMessageHandler.pollFootTrajectoryForFlamingoStance(swingSide));

         pelvisOrientationManager.setToHoldCurrentDesiredInSupportFoot(supportSide);
         comHeightManager.setSupportLeg(getSupportSide());
         loadFoot.set(false);
      }

      @Override
      public void doTransitionOutOfAction()
      {
         super.doTransitionOutOfAction();

         balanceManager.disablePelvisXYControl();
      }

      @Override
      public boolean isStateSafeToConsumePelvisTrajectoryCommand()
      {
         return true;
      }

      @Override
      public boolean isStateSafeToConsumeManipulationCommands()
      {
         return true;
      }

      public void handleEndEffectorLoadBearingCommand(EndEffectorLoadBearingCommand command)
      {
         if (command.getRequest(swingSide, EndEffector.FOOT) == LoadBearingRequest.LOAD)
            initiateFootLoadingProcedure(swingSide);
      }

      private void initiateFootLoadingProcedure(RobotSide swingSide)
      {
         loadFoot.set(true);
         loadFootStartTime.set(yoTime.getDoubleValue());
         balanceManager.setSingleSupportTime(loadFootDuration.getDoubleValue());
         balanceManager.setDoubleSupportTime(loadFootTransferDuration.getDoubleValue());
         balanceManager.clearICPPlan();
         balanceManager.addFootstepToPlan(walkingMessageHandler.getFootstepAtCurrentLocation(swingSide));
         balanceManager.initializeICPPlanForSingleSupport(swingSide.getOppositeSide());
         
         balanceManager.freezePelvisXYControl();
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

   private class AbortCondition implements StateTransitionCondition
   {
      @Override
      public boolean checkCondition()
      {
         if (!abortWalkingRequested.getBooleanValue())
            return false;

         walkingMessageHandler.clearFootsteps();
         walkingMessageHandler.reportWalkingAbortRequested();
         abortWalkingRequested.set(false);
         return true;
      }
   }

   private class StartWalkingCondition implements StateTransitionCondition
   {
      private final RobotSide transferToSide;

      public StartWalkingCondition(RobotSide transferToSide)
      {
         this.transferToSide = transferToSide;
      }

      @Override
      public boolean checkCondition()
      {
         boolean transferringToThisRobotSide;
         if (walkingMessageHandler.hasUpcomingFootsteps())
            transferringToThisRobotSide = transferToSide == walkingMessageHandler.peek(0).getRobotSide().getOppositeSide();
         else
            transferringToThisRobotSide = false;

         boolean shouldStartWalking = transferringToThisRobotSide;
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

         boolean startWalking = shouldStartWalking && isPreparedToWalk;
         if (startWalking)
            preparingForLocomotion.set(false);

         return startWalking;
      }
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
   private final RecyclingArrayList<PlaneContactStateCommand> planeContactStateCommandPool = new RecyclingArrayList<>(4, PlaneContactStateCommand.class);
   private final FramePoint2d capturePoint2d = new FramePoint2d();
   private final FramePoint2d desiredCapturePoint2d = new FramePoint2d();
   private final FrameVector achievedLinearMomentumRate = new FrameVector();

   @Override
   public void doMotionControl()
   {
      momentumBasedController.update();

      headOrientationManager.submitNewNeckJointDesiredConfiguration(controllerCoreOutput.getLowLevelOneDoFJointDesiredDataHolder());

      controllerCoreOutput.getLinearMomentumRate(achievedLinearMomentumRate);
      balanceManager.computeAchievedCMP(achievedLinearMomentumRate);

      consumeHeadCommands();
      consumeChestCommands();
      consumePelvisHeightCommands();
      consumeGoHomeMessages();
      consumeEndEffectorLoadBearingCommands();
      consumeStopAllTrajectoryCommands();
      consumeFootTrajectoryCommands();
      consumeAbortWalkingCommands();
      consumePelvisCommands();
      consumeManipulationCommands();
      handleAutomaticManipulationAbortOnICPError();

      balanceManager.getCapturePoint(capturePoint2d);
      balanceManager.getDesiredICP(desiredCapturePoint2d);
      failureDetectionControlModule.checkIfRobotIsFalling(capturePoint2d, desiredCapturePoint2d);
      if (failureDetectionControlModule.isRobotFalling())
      {
         if (enablePushRecoveryOnFailure.getBooleanValue() && !balanceManager.isPushRecoveryEnabled())
         {
            balanceManager.enablePushRecovery();
         }
         else if (!balanceManager.isPushRecoveryEnabled() || balanceManager.isRecoveryImpossible())
         {
            momentumBasedController.reportControllerFailureToListeners(failureDetectionControlModule.getFallingDirection());
         }
      }

      if (enablePushRecoveryOnFailure.getBooleanValue())
      {
         if (balanceManager.isPushRecoveryEnabled() && balanceManager.isRobotBackToSafeState())
            balanceManager.disablePushRecovery();
      }

      balanceManager.update();

      stateMachine.checkTransitionConditions();
      stateMachine.doAction();

      WalkingState currentState = stateMachine.getCurrentState();

      balanceManager.getDesiredICPVelocity(desiredICPVelocityAsFrameVector);
      boolean isInDoubleSupport = currentState.isDoubleSupportState();
      double omega0 = balanceManager.getOmega0();
      boolean isRecoveringFromPush = balanceManager.isRecovering();
      controlledCoMHeightAcceleration.set(comHeightManager.computeDesiredCoMHeightAcceleration(desiredICPVelocityAsFrameVector, isInDoubleSupport, omega0,
            isRecoveringFromPush, feetManager));

      doFootControl();
      doArmControl();
      doHeadControl();
      doChestControl();
      doPelvisControl();
      JointspaceFeedbackControlCommand unconstrainedJointCommand = doUnconstrainedJointControl();

      boolean keepCMPInsideSupportPolygon = true;
      if (manipulationControlModule != null && manipulationControlModule.isAtLeastOneHandLoadBearing())
         keepCMPInsideSupportPolygon = false;

      balanceManager.compute(currentState.getSupportSide(), controlledCoMHeightAcceleration.getDoubleValue(), keepCMPInsideSupportPolygon);      

      submitControllerCoreCommands(unconstrainedJointCommand);

      for (RobotSide robotSide : RobotSide.values)
      {
         controllerCoreOutput.getDesiredCenterOfPressure(footDesiredCoPs.get(robotSide), feet.get(robotSide).getRigidBody());
         momentumBasedController.setDesiredCenterOfPressure(feet.get(robotSide), footDesiredCoPs.get(robotSide));
      }

      momentumBasedController.doProportionalControlOnCoP(footDesiredCoPs);

      statusOutputManager.reportStatusMessage(balanceManager.updateAndReturnCapturabilityBasedStatus());
   }

   private void submitControllerCoreCommands(JointspaceFeedbackControlCommand unconstrainedJointCommand)
   {
      planeContactStateCommandPool.clear();

      controllerCoreCommand.addInverseDynamicsCommand(privilegedConfigurationCommand);
      

      boolean isHighCoPDampingNeeded = momentumBasedController.estimateIfHighCoPDampingNeeded(footDesiredCoPs);

      for (RobotSide robotSide : RobotSide.values)
      {
         controllerCoreCommand.addFeedbackControlCommand(feetManager.getFeedbackControlCommand(robotSide));
         controllerCoreCommand.addInverseDynamicsCommand(feetManager.getInverseDynamicsCommand(robotSide));

         controllerCoreCommand.addFeedbackControlCommand(manipulationControlModule.getFeedbackControlCommand(robotSide));
         controllerCoreCommand.addInverseDynamicsCommand(manipulationControlModule.getInverseDynamicsCommand(robotSide));
         controllerCoreCommand.completeLowLevelJointData(manipulationControlModule.getLowLevelJointDesiredData(robotSide));

         YoPlaneContactState contactState = momentumBasedController.getContactState(feet.get(robotSide));
         PlaneContactStateCommand planeContactStateCommand = planeContactStateCommandPool.add();
         contactState.getPlaneContactStateCommand(planeContactStateCommand);
         planeContactStateCommand.setUseHighCoPDamping(isHighCoPDampingNeeded);
         controllerCoreCommand.addInverseDynamicsCommand(planeContactStateCommand);
      }

      controllerCoreCommand.addFeedbackControlCommand(headOrientationManager.getFeedbackControlCommand());
      controllerCoreCommand.addInverseDynamicsCommand(headOrientationManager.getInverseDynamicsCommand());
      controllerCoreCommand.completeLowLevelJointData(headOrientationManager.getLowLevelJointDesiredData());
      
      controllerCoreCommand.addFeedbackControlCommand(chestOrientationManager.getFeedbackControlCommand());
      
      controllerCoreCommand.addFeedbackControlCommand(pelvisOrientationManager.getFeedbackControlCommand());

//      controllerCoreCommand.addInverseDynamicsCommand(getUncontrolledJointCommand());
//      controllerCoreCommand.addFeedbackControlCommand(unconstrainedJointCommand);

      controllerCoreCommand.addInverseDynamicsCommand(balanceManager.getInverseDynamicsCommand());
   }

   private void integrateAnkleAccelerationsOnSwingLeg(RobotSide swingSide)
   {
      fullRobotModel.getLegJoint(swingSide, LegJointName.ANKLE_PITCH).setIntegrateDesiredAccelerations(true);
      fullRobotModel.getLegJoint(swingSide, LegJointName.ANKLE_ROLL).setIntegrateDesiredAccelerations(true);
      fullRobotModel.getLegJoint(swingSide.getOppositeSide(), LegJointName.ANKLE_PITCH).setIntegrateDesiredAccelerations(false);
      fullRobotModel.getLegJoint(swingSide.getOppositeSide(), LegJointName.ANKLE_ROLL).setIntegrateDesiredAccelerations(false);
   }

   public void reinitializePelvisOrientation(boolean reinitialize)
   {
      hasWalkingControllerBeenInitialized.set(!reinitialize);
   }

   private void consumeHeadCommands()
   {
      if (commandInputManager.isNewCommandAvailable(HeadTrajectoryCommand.class))
         headOrientationManager.handleHeadTrajectoryCommand(commandInputManager.pollNewestCommand(HeadTrajectoryCommand.class));
      if (commandInputManager.isNewCommandAvailable(NeckTrajectoryCommand.class))
         headOrientationManager.handleNeckTrajectoryCommand(commandInputManager.pollNewestCommand(NeckTrajectoryCommand.class));
      if (commandInputManager.isNewCommandAvailable(NeckDesiredAccelerationsCommand.class))
         headOrientationManager.handleNeckDesiredAccelerationsCommand(commandInputManager.pollNewestCommand(NeckDesiredAccelerationsCommand.class));
   }

   private void consumeChestCommands()
   {
      if (commandInputManager.isNewCommandAvailable(ChestTrajectoryCommand.class))
         chestOrientationManager.handleChestTrajectoryCommand(commandInputManager.pollNewestCommand(ChestTrajectoryCommand.class));
   }

   private void consumePelvisCommands()
   {
      WalkingState currentState = stateMachine.getCurrentState();

      if (commandInputManager.isNewCommandAvailable(PelvisOrientationTrajectoryCommand.class))
      {
         PelvisOrientationTrajectoryCommand newestCommand = commandInputManager.pollNewestCommand(PelvisOrientationTrajectoryCommand.class);
         if (currentState.isStateSafeToConsumePelvisTrajectoryCommand())
            pelvisOrientationManager.handlePelvisOrientationTrajectoryCommands(newestCommand);
      }

      if (commandInputManager.isNewCommandAvailable(PelvisTrajectoryCommand.class))
      {
         PelvisTrajectoryCommand command = commandInputManager.pollNewestCommand(PelvisTrajectoryCommand.class);
         if (currentState.isStateSafeToConsumePelvisTrajectoryCommand())
         {
            pelvisOrientationManager.handlePelvisTrajectoryCommand(command);
            balanceManager.handlePelvisTrajectoryCommand(command);
            comHeightManager.handlePelvisTrajectoryCommand(command);
         }
      }
   }

   private void consumePelvisHeightCommands()
   {
      if (commandInputManager.isNewCommandAvailable(PelvisHeightTrajectoryCommand.class))
         comHeightManager.handlePelvisHeightTrajectoryCommand(commandInputManager.pollNewestCommand(PelvisHeightTrajectoryCommand.class));
   }

   private void consumeManipulationCommands()
   {
      if (manipulationControlModule == null)
         return;

      if (yoTime.getDoubleValue() - timeOfLastManipulationAbortRequest.getDoubleValue() < manipulationIgnoreInputsDurationAfterAbort.getDoubleValue())
      {
         commandInputManager.flushCommands(HandTrajectoryCommand.class);
         commandInputManager.flushCommands(ArmTrajectoryCommand.class);
         commandInputManager.flushCommands(ArmDesiredAccelerationsCommand.class);
         commandInputManager.flushCommands(HandComplianceControlParametersCommand.class);
         return;
      }

      List<HandTrajectoryCommand> handTrajectoryCommands = commandInputManager.pollNewCommands(HandTrajectoryCommand.class);
      List<ArmTrajectoryCommand> armTrajectoryCommands = commandInputManager.pollNewCommands(ArmTrajectoryCommand.class);
      List<ArmDesiredAccelerationsCommand> armDesiredAccelerationCommands = commandInputManager.pollNewCommands(ArmDesiredAccelerationsCommand.class);
      List<HandComplianceControlParametersCommand> handComplianceCommands = commandInputManager.pollNewCommands(HandComplianceControlParametersCommand.class);

      WalkingState currentState = stateMachine.getCurrentState();

      if (currentState.isStateSafeToConsumeManipulationCommands())
      {
         manipulationControlModule.handleHandTrajectoryCommands(handTrajectoryCommands);
         manipulationControlModule.handleArmTrajectoryCommands(armTrajectoryCommands);
         manipulationControlModule.handleArmDesiredAccelerationsCommands(armDesiredAccelerationCommands);
         manipulationControlModule.handleHandComplianceControlParametersCommands(handComplianceCommands);
      }
   }

   private void handleAutomaticManipulationAbortOnICPError()
   {
      if (!stateMachine.getCurrentState().isStateSafeToConsumeManipulationCommands())
         return;

      if (commandInputManager.isNewCommandAvailable(AutomaticManipulationAbortCommand.class))
      {
         AutomaticManipulationAbortCommand message = commandInputManager.pollNewestCommand(AutomaticManipulationAbortCommand.class);
         isAutomaticManipulationAbortEnabled.set(message.isEnable());
      }

      if (!isAutomaticManipulationAbortEnabled.getBooleanValue())
         return;

      if (yoTime.getDoubleValue() - timeOfLastManipulationAbortRequest.getDoubleValue() < minimumDurationBetweenTwoManipulationAborts.getDoubleValue())
         return;

      if (balanceManager.getICPErrorMagnitude() > icpErrorThresholdToAbortManipulation.getDoubleValue())
      {
         hasManipulationBeenAborted.set(true);
         manipulationControlModule.freeze();
         timeOfLastManipulationAbortRequest.set(yoTime.getDoubleValue());

         statusOutputManager.reportStatusMessage(new ManipulationAbortedStatus());
      }
      else
      {
         hasManipulationBeenAborted.set(false);
      }
   }
   private void consumeGoHomeMessages()
   {
      if (!commandInputManager.isNewCommandAvailable(GoHomeCommand.class))
         return;

      GoHomeCommand command = commandInputManager.pollAndCompileCommands(GoHomeCommand.class);
      manipulationControlModule.handleGoHomeCommand(command);
      pelvisOrientationManager.handleGoHomeCommand(command);
      balanceManager.handleGoHomeCommand(command);
      chestOrientationManager.handleGoHomeCommand(command);
   }

   private void consumeEndEffectorLoadBearingCommands()
   {
      if (!commandInputManager.isNewCommandAvailable(EndEffectorLoadBearingCommand.class))
         return;

      EndEffectorLoadBearingCommand command = commandInputManager.pollAndCompileCommands(EndEffectorLoadBearingCommand.class);
      manipulationControlModule.handleEndEffectorLoadBearingCommand(command);
      WalkingState currentState = stateMachine.getCurrentState();
      currentState.handleEndEffectorLoadBearingCommand(command);
   }

   private void consumeStopAllTrajectoryCommands()
   {
      if (!commandInputManager.isNewCommandAvailable(StopAllTrajectoryCommand.class))
         return;

      StopAllTrajectoryCommand command = commandInputManager.pollNewestCommand(StopAllTrajectoryCommand.class);
      manipulationControlModule.handleStopAllTrajectoryCommand(command);
      chestOrientationManager.handleStopAllTrajectoryCommand(command);
      feetManager.handleStopAllTrajectoryCommand(command);
      comHeightManager.handleStopAllTrajectoryCommand(command);
      balanceManager.handleStopAllTrajectoryCommand(command);
      pelvisOrientationManager.handleStopAllTrajectoryCommand(command);
   }

   private void consumeFootTrajectoryCommands()
   {
      if (commandInputManager.isNewCommandAvailable(FootTrajectoryCommand.class))
         walkingMessageHandler.handleFootTrajectoryCommand(commandInputManager.pollNewCommands(FootTrajectoryCommand.class));

      if (commandInputManager.isNewCommandAvailable(PauseWalkingCommand.class))
         walkingMessageHandler.handlePauseWalkingCommand(commandInputManager.pollNewestCommand(PauseWalkingCommand.class));

      if (commandInputManager.isNewCommandAvailable(FootstepDataListCommand.class))
      {
         walkingMessageHandler.handleFootstepDataListCommand(commandInputManager.pollNewestCommand(FootstepDataListCommand.class));
         balanceManager.setDoubleSupportTime(walkingMessageHandler.getTransferTime());
         balanceManager.setSingleSupportTime(walkingMessageHandler.getSwingTime());
      }
   }

   private void consumeAbortWalkingCommands()
   {
      if (!commandInputManager.isNewCommandAvailable(AbortWalkingCommand.class))
         return;
      abortWalkingRequested.set(commandInputManager.pollNewestCommand(AbortWalkingCommand.class).isAbortWalkingRequested());
   }

   @Override
   public ControllerCoreCommand getControllerCoreCommand()
   {
      return controllerCoreCommand;
   }
}
