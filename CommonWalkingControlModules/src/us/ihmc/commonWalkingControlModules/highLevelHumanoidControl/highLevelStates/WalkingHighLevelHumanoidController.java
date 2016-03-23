package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.SdfLoader.partNames.ArmJointName;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.SdfLoader.partNames.LimbName;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
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
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PauseWalkingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisHeightTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisOrientationTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;
import us.ihmc.humanoidRobotics.communication.packets.walking.ManipulationAbortedStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage.EndEffector;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage.LoadBearingRequest;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
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
import us.ihmc.robotics.stateMachines.State;
import us.ihmc.robotics.stateMachines.StateChangedListener;
import us.ihmc.robotics.stateMachines.StateMachine;
import us.ihmc.robotics.stateMachines.StateTransition;
import us.ihmc.robotics.stateMachines.StateTransitionCondition;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.tools.io.printing.PrintTools;

public class WalkingHighLevelHumanoidController extends AbstractHighLevelHumanoidControlPattern
{
   private final static HighLevelState controllerState = HighLevelState.WALKING;

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

   private final EnumYoVariable<RobotSide> supportLeg = new EnumYoVariable<>("supportLeg", registry, RobotSide.class, true);
   private final EnumYoVariable<RobotSide> trailingLeg = new EnumYoVariable<RobotSide>("trailingLeg", "", registry, RobotSide.class, true);
   private final EnumYoVariable<RobotSide> lastPlantedLeg = new EnumYoVariable<RobotSide>("lastPlantedLeg", "", registry, RobotSide.class, true);

   private final BipedSupportPolygons bipedSupportPolygons;

   private final DoubleYoVariable controlledCoMHeightAcceleration = new DoubleYoVariable("controlledCoMHeightAcceleration", registry);

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

      stateMachine = new StateMachine<WalkingState>(namePrefix + "State", namePrefix + "SwitchTime", WalkingState.class, yoTime, registry); // this is used by name, and it is ugly.

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

      double highCoPDampingDuration = walkingControllerParameters.getHighCoPDampingDurationToPreventFootShakies();
      double coPErrorThreshold = walkingControllerParameters.getCoPErrorThresholdForHighCoPDamping();
      boolean enableHighCoPDamping = highCoPDampingDuration > 0.0 && !Double.isInfinite(coPErrorThreshold);
      momentumBasedController.setHighCoPDampingParameters(enableHighCoPDamping, highCoPDampingDuration, coPErrorThreshold);
   }

   private void setupStateMachine()
   {
      DoubleSupportState doubleSupportState = new DoubleSupportState(null);
      SideDependentList<State<WalkingState>> transferStates = new SideDependentList<>();
      SideDependentList<StateTransition<WalkingState>> fromFallingToSingleSupportsTransitions = new SideDependentList<>();

      stateMachine.addState(doubleSupportState);

      for (RobotSide robotSide : RobotSide.values)
      {
         WalkingState doubleSupportStateEnum = doubleSupportState.getStateEnum();
         WalkingState singleSupportStateEnum = WalkingState.getSingleSupportState(robotSide);
         WalkingState transferStateEnum = WalkingState.getTransferState(robotSide);
         WalkingState oppTransferStateEnum = WalkingState.getTransferState(robotSide.getOppositeSide());

         State<WalkingState> transferState = new DoubleSupportState(robotSide);
         State<WalkingState> singleSupportState = new SingleSupportState(robotSide);

         transferStates.put(robotSide, transferState);

         StopWalkingFromTransferCondition stopWalkingFromTranferCondition = new StopWalkingFromTransferCondition(robotSide);
         StopWalkingFromSingleSupportCondition stopWalkingFromSingleSupportCondition = new StopWalkingFromSingleSupportCondition(robotSide);
         DoneWithTransferCondition doneWithTransferCondition = new DoneWithTransferCondition(robotSide);
         SingleSupportToTransferToCondition singleSupportToTransferToOppositeSideCondition = new SingleSupportToTransferToCondition(robotSide, robotSide.getOppositeSide());
         SingleSupportToTransferToCondition singleSupportToTransferToSameSideCondition = new SingleSupportToTransferToCondition(robotSide, robotSide);
         StartWalkingCondition startWalkingCondition = new StartWalkingCondition(robotSide);
         FlamingoStanceCondition flamingoStanceCondition = new FlamingoStanceCondition(robotSide);

         StateTransition<WalkingState> fromTransferToDoubleSupport = new StateTransition<WalkingState>(doubleSupportStateEnum, stopWalkingFromTranferCondition);
         StateTransition<WalkingState> fromSingleSupportToDoubleSupport = new StateTransition<WalkingState>(doubleSupportStateEnum, stopWalkingFromSingleSupportCondition);
         StateTransition<WalkingState> toSingleSupport = new StateTransition<WalkingState>(singleSupportStateEnum, doneWithTransferCondition);
         StateTransition<WalkingState> toTransferOppositeSide = new StateTransition<WalkingState>(oppTransferStateEnum,
               singleSupportToTransferToOppositeSideCondition);
         StateTransition<WalkingState> toTransferSameSide = new StateTransition<WalkingState>(transferStateEnum, singleSupportToTransferToSameSideCondition);
         StateTransition<WalkingState> toTransfer = new StateTransition<WalkingState>(transferStateEnum, startWalkingCondition);
         StateTransition<WalkingState> toTransferForFlamingo = new StateTransition<WalkingState>(transferStateEnum, flamingoStanceCondition);

         transferState.addStateTransition(fromTransferToDoubleSupport);
         transferState.addStateTransition(toSingleSupport);
         singleSupportState.addStateTransition(toTransferOppositeSide);
         singleSupportState.addStateTransition(toTransferSameSide);
         singleSupportState.addStateTransition(fromSingleSupportToDoubleSupport);
         doubleSupportState.addStateTransition(toTransfer);
         doubleSupportState.addStateTransition(toTransferForFlamingo);

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
      stateMachine.setCurrentState(WalkingState.DOUBLE_SUPPORT);

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

   private class DoubleSupportState extends State<WalkingState>
   {
      private final RobotSide transferToSide;
      private final FramePoint2d desiredICPLocal = new FramePoint2d();
      private final FramePoint2d capturePoint2d = new FramePoint2d();
      private final FramePoint2d desiredCMP = new FramePoint2d();

      public DoubleSupportState(RobotSide transferToSide)
      {
         super(transferToSide == null ? WalkingState.DOUBLE_SUPPORT : WalkingState.getTransferState(transferToSide));
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

         switchToToeOffIfPossible();

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

      private void handlePelvisTrajectoryMessage(PelvisTrajectoryCommand message)
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

         if (commandInputManager.isNewCommandAvailable(AutomaticManipulationAbortCommand.class))
         {
            AutomaticManipulationAbortCommand message = commandInputManager.pollNewestCommand(AutomaticManipulationAbortCommand.class);
            isAutomaticManipulationAbortEnabled.set(message.isEnable());
         }

         if (!isAutomaticManipulationAbortEnabled.getBooleanValue())
         {
            return;
         }

         if (yoTime.getDoubleValue() - timeOfLastManipulationAbortRequest.getDoubleValue() < minimumDurationBetweenTwoManipulationAborts.getDoubleValue())
            return;

         balanceManager.getCapturePoint(capturePoint2d);
         balanceManager.getDesiredICP(desiredICPLocal);

         if (capturePoint2d.distance(desiredICPLocal) > icpErrorThresholdToAbortManipulation.getDoubleValue())
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

      public void switchToToeOffIfPossible()
      {
         // the only case left for determining the contact state of the trailing foot
         if (!isPerformingToeOff.getBooleanValue() && transferToSide != null)
         {
            RobotSide trailingLeg = transferToSide.getOppositeSide();

            double predictedToeOffDuration = balanceManager.getTimeRemainingInCurrentState();

            balanceManager.getDesiredCMP(desiredCMP);
            balanceManager.getDesiredICP(desiredICPLocal);
            balanceManager.getCapturePoint(capturePoint2d);

            boolean doToeOff = feetManager.checkIfToeOffSafe(trailingLeg, desiredCMP, desiredICPLocal, capturePoint2d);

            if (doToeOff)
            {
               feetManager.requestToeOff(trailingLeg, predictedToeOffDuration);
               momentumBasedController.updateBipedSupportPolygons(); // need to always update biped support polygons after a change to the contact states
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

         balanceManager.clearICPPlan();
         balanceManager.setDoubleSupportTime(walkingMessageHandler.getTransferTime());
         balanceManager.setSingleSupportTime(walkingMessageHandler.getSwingTime());

         supportLeg.set(null);
         isPerformingToeOff.set(false);
         feetManager.initializeContactStatesForDoubleSupport(transferToSide);
         momentumBasedController.updateBipedSupportPolygons(); // need to always update biped support polygons after a change to the contact states

         commandInputManager.flushCommands(PelvisTrajectoryCommand.class);

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

         balanceManager.resetPushRecovery();

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
            balanceManager.initializeICPPlanForTransfer(transferToSide);
         }
         else if (hasFootTrajectoryMessage)
         {
            balanceManager.setSingleSupportTime(Double.POSITIVE_INFINITY);
            balanceManager.addFootstepToPlan(walkingMessageHandler.getFootstepAtCurrentLocation(transferToSide.getOppositeSide()));
            balanceManager.initializeICPPlanForTransfer(transferToSide);
         }
         else
         {
            if (holdICPToCurrentCoMLocationInNextDoubleSupport.getBooleanValue())
            {
               balanceManager.requestICPPlannerToHoldCurrentCoM(distanceToShrinkSupportPolygonWhenHoldingCurrent.getDoubleValue());
               holdICPToCurrentCoMLocationInNextDoubleSupport.set(false);
            }

            balanceManager.initializeICPPlanForStanding();
         }
      }

      @Override
      public void doTransitionOutOfAction()
      {
         preparingForLocomotion.set(false);
         balanceManager.disablePelvisXYControl();
         feetManager.reset();
         isPerformingToeOff.set(false);

         commandInputManager.flushCommands(PelvisTrajectoryCommand.class);

         if (transferToSide == null)
            momentumBasedController.reportChangeOfRobotMotionStatus(RobotMotionStatus.IN_MOTION);
      }
   }

   private Footstep previousDesiredFootstep;

   private class SingleSupportState extends State<WalkingState>
   {
      private final RobotSide swingSide;
      private final FramePoint2d capturePoint2d = new FramePoint2d();

      private Footstep nextFootstep;

      private final FramePose actualFootPoseInWorld;

      public SingleSupportState(RobotSide supportSide)
      {
         super(WalkingState.getSingleSupportState(supportSide));
         swingSide = supportSide.getOppositeSide();
         actualFootPoseInWorld = new FramePose(worldFrame);
      }

      @Override
      public void doAction()
      {
         RobotSide supportSide = swingSide.getOppositeSide();
         integrateAnkleAccelerationsOnSwingLeg(swingSide);

         if (isInFlamingoStance.getBooleanValue() && walkingMessageHandler.hasFootTrajectoryForFlamingoStance(swingSide))
         {
            feetManager.handleFootTrajectoryMessage(walkingMessageHandler.pollFootTrajectoryForFlamingoStance(swingSide));
         }

         balanceManager.getCapturePoint(capturePoint2d);

         boolean icpErrorIsTooLarge = balanceManager.getICPErrorMagnitude() > icpErrorThresholdToSpeedUpSwing.getDoubleValue();

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
         else if (balanceManager.isPushRecoveryEnabled())
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

         if (isInFlamingoStance.getBooleanValue())
         {
            consumePelvisMessages();
            consumeManipulationMessages();
         }
         else
         {
            walkingMessageHandler.clearFootTrajectory();
         }

         switchToToeOffIfPossible(supportSide);
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

      private void handlePelvisTrajectoryMessage(PelvisTrajectoryCommand message)
      {
         if (!isInFlamingoStance.getBooleanValue() || loadFoot.getBooleanValue())
            return;

         pelvisOrientationManager.handlePelvisTrajectoryMessage(message);
         balanceManager.handlePelvisTrajectoryMessage(message);
      }

      private void handleEndEffectorLoadBearingRequest(EndEffectorLoadBearingCommand message)
      {
         if (isInFlamingoStance.getBooleanValue() && message.getRequest(swingSide, EndEffector.FOOT) == LoadBearingRequest.LOAD)
            initiateFootLoadingProcedure(swingSide);
      }

      private final FramePoint nextExitCMP = new FramePoint();
      private final FramePoint2d toeOffContactPoint = new FramePoint2d();

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

      @Override
      public void doTransitionIntoAction()
      {
         lastPlantedLeg.set(swingSide.getOppositeSide());
         trailingLeg.set(null);

         RobotSide supportSide = swingSide.getOppositeSide();
         supportLeg.set(supportSide);
         balanceManager.clearICPPlan();

         footSwitches.get(swingSide).reset();

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

            balanceManager.addFootstepToPlan(nextFootstep);
            balanceManager.addFootstepToPlan(walkingMessageHandler.peek(0));
            balanceManager.addFootstepToPlan(walkingMessageHandler.peek(1));
            balanceManager.initializeICPPlanForSingleSupport(supportSide);

            if (balanceManager.isRecoveringFromDoubleSupportFall())
            {
               balanceManager.updateICPPlanForSingleSupportDisturbances();
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
         momentumBasedController.updateBipedSupportPolygons();

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

         balanceManager.resetPushRecovery();

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
      balanceManager.clearICPPlan();
      balanceManager.addFootstepToPlan(walkingMessageHandler.getFootstepAtCurrentLocation(swingSide));
      balanceManager.initializeICPPlanForSingleSupport(supportLeg.getEnumValue());

      balanceManager.freezePelvisXYControl();
   }

   private class DoubleSupportToSingleSupportConditionForDisturbanceRecovery implements StateTransitionCondition
   {
      private final RobotSide transferToSide;

      public DoubleSupportToSingleSupportConditionForDisturbanceRecovery(RobotSide robotSide)
      {
         transferToSide = robotSide;
      }

      @Override
      public boolean checkCondition()
      {
         if (!balanceManager.isPushRecoveryEnabled())
            return false;

         RobotSide suggestedSwingSide = balanceManager.isRobotFallingFromDoubleSupport();
         boolean isRobotFalling = suggestedSwingSide != null;

         if (!isRobotFalling)
            return false;

         boolean switchToSingleSupport = transferToSide != suggestedSwingSide;

         if (switchToSingleSupport)
            balanceManager.prepareForDoubleSupportPushRecovery();

         return switchToSingleSupport;
      }
   }

   public class StartWalkingCondition implements StateTransitionCondition
   {
      private final RobotSide transferToSide;

      public StartWalkingCondition(RobotSide robotSide)
      {
         transferToSide = robotSide;
      }

      @Override
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
         transferToSide = robotSide;
      }

      @Override
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
      private final FrameVector2d icpError2d = new FrameVector2d();

      public DoneWithTransferCondition(RobotSide robotSide)
      {
         this.robotSide = robotSide;
      }

      @Override
      public boolean checkCondition()
      {
         boolean icpTrajectoryIsDone = balanceManager.isICPPlanDone();

         if (!icpTrajectoryIsDone)
            return false;

         balanceManager.getICPError(icpError2d);
         icpError2d.changeFrame(referenceFrames.getAnkleZUpFrame(robotSide));

         double ellipticErrorSquared = MathTools.square(icpError2d.getX() / maxICPErrorBeforeSingleSupportX.getDoubleValue())
               + MathTools.square(icpError2d.getY() / maxICPErrorBeforeSingleSupportY.getDoubleValue());
         boolean closeEnough = ellipticErrorSquared < 1.0;

         return closeEnough;
      }
   }

   private class SingleSupportToTransferToCondition extends DoneWithSingleSupportCondition
   {
      private final RobotSide transferToSide;

      public SingleSupportToTransferToCondition(RobotSide supportSide, RobotSide transferToSide)
      {
         super(supportSide);

         this.transferToSide = transferToSide;
      }

      @Override
      public boolean checkCondition()
      {
         if (!super.checkCondition())
            return false;

         boolean hasNextFootstepForThisSide = walkingMessageHandler.isNextFootstepFor(transferToSide.getOppositeSide());
         return hasNextFootstepForThisSide;
      }
   }

   private class DoneWithSingleSupportCondition implements StateTransitionCondition
   {
      private final RobotSide supportSide;

      public DoneWithSingleSupportCondition(RobotSide supportSide)
      {
         this.supportSide = supportSide;
      }

      @Override
      public boolean checkCondition()
      {
         RobotSide swingSide = supportSide.getOppositeSide();
         hasMinimumTimePassed.set(hasMinimumTimePassed());

         if (!hasMinimumTimePassed.getBooleanValue())
            return false;

         boolean finishSingleSupportWhenICPPlannerIsDone = walkingControllerParameters.finishSingleSupportWhenICPPlannerIsDone();// || pushRecoveryModule.isRecovering();

         if (finishSingleSupportWhenICPPlannerIsDone && !isInFlamingoStance.getBooleanValue())
         {
            if (balanceManager.isICPPlanDone())
               return true;
         }

         if (loadFoot.getBooleanValue() && yoTime.getDoubleValue() > loadFootStartTime.getDoubleValue() + loadFootDuration.getDoubleValue())
         {
            loadFoot.set(false);
            return true;
         }

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
   }

   private class StopWalkingFromSingleSupportCondition extends DoneWithSingleSupportCondition
   {
      private final RobotSide supportSide;

      public StopWalkingFromSingleSupportCondition(RobotSide supportSide)
      {
         super(supportSide);

         this.supportSide = supportSide;
      }

      @Override
      public boolean checkCondition()
      {
         if (abortWalkingRequested.getBooleanValue())
         {
            return true;
         }

         if (!super.checkCondition())
            return false;

         boolean noMoreFootsteps = !walkingMessageHandler.hasUpcomingFootsteps();
         boolean noMoreFootTrajectoryMessages = !walkingMessageHandler.hasFootTrajectoryForFlamingoStance(supportSide.getOppositeSide());
         boolean readyToStopWalking = noMoreFootsteps && noMoreFootTrajectoryMessages;
         return readyToStopWalking;
      }
   }

   private class StopWalkingFromTransferCondition implements StateTransitionCondition
   {
      private final RobotSide supportSide;

      public StopWalkingFromTransferCondition(RobotSide supportSide)
      {
         this.supportSide = supportSide;
      }

      @Override
      public boolean checkCondition()
      {
         if (abortWalkingRequested.getBooleanValue())
         {
            return true;
         }

         boolean noMoreFootstepsForThisSide = !walkingMessageHandler.isNextFootstepFor(supportSide.getOppositeSide());
         boolean noMoreFootTrajectoryMessages = !walkingMessageHandler.hasFootTrajectoryForFlamingoStance(supportSide.getOppositeSide());
         boolean readyToStopWalking = noMoreFootstepsForThisSide && noMoreFootTrajectoryMessages;
         return readyToStopWalking;
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

      manipulationControlModule.submitNewArmJointDesiredConfiguration(controllerCoreOutput.getLowLevelOneDoFJointDesiredDataHolder());

      consumeHeadMessages();
      consumeChestMessages();
      consumePelvisHeightMessages();
      consumeGoHomeMessages();
      consumeEndEffectorLoadBearingMessages();
      consumeStopAllTrajectoryMessages();
      consumeFootTrajectoryMessages();
      consumeAbortWalkingMessages();

      controllerCoreOutput.getLinearMomentumRate(achievedLinearMomentumRate);
      balanceManager.computeAchievedCMP(achievedLinearMomentumRate);

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

      balanceManager.getDesiredICPVelocity(desiredICPVelocityAsFrameVector);
      boolean isInDoubleSupport = supportLeg.getEnumValue() == null;
      double omega0 = balanceManager.getOmega0();
      boolean isRecoveringFromPush = balanceManager.isRecovering();
      controlledCoMHeightAcceleration.set(comHeightManager.computeDesiredCoMHeightAcceleration(desiredICPVelocityAsFrameVector, isInDoubleSupport, omega0,
            trailingLeg.getEnumValue(), isRecoveringFromPush, feetManager));

      doFootControl();
      doArmControl();
      doHeadControl();
      doChestControl();
      doPelvisControl();
      JointspaceFeedbackControlCommand unconstrainedJointCommand = doUnconstrainedJointControl();

      boolean keepCMPInsideSupportPolygon = true;
      if (manipulationControlModule != null && manipulationControlModule.isAtLeastOneHandLoadBearing())
         keepCMPInsideSupportPolygon = false;

      balanceManager.compute(supportLeg.getEnumValue(), controlledCoMHeightAcceleration.getDoubleValue(), keepCMPInsideSupportPolygon);      

      submitControllerCoreCommands(unconstrainedJointCommand);

      for (RobotSide robotSide : RobotSide.values)
      {
         controllerCoreOutput.getDesiredCenterOfPressure(footDesiredCoPs.get(robotSide), feet.get(robotSide).getRigidBody());
         momentumBasedController.setDesiredCenterOfPressure(feet.get(robotSide), footDesiredCoPs.get(robotSide));
      }

      momentumBasedController.doProportionalControlOnCoP(footDesiredCoPs);

      statusOutputManager.reportStatusMessage(balanceManager.updateAndReturnCapturabilityBasedStatus());
   }

   public void submitControllerCoreCommands(JointspaceFeedbackControlCommand unconstrainedJointCommand)
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
      if (commandInputManager.isNewCommandAvailable(HeadTrajectoryCommand.class))
         headOrientationManager.handleHeadTrajectoryMessage(commandInputManager.pollNewestCommand(HeadTrajectoryCommand.class));
   }

   private void consumeChestMessages()
   {
      if (commandInputManager.isNewCommandAvailable(ChestTrajectoryCommand.class))
         chestOrientationManager.handleChestTrajectoryMessage(commandInputManager.pollNewestCommand(ChestTrajectoryCommand.class));
   }

   private void consumePelvisMessages()
   {
      if (commandInputManager.isNewCommandAvailable(PelvisOrientationTrajectoryCommand.class))
         pelvisOrientationManager
               .handlePelvisOrientationTrajectoryMessages(commandInputManager.pollNewestCommand(PelvisOrientationTrajectoryCommand.class));

      if (commandInputManager.isNewCommandAvailable(PelvisTrajectoryCommand.class))
      {
         PelvisTrajectoryCommand message = commandInputManager.pollNewestCommand(PelvisTrajectoryCommand.class);
         State<WalkingState> currentState = stateMachine.getCurrentState();
         if (currentState instanceof DoubleSupportState)
            ((DoubleSupportState) currentState).handlePelvisTrajectoryMessage(message);
         else if (currentState instanceof SingleSupportState)
            ((SingleSupportState) currentState).handlePelvisTrajectoryMessage(message);
      }
   }

   private void consumePelvisHeightMessages()
   {
      if (commandInputManager.isNewCommandAvailable(PelvisHeightTrajectoryCommand.class))
         comHeightManager.handlePelvisHeightTrajectoryMessage(commandInputManager.pollNewestCommand(PelvisHeightTrajectoryCommand.class));
   }

   private void consumeManipulationMessages()
   {
      if (yoTime.getDoubleValue() - timeOfLastManipulationAbortRequest.getDoubleValue() < manipulationIgnoreInputsDurationAfterAbort.getDoubleValue())
      {
         commandInputManager.flushCommands(HandTrajectoryCommand.class);
         commandInputManager.flushCommands(ArmTrajectoryCommand.class);
         commandInputManager.flushCommands(ArmDesiredAccelerationsCommand.class);
         commandInputManager.flushCommands(HandComplianceControlParametersCommand.class);
         return;
      }

      manipulationControlModule.handleHandTrajectoryMessages(commandInputManager.pollNewCommands(HandTrajectoryCommand.class));
      manipulationControlModule.handleArmTrajectoryMessages(commandInputManager.pollNewCommands(ArmTrajectoryCommand.class));
      manipulationControlModule.handleArmDesiredAccelerationsMessages(commandInputManager.pollNewCommands(ArmDesiredAccelerationsCommand.class));
      manipulationControlModule.handleHandComplianceControlParametersMessages(commandInputManager.pollNewCommands(HandComplianceControlParametersCommand.class));
   }

   private void consumeGoHomeMessages()
   {
      if (!commandInputManager.isNewCommandAvailable(GoHomeCommand.class))
         return;

      GoHomeCommand message = commandInputManager.pollAndCompileCommands(GoHomeCommand.class);
      manipulationControlModule.handleGoHomeMessage(message);

      pelvisOrientationManager.handleGoHomeMessage(message);
      balanceManager.handleGoHomeMessage(message);
      chestOrientationManager.handleGoHomeMessage(message);
   }

   private void consumeEndEffectorLoadBearingMessages()
   {
      if (!commandInputManager.isNewCommandAvailable(EndEffectorLoadBearingCommand.class))
         return;

      EndEffectorLoadBearingCommand message = commandInputManager.pollAndCompileCommands(EndEffectorLoadBearingCommand.class);
      manipulationControlModule.handleEndEffectorLoadBearingMessage(message);
      State<WalkingState> currentState = stateMachine.getCurrentState();
      if (currentState instanceof SingleSupportState)
         ((SingleSupportState) currentState).handleEndEffectorLoadBearingRequest(message);
   }

   private void consumeStopAllTrajectoryMessages()
   {
      if (!commandInputManager.isNewCommandAvailable(StopAllTrajectoryCommand.class))
         return;

      StopAllTrajectoryCommand message = commandInputManager.pollNewestCommand(StopAllTrajectoryCommand.class);
      manipulationControlModule.handleStopAllTrajectoryMessage(message);
      chestOrientationManager.handleStopAllTrajectoryMessage(message);
      feetManager.handleStopAllTrajectoryMessage(message);
      comHeightManager.handleStopAllTrajectoryMessage(message);
      balanceManager.handleStopAllTrajectoryMessage(message);
      pelvisOrientationManager.handleStopAllTrajectoryMessage(message);
   }

   private void consumeFootTrajectoryMessages()
   {
      if (commandInputManager.isNewCommandAvailable(FootTrajectoryCommand.class))
         walkingMessageHandler.handleFootTrajectoryMessage(commandInputManager.pollNewestCommand(FootTrajectoryCommand.class));

      if (commandInputManager.isNewCommandAvailable(PauseWalkingCommand.class))
         walkingMessageHandler.handlePauseWalkingMessage(commandInputManager.pollNewestCommand(PauseWalkingCommand.class));

      if (commandInputManager.isNewCommandAvailable(FootstepDataListCommand.class))
         walkingMessageHandler.handleFootstepDataListMessage(commandInputManager.pollNewestCommand(FootstepDataListCommand.class));
   }

   private void consumeAbortWalkingMessages()
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
