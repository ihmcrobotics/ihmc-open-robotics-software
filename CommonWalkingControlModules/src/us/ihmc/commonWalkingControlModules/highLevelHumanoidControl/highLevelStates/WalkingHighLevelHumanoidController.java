package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitEnforcementMethodCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.commonWalkingControlModules.desiredFootStep.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.WalkingCommandConsumer;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.stateTransitionConditions.DoubSuppToSingSuppCond4DistRecov;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.stateTransitionConditions.SingleSupportToTransferToCondition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.stateTransitionConditions.StartFlamingoCondition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.stateTransitionConditions.StartWalkingCondition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.stateTransitionConditions.StopFlamingoCondition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.stateTransitionConditions.StopWalkingFromSingleSupportCondition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.stateTransitionConditions.StopWalkingFromTransferCondition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.FlamingoStanceState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.SingleSupportState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.StandingState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.TransferToFlamingoStanceState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.TransferToStandingState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.TransferToWalkingSingleSupportState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingSingleSupportState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.BalanceManager;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitEnforcement;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.GenericStateMachine;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.State;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateChangedListener;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransition;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;

public class WalkingHighLevelHumanoidController extends HighLevelBehavior
{
   private final static HighLevelState controllerState = HighLevelState.WALKING;

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final DoubleYoVariable yoTime;

   private final HighLevelControlManagerFactory managerFactory;

   private final PelvisOrientationManager pelvisOrientationManager;
   private final FeetManager feetManager;
   private final BalanceManager balanceManager;
   private final CenterOfMassHeightManager comHeightManager;

   private final ArrayList<RigidBodyControlManager> bodyManagers = new ArrayList<>();

   private final OneDoFJoint[] allOneDoFjoints;

   private final FullHumanoidRobotModel fullRobotModel;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WalkingControllerParameters walkingControllerParameters;

   private final SideDependentList<? extends ContactablePlaneBody> feet;

   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private final GenericStateMachine<WalkingStateEnum, WalkingState> stateMachine;

   private final WalkingMessageHandler walkingMessageHandler;
   private final BooleanYoVariable abortWalkingRequested = new BooleanYoVariable("requestAbortWalking", registry);

   private final DoubleYoVariable controlledCoMHeightAcceleration = new DoubleYoVariable("controlledCoMHeightAcceleration", registry);

   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final BooleanYoVariable hasWalkingControllerBeenInitialized = new BooleanYoVariable("hasWalkingControllerBeenInitialized", registry);

   private final BooleanYoVariable hasICPPlannerBeenInitializedAtStart = new BooleanYoVariable("hasICPPlannerBeenInitializedAtStart", registry);

   private final BooleanYoVariable enablePushRecoveryOnFailure = new BooleanYoVariable("enablePushRecoveryOnFailure", registry);

   private final BooleanYoVariable allowUpperBodyMotionDuringLocomotion = new BooleanYoVariable("allowUpperBodyMotionDuringLocomotion", registry);

   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusOutputManager;
   private final WalkingCommandConsumer commandConsumer;

   private final JointLimitEnforcementMethodCommand jointLimitEnforcementMethodCommand = new JointLimitEnforcementMethodCommand();
   private final BooleanYoVariable limitCommandSent = new BooleanYoVariable("limitCommandSent", registry);

   private final PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);
   private ControllerCoreOutputReadOnly controllerCoreOutput;

   public WalkingHighLevelHumanoidController(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
         HighLevelControlManagerFactory managerFactory, WalkingControllerParameters walkingControllerParameters,
         CapturePointPlannerParameters capturePointPlannerParameters, HighLevelHumanoidControllerToolbox controllerToolbox)
   {
      super(controllerState);

      this.managerFactory = managerFactory;

      this.yoGraphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();

      // Getting parameters from the HighLevelHumanoidControllerToolbox
      this.controllerToolbox = controllerToolbox;
      fullRobotModel = controllerToolbox.getFullRobotModel();
      yoTime = controllerToolbox.getYoTime();

      feet = controllerToolbox.getContactableFeet();

      allOneDoFjoints = fullRobotModel.getOneDoFJoints();

      this.pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      this.feetManager = managerFactory.getOrCreateFeetManager();

      RigidBody head = fullRobotModel.getHead();
      RigidBody chest = fullRobotModel.getChest();
      RigidBody pelvis = fullRobotModel.getPelvis();

      ReferenceFrame pelvisZUpFrame = controllerToolbox.getPelvisZUpFrame();
      ReferenceFrame chestBodyFrame = chest.getBodyFixedFrame();
      ReferenceFrame headBodyFrame = head.getBodyFixedFrame();

      RigidBodyControlManager chestManager = managerFactory.getOrCreateRigidBodyManager(chest, pelvis, chestBodyFrame, pelvisZUpFrame);
      bodyManagers.add(chestManager);

      RigidBodyControlManager headManager = managerFactory.getOrCreateRigidBodyManager(head, chest, headBodyFrame, chestBodyFrame);
      bodyManagers.add(headManager);

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody hand = fullRobotModel.getHand(robotSide);
         ReferenceFrame handControlFrame = fullRobotModel.getHandControlFrame(robotSide);
         RigidBodyControlManager handManager = managerFactory.getOrCreateRigidBodyManager(hand, chest, handControlFrame, chestBodyFrame);
         bodyManagers.add(handManager);
      }

      this.walkingControllerParameters = walkingControllerParameters;

      balanceManager = managerFactory.getOrCreateBalanceManager();
      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();

      this.commandInputManager = commandInputManager;
      this.statusOutputManager = statusOutputManager;

      allowUpperBodyMotionDuringLocomotion.set(walkingControllerParameters.allowUpperBodyMotionDuringLocomotion());

      hasWalkingControllerBeenInitialized.set(false);

      failureDetectionControlModule = new WalkingFailureDetectionControlModule(controllerToolbox.getContactableFeet(), registry);

      double defaultTransferTime = walkingControllerParameters.getDefaultTransferTime();
      double defaultSwingTime = walkingControllerParameters.getDefaultSwingTime();
      double defaultInitialTransferTime = walkingControllerParameters.getDefaultInitialTransferTime();
      walkingMessageHandler = new WalkingMessageHandler(defaultTransferTime, defaultSwingTime, defaultInitialTransferTime, feet, statusOutputManager, yoTime, yoGraphicsListRegistry, registry);

      commandConsumer = new WalkingCommandConsumer(commandInputManager, statusOutputManager, controllerToolbox, walkingMessageHandler, managerFactory, walkingControllerParameters, registry);

      String namePrefix = "walking";
      stateMachine = new GenericStateMachine<>(namePrefix + "State", namePrefix + "SwitchTime", WalkingStateEnum.class, yoTime, registry); // this is used by name, and it is ugly.

      setupStateMachine();

      double highCoPDampingDuration = walkingControllerParameters.getHighCoPDampingDurationToPreventFootShakies();
      double coPErrorThreshold = walkingControllerParameters.getCoPErrorThresholdForHighCoPDamping();
      boolean enableHighCoPDamping = highCoPDampingDuration > 0.0 && !Double.isInfinite(coPErrorThreshold);
      controllerToolbox.setHighCoPDampingParameters(enableHighCoPDamping, highCoPDampingDuration, coPErrorThreshold);

      JointLimitParameters limitParameters = new JointLimitParameters();
      String[] jointNamesRestrictiveLimits = walkingControllerParameters.getJointsWithRestrictiveLimits(limitParameters);
      OneDoFJoint[] jointsWithRestrictiveLimit = ScrewTools.filterJoints(ScrewTools.findJointsWithNames(allOneDoFjoints, jointNamesRestrictiveLimits), OneDoFJoint.class);
      for (OneDoFJoint joint : jointsWithRestrictiveLimit)
         jointLimitEnforcementMethodCommand.addLimitEnforcementMethod(joint, JointLimitEnforcement.RESTRICTIVE, limitParameters);
   }

   private void setupStateMachine()
   {
      StandingState standingState = new StandingState(commandInputManager, walkingMessageHandler, controllerToolbox, managerFactory, failureDetectionControlModule, walkingControllerParameters, registry);
      TransferToStandingState toStandingState = new TransferToStandingState(walkingMessageHandler, controllerToolbox, managerFactory, failureDetectionControlModule, registry);
      WalkingStateEnum toStandingStateEnum = toStandingState.getStateEnum();

      stateMachine.addState(toStandingState);
      stateMachine.addState(standingState);

      SideDependentList<TransferToWalkingSingleSupportState> walkingTransferStates = new SideDependentList<>();

      for (RobotSide transferToSide : RobotSide.values)
      {
         double minimumTransferTime = walkingControllerParameters.getMinimumTransferTime();
         TransferToWalkingSingleSupportState transferState = new TransferToWalkingSingleSupportState(transferToSide, walkingMessageHandler,
               controllerToolbox, managerFactory, failureDetectionControlModule, minimumTransferTime, registry);
         walkingTransferStates.put(transferToSide, transferState);
         stateMachine.addState(transferState);
      }

      SideDependentList<WalkingSingleSupportState> walkingSingleSupportStates = new SideDependentList<>();

      for (RobotSide supportSide : RobotSide.values)
      {
         WalkingSingleSupportState singleSupportState = new WalkingSingleSupportState(supportSide, walkingMessageHandler, controllerToolbox,
               managerFactory, walkingControllerParameters, failureDetectionControlModule, registry);
         walkingSingleSupportStates.put(supportSide, singleSupportState);
         stateMachine.addState(singleSupportState);
      }

      SideDependentList<TransferToFlamingoStanceState> flamingoTransferStates = new SideDependentList<>();

      for (RobotSide transferToSide : RobotSide.values)
      {
         TransferToFlamingoStanceState transferState = new TransferToFlamingoStanceState(transferToSide, walkingMessageHandler, controllerToolbox, managerFactory, failureDetectionControlModule, registry);
         flamingoTransferStates.put(transferToSide, transferState);
         stateMachine.addState(transferState);
      }

      SideDependentList<FlamingoStanceState> flamingoSingleSupportStates = new SideDependentList<>();

      for (RobotSide supportSide : RobotSide.values)
      {
         FlamingoStanceState singleSupportState = new FlamingoStanceState(supportSide, walkingMessageHandler, controllerToolbox, managerFactory, failureDetectionControlModule, registry);
         flamingoSingleSupportStates.put(supportSide, singleSupportState);
         stateMachine.addState(singleSupportState);
      }

      // Encapsulate toStandingTransition to make sure it is not used afterwards
      {
         toStandingState.addDoneWithStateTransition(WalkingStateEnum.STANDING);
      }

      // Setup start/stop walking conditions
      for (RobotSide robotSide : RobotSide.values)
      {
         TransferToWalkingSingleSupportState transferState = walkingTransferStates.get(robotSide);
         SingleSupportState singleSupportState = walkingSingleSupportStates.get(robotSide);

         WalkingStateEnum transferStateEnum = transferState.getStateEnum();

         // Start walking
         StartWalkingCondition startWalkingCondition = new StartWalkingCondition(transferState.getTransferToSide(), walkingMessageHandler);
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

         transferState.addDoneWithStateTransition(singleSupportStateEnum);
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

         transferState.addDoneWithStateTransition(singleSupportStateEnum);
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
         RobotSide swingSide = walkingSingleSupportState.getSwingSide();

         DoubSuppToSingSuppCond4DistRecov isFallingFromDoubleSupportCondition = new DoubSuppToSingSuppCond4DistRecov(swingSide, balanceManager);
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
            controllerToolbox.reportControllerStateChangeToListeners(oldState.getStateEnum(), newState.getStateEnum());
         }
      });
   }

   @Override
   public void setControllerCoreOutput(ControllerCoreOutputReadOnly controllerCoreOutput)
   {
      this.controllerCoreOutput = controllerCoreOutput;
   }

   public void initialize()
   {
      controllerToolbox.initialize();
      managerFactory.initializeManagers();

      commandInputManager.flushAllCommands();

      privilegedConfigurationCommand.clear();
      privilegedConfigurationCommand.setPrivilegedConfigurationOption(PrivilegedConfigurationOption.AT_ZERO);

      for (RobotSide robotSide : RobotSide.values)
      {
         ArmJointName[] armJointNames = fullRobotModel.getRobotSpecificJointNames().getArmJointNames();
         for (int i = 0; i < armJointNames.length; i++)
            privilegedConfigurationCommand.addJoint(fullRobotModel.getArmJoint(robotSide, armJointNames[i]), PrivilegedConfigurationOption.AT_MID_RANGE);
      }

      initializeContacts();

      for (RobotSide robotSide : RobotSide.values)
      {
         controllerCoreOutput.getDesiredCenterOfPressure(footDesiredCoPs.get(robotSide), feet.get(robotSide).getRigidBody());
         controllerToolbox.setDesiredCenterOfPressure(feet.get(robotSide), footDesiredCoPs.get(robotSide));
      }

      balanceManager.disablePelvisXYControl();

      double stepTime = walkingMessageHandler.getDefaultStepTime();
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

      for (int managerIdx = 0; managerIdx < bodyManagers.size(); managerIdx++)
      {
         RigidBodyControlManager bodyManager = bodyManagers.get(managerIdx);
         if (bodyManager != null)
            bodyManager.initialize();
      }

      balanceManager.initialize();
      feetManager.initialize();
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
      controllerToolbox.clearContacts();

      for (RobotSide robotSide : RobotSide.values)
      {
         feetManager.setFlatFootContactState(robotSide);
      }
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

   private final FrameVector2d desiredICPVelocityAsFrameVector = new FrameVector2d();

   private final SideDependentList<FramePoint2d> footDesiredCoPs = new SideDependentList<FramePoint2d>(new FramePoint2d(), new FramePoint2d());
   private final RecyclingArrayList<PlaneContactStateCommand> planeContactStateCommandPool = new RecyclingArrayList<>(4, PlaneContactStateCommand.class);
   private final FramePoint2d capturePoint2d = new FramePoint2d();
   private final FramePoint2d desiredCapturePoint2d = new FramePoint2d();
   private final FrameVector achievedLinearMomentumRate = new FrameVector();

   @Override
   public void doAction()
   {
      controllerToolbox.update();

      controllerCoreOutput.getLinearMomentumRate(achievedLinearMomentumRate);
      balanceManager.computeAchievedCMP(achievedLinearMomentumRate);

      WalkingState currentState = stateMachine.getCurrentState();

      commandConsumer.consumeHeadCommands();
      commandConsumer.consumeChestCommands();
      commandConsumer.consumePelvisHeightCommands();
      commandConsumer.consumeGoHomeMessages();
      commandConsumer.consumeEndEffectorLoadBearingCommands(currentState);
      commandConsumer.consumeStopAllTrajectoryCommands();
      commandConsumer.consumeFootCommands();
      commandConsumer.consumeAbortWalkingCommands(abortWalkingRequested);
      commandConsumer.consumePelvisCommands(currentState, allowUpperBodyMotionDuringLocomotion.getBooleanValue());
      commandConsumer.consumeManipulationCommands(currentState, allowUpperBodyMotionDuringLocomotion.getBooleanValue());
      commandConsumer.handleAutomaticManipulationAbortOnICPError(currentState);

      updateFailureDetection();

      balanceManager.update();

      stateMachine.checkTransitionConditions();
      stateMachine.doAction();

      currentState = stateMachine.getCurrentState();

      updateManagers(currentState);

      handleChangeInContactState();

      submitControllerCoreCommands();

      for (RobotSide robotSide : RobotSide.values)
      {
         controllerCoreOutput.getDesiredCenterOfPressure(footDesiredCoPs.get(robotSide), feet.get(robotSide).getRigidBody());
         controllerToolbox.setDesiredCenterOfPressure(feet.get(robotSide), footDesiredCoPs.get(robotSide));
      }

      statusOutputManager.reportStatusMessage(balanceManager.updateAndReturnCapturabilityBasedStatus());
   }

   private void handleChangeInContactState()
   {
      boolean haveContactStatesChanged = false;
      for (RobotSide robotSide : RobotSide.values)
      {
         YoPlaneContactState contactState = controllerToolbox.getContactState(feet.get(robotSide));
         if (contactState.pollContactHasChangedNotification())
            haveContactStatesChanged = true;
      }

      if (!haveContactStatesChanged)
         return;

      controllerToolbox.updateBipedSupportPolygons();
      balanceManager.updateCurrentICPPlan();
   }

   public void updateFailureDetection()
   {
      balanceManager.getCapturePoint(capturePoint2d);
      balanceManager.getDesiredICP(desiredCapturePoint2d);
      failureDetectionControlModule.checkIfRobotIsFalling(capturePoint2d, desiredCapturePoint2d);
      if (failureDetectionControlModule.isRobotFalling())
      {
         walkingMessageHandler.clearFootsteps();
         walkingMessageHandler.clearFootTrajectory();
         commandInputManager.flushAllCommands();

         if (enablePushRecoveryOnFailure.getBooleanValue() && !balanceManager.isPushRecoveryEnabled())
         {
            balanceManager.enablePushRecovery();
         }
         else if (!balanceManager.isPushRecoveryEnabled() || balanceManager.isRecoveryImpossible())
         {
            FrameVector2d fallingDirection = failureDetectionControlModule.getFallingDirection();
            walkingMessageHandler.reportControllerFailure(fallingDirection);
            controllerToolbox.reportControllerFailureToListeners(fallingDirection);
         }
      }

      if (enablePushRecoveryOnFailure.getBooleanValue())
      {
         if (balanceManager.isPushRecoveryEnabled() && balanceManager.isRobotBackToSafeState())
            balanceManager.disablePushRecovery();
      }
   }

   public void updateManagers(WalkingState currentState)
   {
      balanceManager.getDesiredICPVelocity(desiredICPVelocityAsFrameVector);
      boolean isInDoubleSupport = currentState.isDoubleSupportState();
      double omega0 = controllerToolbox.getOmega0();
      boolean isRecoveringFromPush = balanceManager.isRecovering();
      controlledCoMHeightAcceleration.set(comHeightManager.computeDesiredCoMHeightAcceleration(desiredICPVelocityAsFrameVector, isInDoubleSupport, omega0,
            isRecoveringFromPush, feetManager));

      feetManager.compute();

      boolean bodyManagerIsLoadBearing = false;
      for (int managerIdx = 0; managerIdx < bodyManagers.size(); managerIdx++)
      {
         RigidBodyControlManager bodyManager = bodyManagers.get(managerIdx);
         if (bodyManager != null)
         {
            bodyManager.compute();

            if (bodyManager.isLoadBearing())
               bodyManagerIsLoadBearing = true;
         }
      }

      if (pelvisOrientationManager != null)
         pelvisOrientationManager.compute();

      boolean keepCMPInsideSupportPolygon = !bodyManagerIsLoadBearing;
      balanceManager.compute(currentState.getSupportSide(), controlledCoMHeightAcceleration.getDoubleValue(), keepCMPInsideSupportPolygon);
   }

   private void submitControllerCoreCommands()
   {
      planeContactStateCommandPool.clear();

      controllerCoreCommand.addInverseDynamicsCommand(privilegedConfigurationCommand);
      if (!limitCommandSent.getBooleanValue())
      {
         controllerCoreCommand.addInverseDynamicsCommand(jointLimitEnforcementMethodCommand);
         limitCommandSent.set(true);
      }

      boolean isHighCoPDampingNeeded = controllerToolbox.estimateIfHighCoPDampingNeeded(footDesiredCoPs);

      for (RobotSide robotSide : RobotSide.values)
      {
         controllerCoreCommand.addFeedbackControlCommand(feetManager.getFeedbackControlCommand(robotSide));
         controllerCoreCommand.addInverseDynamicsCommand(feetManager.getInverseDynamicsCommand(robotSide));

         YoPlaneContactState contactState = controllerToolbox.getContactState(feet.get(robotSide));
         PlaneContactStateCommand planeContactStateCommand = planeContactStateCommandPool.add();
         contactState.getPlaneContactStateCommand(planeContactStateCommand);
         planeContactStateCommand.setUseHighCoPDamping(isHighCoPDampingNeeded);
         controllerCoreCommand.addInverseDynamicsCommand(planeContactStateCommand);
      }

      for (int managerIdx = 0; managerIdx < bodyManagers.size(); managerIdx++)
      {
         RigidBodyControlManager bodyManager = bodyManagers.get(managerIdx);
         if (bodyManager != null)
         {
            controllerCoreCommand.addFeedbackControlCommand(bodyManager.getFeedbackControlCommand());
            controllerCoreCommand.addInverseDynamicsCommand(bodyManager.getInverseDynamicsCommand());
            controllerCoreCommand.completeLowLevelJointData(bodyManager.getLowLevelJointDesiredData());
         }
      }

      controllerCoreCommand.addFeedbackControlCommand(pelvisOrientationManager.getFeedbackControlCommand());

      controllerCoreCommand.addInverseDynamicsCommand(balanceManager.getInverseDynamicsCommand());
   }

   public void reinitializePelvisOrientation(boolean reinitialize)
   {
      hasWalkingControllerBeenInitialized.set(!reinitialize);
   }

   @Override
   public void doTransitionIntoAction()
   {
      for (int i = 0; i < allOneDoFjoints.length; i++)
      {
         allOneDoFjoints[i].resetDesiredAccelerationIntegrator();
         allOneDoFjoints[i].setQddDesired(0.0);
         allOneDoFjoints[i].setTau(0.0);
      }

      initialize();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      for (int i = 0; i < allOneDoFjoints.length; i++)
      {
         allOneDoFjoints[i].resetDesiredAccelerationIntegrator();
         allOneDoFjoints[i].setQddDesired(0.0);
         allOneDoFjoints[i].setTau(0.0);
      }
   }

   @Override
   public ControllerCoreCommand getControllerCoreCommand()
   {
      return controllerCoreCommand;
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }
}
