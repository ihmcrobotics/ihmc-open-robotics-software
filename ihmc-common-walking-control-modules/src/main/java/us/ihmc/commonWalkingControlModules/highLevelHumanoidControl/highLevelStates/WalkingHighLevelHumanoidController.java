package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationManager;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitEnforcementMethodCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
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
import us.ihmc.commonWalkingControlModules.capturePoint.BalanceManager;
import us.ihmc.commonWalkingControlModules.capturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitEnforcement;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.partNames.ArmJointName;
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
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class WalkingHighLevelHumanoidController
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final YoDouble yoTime;

   private final HighLevelControlManagerFactory managerFactory;

   private final PelvisOrientationManager pelvisOrientationManager;
   private final FeetManager feetManager;
   private final LegConfigurationManager legConfigurationManager;
   private final BalanceManager balanceManager;
   private final CenterOfMassHeightManager comHeightManager;

   private final ArrayList<RigidBodyControlManager> bodyManagers = new ArrayList<>();
   private final Map<String, RigidBodyControlManager> bodyManagerByJointName = new HashMap<>();
   private final SideDependentList<Set<String>> legJointNames = new SideDependentList<>();

   private final OneDoFJoint[] allOneDoFjoints;

   private final FullHumanoidRobotModel fullRobotModel;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WalkingControllerParameters walkingControllerParameters;

   private final SideDependentList<? extends ContactablePlaneBody> feet;

   private final GenericStateMachine<WalkingStateEnum, WalkingState> stateMachine;

   private final WalkingMessageHandler walkingMessageHandler;
   private final YoBoolean abortWalkingRequested = new YoBoolean("requestAbortWalking", registry);

   private final YoDouble controlledCoMHeightAcceleration = new YoDouble("controlledCoMHeightAcceleration", registry);

   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final YoBoolean hasICPPlannerBeenInitializedAtStart = new YoBoolean("hasICPPlannerBeenInitializedAtStart", registry);

   private final YoBoolean enablePushRecoveryOnFailure = new YoBoolean("enablePushRecoveryOnFailure", registry);

   private final YoBoolean allowUpperBodyMotionDuringLocomotion = new YoBoolean("allowUpperBodyMotionDuringLocomotion", registry);

   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusOutputManager;
   private final WalkingCommandConsumer commandConsumer;

   private final JointLimitEnforcementMethodCommand jointLimitEnforcementMethodCommand = new JointLimitEnforcementMethodCommand();
   private final YoBoolean limitCommandSent = new YoBoolean("limitCommandSent", registry);

   private final PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);
   private ControllerCoreOutputReadOnly controllerCoreOutput;

   public WalkingHighLevelHumanoidController(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
                                             HighLevelControlManagerFactory managerFactory, WalkingControllerParameters walkingControllerParameters,
                                             HighLevelHumanoidControllerToolbox controllerToolbox)
   {
      this.managerFactory = managerFactory;

      // Getting parameters from the HighLevelHumanoidControllerToolbox
      this.controllerToolbox = controllerToolbox;
      fullRobotModel = controllerToolbox.getFullRobotModel();
      yoTime = controllerToolbox.getYoTime();

      feet = controllerToolbox.getContactableFeet();

      allOneDoFjoints = fullRobotModel.getOneDoFJoints();

      this.pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      this.feetManager = managerFactory.getOrCreateFeetManager();
      this.legConfigurationManager = managerFactory.getOrCreateLegConfigurationManager();

      RigidBody head = fullRobotModel.getHead();
      RigidBody chest = fullRobotModel.getChest();
      RigidBody pelvis = fullRobotModel.getPelvis();

      Collection<ReferenceFrame> trajectoryFrames = controllerToolbox.getTrajectoryFrames();
      ReferenceFrame pelvisZUpFrame = controllerToolbox.getPelvisZUpFrame();

      ReferenceFrame chestBodyFrame = null;
      if(chest != null)
      {
         chestBodyFrame = chest.getBodyFixedFrame();
         RigidBodyControlManager chestManager = managerFactory.getOrCreateRigidBodyManager(chest, pelvis, chestBodyFrame, pelvisZUpFrame,trajectoryFrames);
         bodyManagers.add(chestManager);
      }

      if (head != null)
      {
         ReferenceFrame headBodyFrame = head.getBodyFixedFrame();
         RigidBodyControlManager headManager = managerFactory.getOrCreateRigidBodyManager(head, chest, headBodyFrame, chestBodyFrame, trajectoryFrames);
         bodyManagers.add(headManager);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody hand = fullRobotModel.getHand(robotSide);
         ReferenceFrame handControlFrame = fullRobotModel.getHandControlFrame(robotSide);
         RigidBodyControlManager handManager = managerFactory.getOrCreateRigidBodyManager(hand, chest, handControlFrame, chestBodyFrame, trajectoryFrames);
         bodyManagers.add(handManager);
      }

      for (RigidBodyControlManager manager : bodyManagers)
      {
         if (manager == null)
         {
            continue;
         }
         Arrays.asList(manager.getControlledJoints()).stream().forEach(joint -> bodyManagerByJointName.put(joint.getName(), manager));
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         OneDoFJoint[] legJoints = ScrewTools.filterJoints(ScrewTools.createJointPath(pelvis, foot), OneDoFJoint.class);
         Set<String> jointNames = new HashSet<>();
         Arrays.asList(legJoints).stream().forEach(legJoint -> jointNames.add(legJoint.getName()));
         legJointNames.put(robotSide, jointNames);
      }

      this.walkingControllerParameters = walkingControllerParameters;

      balanceManager = managerFactory.getOrCreateBalanceManager();
      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();

      this.commandInputManager = commandInputManager;
      this.statusOutputManager = statusOutputManager;

      allowUpperBodyMotionDuringLocomotion.set(walkingControllerParameters.allowUpperBodyMotionDuringLocomotion());

      failureDetectionControlModule = new WalkingFailureDetectionControlModule(controllerToolbox.getContactableFeet(), registry);

      walkingMessageHandler = controllerToolbox.getWalkingMessageHandler();
      commandConsumer = new WalkingCommandConsumer(commandInputManager, statusOutputManager, controllerToolbox, managerFactory, walkingControllerParameters, registry);

      String namePrefix = "walking";
      stateMachine = new GenericStateMachine<>(namePrefix + "State", namePrefix + "SwitchTime", WalkingStateEnum.class, yoTime, registry); // this is used by name, and it is ugly.

      setupStateMachine();

      double highCoPDampingDuration = walkingControllerParameters.getHighCoPDampingDurationToPreventFootShakies();
      double coPErrorThreshold = walkingControllerParameters.getCoPErrorThresholdForHighCoPDamping();
      boolean enableHighCoPDamping = highCoPDampingDuration > 0.0 && !Double.isInfinite(coPErrorThreshold);
      controllerToolbox.setHighCoPDampingParameters(enableHighCoPDamping, highCoPDampingDuration, coPErrorThreshold);

      String[] jointNamesRestrictiveLimits = walkingControllerParameters.getJointsWithRestrictiveLimits();
      JointLimitParameters limitParameters = walkingControllerParameters.getJointLimitParametersForJointsWithRestictiveLimits();
      OneDoFJoint[] jointsWithRestrictiveLimit = ScrewTools.filterJoints(ScrewTools.findJointsWithNames(allOneDoFjoints, jointNamesRestrictiveLimits), OneDoFJoint.class);
      for (OneDoFJoint joint : jointsWithRestrictiveLimit)
      {
         if (limitParameters == null)
         {
            throw new RuntimeException("Must define joint limit parameters if using joints with restrictive limits.");
         }
         jointLimitEnforcementMethodCommand.addLimitEnforcementMethod(joint, JointLimitEnforcement.RESTRICTIVE, limitParameters);
      }
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
               controllerToolbox, managerFactory, walkingControllerParameters, failureDetectionControlModule, minimumTransferTime, registry);
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
         TransferToFlamingoStanceState transferState = new TransferToFlamingoStanceState(transferToSide, walkingControllerParameters, walkingMessageHandler, controllerToolbox, managerFactory, failureDetectionControlModule, registry);
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

   public void setControllerCoreOutput(ControllerCoreOutputReadOnly controllerCoreOutput)
   {
      this.controllerCoreOutput = controllerCoreOutput;
   }

   public void initialize()
   {
      controllerCoreCommand.requestReinitialization();
      controllerToolbox.initialize();
      managerFactory.initializeManagers();

      commandInputManager.flushAllCommands();
      walkingMessageHandler.clearFootsteps();
      walkingMessageHandler.clearFootTrajectory();

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

      for (int managerIdx = 0; managerIdx < bodyManagers.size(); managerIdx++)
      {
         RigidBodyControlManager bodyManager = bodyManagers.get(managerIdx);
         if (bodyManager != null)
            bodyManager.initialize();
      }

      pelvisOrientationManager.initialize();
      balanceManager.initialize();
      feetManager.initialize();
      //      requestICPPlannerToHoldCurrent(); // Not sure if we want to do this. Might cause robot to fall. Might just be better to recenter ICP whenever switching to walking.

      // Need to reset it so the planner will be initialized even when restarting the walking controller.
      hasICPPlannerBeenInitializedAtStart.set(false);
      stateMachine.setCurrentState(WalkingStateEnum.TO_STANDING);

      commandConsumer.avoidManipulationAbortForDuration(RigidBodyControlManager.INITIAL_GO_HOME_TIME);
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

   private final FrameVector2D desiredICPVelocityAsFrameVector = new FrameVector2D();

   private final SideDependentList<FramePoint2D> footDesiredCoPs = new SideDependentList<FramePoint2D>(new FramePoint2D(), new FramePoint2D());
   private final RecyclingArrayList<PlaneContactStateCommand> planeContactStateCommandPool = new RecyclingArrayList<>(4, PlaneContactStateCommand.class);
   private final FramePoint2D capturePoint2d = new FramePoint2D();
   private final FramePoint2D desiredCapturePoint2d = new FramePoint2D();
   private final FrameVector3D achievedLinearMomentumRate = new FrameVector3D();

   public void doAction()
   {
      controllerToolbox.update();

      controllerCoreOutput.getLinearMomentumRate(achievedLinearMomentumRate);
      balanceManager.computeAchievedCMP(achievedLinearMomentumRate);

      WalkingState currentState = stateMachine.getCurrentState();
      commandConsumer.update();
      commandConsumer.consumeHeadCommands();
      commandConsumer.consumeChestCommands();
      commandConsumer.consumePelvisHeightCommands();
      commandConsumer.consumeGoHomeMessages();
      commandConsumer.consumeFootLoadBearingCommands(currentState);
      commandConsumer.consumeStopAllTrajectoryCommands();
      commandConsumer.consumeFootCommands();
      commandConsumer.consumeAbortWalkingCommands(abortWalkingRequested);
      commandConsumer.consumePelvisCommands(currentState, allowUpperBodyMotionDuringLocomotion.getBooleanValue());
      commandConsumer.consumeManipulationCommands(currentState, allowUpperBodyMotionDuringLocomotion.getBooleanValue());
      commandConsumer.handleAutomaticManipulationAbortOnICPError(currentState);
      commandConsumer.consumeLoadBearingCommands();
      commandConsumer.consumePlanarRegionsListCommand();

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
         YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotSide);
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
            FrameVector2D fallingDirection = failureDetectionControlModule.getFallingDirection();
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

      feetManager.compute();
      legConfigurationManager.compute();

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

      comHeightManager.compute();
      controlledCoMHeightAcceleration.set(comHeightManager.computeDesiredCoMHeightAcceleration(desiredICPVelocityAsFrameVector, isInDoubleSupport, omega0,
            isRecoveringFromPush, feetManager));

      // the comHeightManager can control the pelvis with a feedback controller and doesn't always need the z component of the momentum command. It would be better to remove the coupling between these two modules
      boolean controlHeightWithMomentum = comHeightManager.getControlHeightWithMomentum();
      boolean keepCMPInsideSupportPolygon = !bodyManagerIsLoadBearing;
      if (currentState.isDoubleSupportState())
         balanceManager.compute(currentState.getTransferToSide(), controlledCoMHeightAcceleration.getDoubleValue(), keepCMPInsideSupportPolygon, controlHeightWithMomentum);
      else
         balanceManager.compute(currentState.getSupportSide(), controlledCoMHeightAcceleration.getDoubleValue(), keepCMPInsideSupportPolygon, controlHeightWithMomentum);
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

         controllerCoreCommand.addFeedbackControlCommand(legConfigurationManager.getFeedbackControlCommand(robotSide));
         controllerCoreCommand.addInverseDynamicsCommand(legConfigurationManager.getInverseDynamicsCommand(robotSide));

         YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotSide);
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
         }
      }

      controllerCoreCommand.addFeedbackControlCommand(pelvisOrientationManager.getFeedbackControlCommand());
      controllerCoreCommand.addFeedbackControlCommand(comHeightManager.getFeedbackControlCommand());

      controllerCoreCommand.addInverseDynamicsCommand(balanceManager.getInverseDynamicsCommand());

      /*
       * FIXME: This is mainly used for resetting the integrators at touchdown.
       * It is done in SingleSupportState.doTransitionOutOfAction. Need to
       * figure out how to use directly the joint data holder instead of
       * OneDoFJoint.
       */
      LowLevelOneDoFJointDesiredDataHolder jointDesiredDataHolder = controllerCoreCommand.getLowLevelOneDoFJointDesiredDataHolder();

      for (int i = 0; i < allOneDoFjoints.length; i++)
      {
         OneDoFJoint joint = allOneDoFjoints[i];
         if (joint.getResetDesiredAccelerationIntegrator())
         {
            JointDesiredOutput jointData = jointDesiredDataHolder.getJointDesiredOutput(joint);
            if (jointData == null)
               jointData = jointDesiredDataHolder.registerJointWithEmptyData(joint);
            jointData.setResetIntegrators(true);
         }
      }
   }

   public void reinitializePelvisOrientation(boolean reinitialize)
   {
      pelvisOrientationManager.initialize();
   }

   public void resetJointIntegrators()
   {
      for (int i = 0; i < allOneDoFjoints.length; i++)
      {
         allOneDoFjoints[i].resetDesiredAccelerationIntegrator();
         allOneDoFjoints[i].setQddDesired(0.0);
         allOneDoFjoints[i].setTau(0.0);
      }
   }

   public ControllerCoreCommand getControllerCoreCommand()
   {
      return controllerCoreCommand;
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   /**
    * Returns the currently active walking state. This is used for unit testing.
    * @return WalkingStateEnum
    */
   public WalkingStateEnum getWalkingStateEnum()
   {
      return stateMachine.getCurrentStateEnum();
   }

   public boolean isJointLoaded(String jointName)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         boolean legLoaded = feetManager.getCurrentConstraintType(robotSide).isLoaded();
         if (legLoaded && legJointNames.get(robotSide).contains(jointName))
         {
            return true;
         }
      }

      RigidBodyControlManager manager = bodyManagerByJointName.get(jointName);
      if (manager != null && manager.isLoadBearing())
      {
         return true;
      }

      return false;
   }
}
