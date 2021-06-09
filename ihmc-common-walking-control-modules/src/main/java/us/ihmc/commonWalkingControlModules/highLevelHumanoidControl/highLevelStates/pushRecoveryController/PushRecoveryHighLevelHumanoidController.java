package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController;

import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.capturePoint.BalanceManager;
import us.ihmc.commonWalkingControlModules.capturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.capturePoint.LinearMomentumRateControlModuleInput;
import us.ihmc.commonWalkingControlModules.capturePoint.LinearMomentumRateControlModuleOutput;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationManager;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointLimitEnforcementMethodCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.JointLoadStatusProvider;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.ParameterizedControllerCoreOptimizationSettings;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.stateTransitionConditions.PushRecoverySingleSupportToTransferToCondition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.stateTransitionConditions.StartPushRecoveryCondition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.stateTransitionConditions.StopPushRecoveryFromSingleSupportCondition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.states.*;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.TouchdownErrorCompensator;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitEnforcement;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.*;

public class PushRecoveryHighLevelHumanoidController implements JointLoadStatusProvider
{
   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);

   private final YoDouble yoTime;

   private final HighLevelControlManagerFactory managerFactory;

   private final PelvisOrientationManager pelvisOrientationManager;
   private final FeetManager feetManager;
   private final LegConfigurationManager legConfigurationManager;
   private final BalanceManager balanceManager;    //TODO replace with PushRecoverManager?
   private final CenterOfMassHeightManager comHeightManager;

   private final TouchdownErrorCompensator touchdownErrorCompensator;

   private final ArrayList<RigidBodyControlManager> bodyManagers = new ArrayList<>();
   private final Map<String, RigidBodyControlManager> bodyManagerByJointName = new HashMap<>();
   private final SideDependentList<Set<String>> legJointNames = new SideDependentList<>();

   private final OneDoFJointBasics[] allOneDoFjoints;

   private final FullHumanoidRobotModel fullRobotModel;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final PushRecoveryControllerParameters pushRecoveryControllerParameters;   // TODO

   private final SideDependentList<? extends ContactablePlaneBody> feet;

   private final StateMachine<PushRecoveryStateEnum, PushRecoveryState> stateMachine;

   private final WalkingMessageHandler walkingMessageHandler;
   private final YoBoolean abortWalkingRequested = new YoBoolean("requestAbortWalking", registry);

   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final YoBoolean enablePushRecoveryOnFailure = new YoBoolean("enablePushRecoveryOnFailure", registry);

   private final YoBoolean allowUpperBodyMotionDuringLocomotion = new YoBoolean("allowUpperBodyMotionDuringLocomotion", registry);

   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusOutputManager;

   private final TaskspaceTrajectoryStatusMessage pelvisStatusMessage = new TaskspaceTrajectoryStatusMessage();

   private final JointLimitEnforcementMethodCommand jointLimitEnforcementMethodCommand = new JointLimitEnforcementMethodCommand();
   private final YoBoolean limitCommandSent = new YoBoolean("limitCommandSent", registry);

   private final PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);
   private ControllerCoreOutputReadOnly controllerCoreOutput;

   private final DoubleProvider unloadFraction;

   private final ParameterizedControllerCoreOptimizationSettings controllerCoreOptimizationSettings;

   private final YoBoolean enableHeightFeedbackControl = new YoBoolean("enableHeightFeedbackControl", registry);

   private boolean firstTick = true;

   public PushRecoveryHighLevelHumanoidController(CommandInputManager commandInputManager,
                                                  StatusMessageOutputManager statusOutputManager,
                                                  HighLevelControlManagerFactory managerFactory,
                                                  PushRecoveryControllerParameters pushRecoveryControllerParameters,
                                                  HighLevelHumanoidControllerToolbox controllerToolbox)
   {
      this.managerFactory = managerFactory;

      // Getting parameters from the HighLevelHumanoidControllerToolbox
      this.controllerToolbox = controllerToolbox;
      fullRobotModel = controllerToolbox.getFullRobotModel();
      yoTime = controllerToolbox.getYoTime();

      feet = controllerToolbox.getContactableFeet();

      allOneDoFjoints = MultiBodySystemTools.filterJoints(controllerToolbox.getControlledJoints(), OneDoFJointBasics.class);

      this.pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      this.feetManager = managerFactory.getOrCreateFeetManager();
      this.legConfigurationManager = managerFactory.getOrCreateLegConfigurationManager();

      RigidBodyBasics head = fullRobotModel.getHead();
      RigidBodyBasics chest = fullRobotModel.getChest();
      RigidBodyBasics pelvis = fullRobotModel.getPelvis();

      pelvisStatusMessage.setEndEffectorName(pelvis.getName());

      unloadFraction = pushRecoveryControllerParameters.enforceSmoothFootUnloading() ? new DoubleParameter("unloadFraction", registry, 0.5) : null;

      ReferenceFrame pelvisZUpFrame = controllerToolbox.getPelvisZUpFrame();

      ReferenceFrame chestBodyFrame = null;
      if (chest != null)
      {
         chestBodyFrame = chest.getBodyFixedFrame();
         RigidBodyControlManager chestManager = managerFactory.getOrCreateRigidBodyManager(chest, pelvis, chestBodyFrame, pelvisZUpFrame);
         bodyManagers.add(chestManager);
      }

      if (head != null)
      {
         ReferenceFrame headBodyFrame = head.getBodyFixedFrame();
         RigidBodyControlManager headManager = managerFactory.getOrCreateRigidBodyManager(head, chest, headBodyFrame, chestBodyFrame);
         bodyManagers.add(headManager);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         ReferenceFrame handControlFrame = fullRobotModel.getHandControlFrame(robotSide);
         RigidBodyControlManager handManager = managerFactory.getOrCreateRigidBodyManager(hand, chest, handControlFrame, chestBodyFrame);
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
         RigidBodyBasics foot = fullRobotModel.getFoot(robotSide);
         OneDoFJointBasics[] legJoints = MultiBodySystemTools.filterJoints(MultiBodySystemTools.createJointPath(pelvis, foot), OneDoFJointBasics.class);
         Set<String> jointNames = new HashSet<>();
         Arrays.asList(legJoints).stream().forEach(legJoint -> jointNames.add(legJoint.getName()));
         legJointNames.put(robotSide, jointNames);
      }

      this.pushRecoveryControllerParameters = pushRecoveryControllerParameters;

      balanceManager = managerFactory.getOrCreateBalanceManager();
      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();

      this.commandInputManager = commandInputManager;
      this.statusOutputManager = statusOutputManager;

      enableHeightFeedbackControl.set(pushRecoveryControllerParameters.enableHeightFeedbackControl());

      failureDetectionControlModule = controllerToolbox.getFailureDetectionControlModule();

      walkingMessageHandler = controllerToolbox.getWalkingMessageHandler();

      touchdownErrorCompensator = new TouchdownErrorCompensator(walkingMessageHandler, controllerToolbox.getReferenceFrames().getSoleFrames(), registry);
      stateMachine = setupStateMachine();

      String[] jointNamesRestrictiveLimits = pushRecoveryControllerParameters.getJointsWithRestrictiveLimits();
      OneDoFJointBasics[] jointsWithRestrictiveLimit = MultiBodySystemTools.filterJoints(ScrewTools.findJointsWithNames(allOneDoFjoints,
                                                                                                                        jointNamesRestrictiveLimits),
                                                                                         OneDoFJointBasics.class);
      for (OneDoFJointBasics joint : jointsWithRestrictiveLimit)
      {
         JointLimitParameters limitParameters = pushRecoveryControllerParameters.getJointLimitParametersForJointsWithRestrictiveLimits(joint.getName());
         if (limitParameters == null)
            throw new RuntimeException("Must define joint limit parameters for joint " + joint.getName() + " if using joints with restrictive limits.");
         jointLimitEnforcementMethodCommand.addLimitEnforcementMethod(joint, JointLimitEnforcement.RESTRICTIVE, limitParameters);
      }

      ControllerCoreOptimizationSettings defaultControllerCoreOptimizationSettings = pushRecoveryControllerParameters.getMomentumOptimizationSettings();
      controllerCoreOptimizationSettings = new ParameterizedControllerCoreOptimizationSettings(defaultControllerCoreOptimizationSettings, registry);
   }

   private StateMachine<PushRecoveryStateEnum, PushRecoveryState> setupStateMachine()
   {
      StateMachineFactory<PushRecoveryStateEnum, PushRecoveryState> factory = new StateMachineFactory<>(PushRecoveryStateEnum.class);
      factory.setNamePrefix("pushRecovery").setRegistry(registry).buildYoClock(yoTime);

      TransferToStandingPushRecoveryState toStandingState = new TransferToStandingPushRecoveryState(walkingMessageHandler, touchdownErrorCompensator, controllerToolbox, managerFactory,
                                                                            failureDetectionControlModule, registry);
      factory.addState(PushRecoveryStateEnum.TO_STANDING, toStandingState);

      DoubleProvider minimumTransferTime = new DoubleParameter("MinimumTransferTime", registry, pushRecoveryControllerParameters.getMinimumTransferTime());
      DoubleProvider minimumSwingTime = new DoubleParameter("MinimumSwingTime", registry, pushRecoveryControllerParameters.getMinimumSwingTime());
      DoubleProvider rhoMin = () -> controllerCoreOptimizationSettings.getRhoMin();

      SideDependentList<TransferToRecoveringSingleSupportState> recoveryTransferStates = new SideDependentList<>();
      for (RobotSide transferToSide : RobotSide.values)
      {
         PushRecoveryStateEnum stateEnum = PushRecoveryStateEnum.getPushRecoveryTransferState(transferToSide);
         TransferToRecoveringSingleSupportState transferState = new TransferToRecoveringSingleSupportState(stateEnum, walkingMessageHandler, touchdownErrorCompensator,
                 controllerToolbox, managerFactory, pushRecoveryControllerParameters,
                 failureDetectionControlModule, minimumTransferTime, minimumSwingTime,
                 unloadFraction, rhoMin, registry);
         recoveryTransferStates.put(transferToSide, transferState);
         factory.addState(stateEnum, transferState);
      }

      SideDependentList<RecoveringSingleSupportState> recoveringSingleSupportStates = new SideDependentList<>();
      for (RobotSide supportSide : RobotSide.values)
      {
         PushRecoveryStateEnum stateEnum = PushRecoveryStateEnum.getPushRecoverySingleSupportState(supportSide);
         RecoveringSingleSupportState singleSupportState = new RecoveringSingleSupportState(stateEnum, walkingMessageHandler, touchdownErrorCompensator,
                 controllerToolbox, managerFactory, pushRecoveryControllerParameters,
                 failureDetectionControlModule, registry);
         recoveringSingleSupportStates.put(supportSide, singleSupportState);
         factory.addState(stateEnum, singleSupportState);
      }

      // Setup start/stop push recovery conditions
      for (RobotSide robotSide : RobotSide.values)
      {
         TransferToRecoveringSingleSupportState transferState = recoveryTransferStates.get(robotSide);
         SingleSupportPushRecoveryState singleSupportState = recoveringSingleSupportStates.get(robotSide);

         PushRecoveryStateEnum transferStateEnum = transferState.getStateEnum();
         PushRecoveryStateEnum singleSupportStateEnum = singleSupportState.getStateEnum();

         // Start push recovery
         factory.addTransition(PushRecoveryStateEnum.TO_STANDING, transferStateEnum,
                 new StartPushRecoveryCondition(transferStateEnum.getTransferToSide(), walkingMessageHandler));

         // Stop push recovery when in single support
         factory.addTransition(singleSupportStateEnum, PushRecoveryStateEnum.TO_STANDING,
                 new StopPushRecoveryFromSingleSupportCondition(singleSupportState, walkingMessageHandler));
      }

      // Setup push recovery transfer to single support conditions
      for (RobotSide robotSide : RobotSide.values)
      {
         PushRecoveryState transferState = recoveryTransferStates.get(robotSide);
         PushRecoveryState singleSupportState = recoveringSingleSupportStates.get(robotSide);

         PushRecoveryStateEnum transferStateEnum = transferState.getStateEnum();
         PushRecoveryStateEnum singleSupportStateEnum = singleSupportState.getStateEnum();

         factory.addDoneTransition(transferStateEnum, singleSupportStateEnum);
      }

      // Setup push recovery single support to transfer conditions
      for (RobotSide robotSide : RobotSide.values)
      {
         SingleSupportPushRecoveryState singleSupportState = recoveringSingleSupportStates.get(robotSide);
         PushRecoveryStateEnum singleSupportStateEnum = singleSupportState.getStateEnum();

         // Single support to transfer with same side
         {
            TransferToRecoveringSingleSupportState transferState = recoveryTransferStates.get(robotSide);
            PushRecoveryStateEnum transferStateEnum = transferState.getStateEnum();

            factory.addTransition(singleSupportStateEnum, transferStateEnum,
                    new PushRecoverySingleSupportToTransferToCondition(singleSupportState, transferState, walkingMessageHandler));
         }

         // Single support to transfer with opposite side
         {
            TransferToRecoveringSingleSupportState transferState = recoveryTransferStates.get(robotSide.getOppositeSide());
            PushRecoveryStateEnum transferStateEnum = transferState.getStateEnum();

            factory.addTransition(singleSupportStateEnum, transferStateEnum,
                    new PushRecoverySingleSupportToTransferToCondition(singleSupportState, transferState, walkingMessageHandler));
         }
      }

      // Setup the abort condition from all states to the toStandingState
      Set<PushRecoveryStateEnum> allButStandingStates = EnumSet.complementOf(EnumSet.of(PushRecoveryStateEnum.TO_STANDING));
      factory.addTransition(allButStandingStates, PushRecoveryStateEnum.TO_STANDING, new AbortCondition());

      // Update the previous state info for each state using state changed listeners.
      factory.getRegisteredStates().forEach(state -> factory.addStateChangedListener((from, to) -> state.setPreviousWalkingStateEnum(from)));
      factory.addStateChangedListener((from, to) -> controllerToolbox.reportControllerStateChangeToListeners(from, to));

      return factory.build(PushRecoveryStateEnum.TO_STANDING);
   }

   public void setControllerCoreOutput(ControllerCoreOutputReadOnly controllerCoreOutput)
   {
      this.controllerCoreOutput = controllerCoreOutput;
   }

   public void setLinearMomentumRateControlModuleOutput(LinearMomentumRateControlModuleOutput output)
   {
      balanceManager.setLinearMomentumRateControlModuleOutput(output);
   }

   public void initialize()
   {

      commandInputManager.clearAllCommands();
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

      for (RobotSide robotSide : RobotSide.values)
      {
         footDesiredCoPs.get(robotSide).setToZero(feet.get(robotSide).getSoleFrame());
         controllerToolbox.setDesiredCenterOfPressure(feet.get(robotSide), footDesiredCoPs.get(robotSide));
      }

//      PushRecoveryStateEnum nextState = getInitialPushRecoveryState();
//      stateMachine.performTransition(nextState);
      stateMachine.resetToInitialState();

      firstTick = true;
   }

   private PushRecoveryStateEnum getInitialPushRecoveryState()
   {
      if(feetManager.getCurrentConstraintType(RobotSide.LEFT) == FootControlModule.ConstraintType.FULL ||
              feetManager.getCurrentConstraintType(RobotSide.RIGHT) == FootControlModule.ConstraintType.SWING)
      {
         return PushRecoveryStateEnum.TO_PUSH_RECOVERY_LEFT_SUPPORT;
      }
      else if(feetManager.getCurrentConstraintType(RobotSide.RIGHT) == FootControlModule.ConstraintType.FULL ||
         feetManager.getCurrentConstraintType(RobotSide.LEFT) == FootControlModule.ConstraintType.SWING)
      {
         return PushRecoveryStateEnum.TO_PUSH_RECOVERY_RIGHT_SUPPORT;
      }
      return PushRecoveryStateEnum.TO_STANDING;
   }

   private class AbortCondition implements StateTransitionCondition
   {
      @Override
      public boolean testCondition(double timeInState)
      {
         if (!abortWalkingRequested.getBooleanValue())
            return false;

         walkingMessageHandler.clearFootsteps();
         walkingMessageHandler.reportWalkingAbortRequested();
         abortWalkingRequested.set(false);
         return true;
      }
   }

   private final FrameVector2D desiredCoMVelocityAsFrameVector = new FrameVector2D();

   private final SideDependentList<FramePoint2D> footDesiredCoPs = new SideDependentList<FramePoint2D>(new FramePoint2D(), new FramePoint2D());
   private final RecyclingArrayList<PlaneContactStateCommand> planeContactStateCommandPool = new RecyclingArrayList<>(4, PlaneContactStateCommand.class);
   private final FramePoint2D capturePoint2d = new FramePoint2D();

   public void doAction()
   {
      PushRecoveryState currentState;

//      updateFailureDetection();

      // Do transitions will request ICP planner updates.
      if (!firstTick) // this avoids doing two transitions in a single tick if the initialize reset the state machine.
         stateMachine.doTransitions();
      // Do action is relying on the ICP plan being valid.
      stateMachine.doAction();

      currentState = stateMachine.getCurrentState();

      updateManagers(currentState);
      reportStatusMessages();

      handleChangeInContactState();

      submitControllerCoreCommands();

      for (RobotSide robotSide : RobotSide.values)
      {
         controllerCoreOutput.getDesiredCenterOfPressure(footDesiredCoPs.get(robotSide), feet.get(robotSide).getRigidBody());
         // This happens on the first tick when the controller core has not yet run to update the center of pressure.
         if (footDesiredCoPs.get(robotSide).getReferenceFrame() != feet.get(robotSide).getSoleFrame())
         {
            footDesiredCoPs.get(robotSide).setToZero(feet.get(robotSide).getSoleFrame());
         }
         controllerToolbox.setDesiredCenterOfPressure(feet.get(robotSide), footDesiredCoPs.get(robotSide));
         controllerToolbox.getFootContactState(robotSide).pollContactHasChangedNotification();
      }

      statusOutputManager.reportStatusMessage(balanceManager.updateAndReturnCapturabilityBasedStatus());

      firstTick = false;
   }

   private void handleChangeInContactState()
   {
      boolean haveContactStatesChanged = false;
      for (RobotSide robotSide : RobotSide.values)
      {
         YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotSide);
         if (contactState.peekContactHasChangedNotification())
            haveContactStatesChanged = true;
      }

      if (!haveContactStatesChanged)
         return;

      controllerToolbox.updateBipedSupportPolygons();
      balanceManager.computeICPPlan();
   }

   public void updateFailureDetection()
   {
      capturePoint2d.setIncludingFrame(balanceManager.getCapturePoint());
      failureDetectionControlModule.checkIfRobotIsFalling(capturePoint2d, balanceManager.getDesiredICP());
      if (failureDetectionControlModule.isRobotFalling())
      {
         walkingMessageHandler.clearFootsteps();
         walkingMessageHandler.clearFootTrajectory();
         commandInputManager.clearAllCommands();

         if (enablePushRecoveryOnFailure.getBooleanValue() && !balanceManager.isPushRecoveryEnabled())
         {
            balanceManager.enablePushRecovery();
         }
         else if (!balanceManager.isPushRecoveryEnabled() || balanceManager.isRecoveryImpossible())
         {
            walkingMessageHandler.reportControllerFailure(failureDetectionControlModule.getFallingDirection3D());
            controllerToolbox.reportControllerFailureToListeners(failureDetectionControlModule.getFallingDirection2D());
         }
      }

      if (enablePushRecoveryOnFailure.getBooleanValue())
      {
         if (balanceManager.isPushRecoveryEnabled() && balanceManager.isRobotBackToSafeState())
            balanceManager.disablePushRecovery();
      }
   }

   public void updateManagers(PushRecoveryState currentState)
   {
      desiredCoMVelocityAsFrameVector.setIncludingFrame(balanceManager.getDesiredCoMVelocity());

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

      pelvisOrientationManager.compute();

      comHeightManager.compute(balanceManager.getDesiredICPVelocity(),
                               desiredCoMVelocityAsFrameVector,
                               isInDoubleSupport,
                               omega0,
                               isRecoveringFromPush,
                               feetManager);
      FeedbackControlCommand<?> heightControlCommand = comHeightManager.getHeightControlCommand();

      // the comHeightManager can control the pelvis with a feedback controller and doesn't always need the z component of the momentum command. It would be better to remove the coupling between these two modules
      boolean controlHeightWithMomentum = comHeightManager.getControlHeightWithMomentum() && enableHeightFeedbackControl.getValue();
      boolean keepCMPInsideSupportPolygon = !bodyManagerIsLoadBearing;
      if (currentState.isDoubleSupportState())
         balanceManager.compute(currentState.getTransferToSide(), heightControlCommand, keepCMPInsideSupportPolygon, controlHeightWithMomentum);
      else
         balanceManager.compute(currentState.getSupportSide(), heightControlCommand, keepCMPInsideSupportPolygon, controlHeightWithMomentum);
   }

   private void reportStatusMessages()
   {
      Object statusMessage;

      for (RobotSide robotSide : RobotSide.values)
      {
         statusMessage = feetManager.pollStatusToReport(robotSide);
         if (statusMessage != null)
            statusOutputManager.reportStatusMessage(statusMessage);
      }

      for (int managerIdx = 0; managerIdx < bodyManagers.size(); managerIdx++)
      {
         RigidBodyControlManager bodyManager = bodyManagers.get(managerIdx);
         if (bodyManager != null)
         {
            statusMessage = bodyManager.pollStatusToReport();
            if (statusMessage != null)
               statusOutputManager.reportStatusMessage(statusMessage);
         }
      }

      TaskspaceTrajectoryStatusMessage pelvisOrientationStatus = pelvisOrientationManager.pollStatusToReport();
      TaskspaceTrajectoryStatusMessage pelvisXYStatus = balanceManager.pollPelvisXYTranslationStatusToReport();
      TaskspaceTrajectoryStatusMessage pelvisHeightStatus = comHeightManager.pollStatusToReport();

      TaskspaceTrajectoryStatusMessage mergedPelvisStatus = mergePelvisStatusMessages(pelvisOrientationStatus, pelvisXYStatus, pelvisHeightStatus);

      if (mergedPelvisStatus != null)
      {
         statusOutputManager.reportStatusMessage(mergedPelvisStatus);
      }
   }

   private TaskspaceTrajectoryStatusMessage mergePelvisStatusMessages(TaskspaceTrajectoryStatusMessage pelvisOrientationStatus,
                                                                      TaskspaceTrajectoryStatusMessage pelvisXYStatus,
                                                                      TaskspaceTrajectoryStatusMessage pelvisHeightStatus)
   {
      int numberOfStatus = pelvisOrientationStatus != null ? 1 : 0;
      numberOfStatus += pelvisXYStatus != null ? 1 : 0;
      numberOfStatus += pelvisHeightStatus != null ? 1 : 0;

      if (numberOfStatus <= 1)
      {
         if (pelvisOrientationStatus != null)
            return pelvisOrientationStatus;
         if (pelvisXYStatus != null)
            return pelvisXYStatus;
         if (pelvisHeightStatus != null)
            return pelvisHeightStatus;
         return null;
      }

      Quaternion desiredEndEffectorOrientation = pelvisStatusMessage.getDesiredEndEffectorOrientation();
      Point3D desiredEndEffectorPosition = pelvisStatusMessage.getDesiredEndEffectorPosition();
      Quaternion actualEndEffectorOrientation = pelvisStatusMessage.getActualEndEffectorOrientation();
      Point3D actualEndEffectorPosition = pelvisStatusMessage.getActualEndEffectorPosition();

      desiredEndEffectorOrientation.setToNaN();
      desiredEndEffectorPosition.setToNaN();
      actualEndEffectorOrientation.setToNaN();
      actualEndEffectorPosition.setToNaN();

      if (pelvisOrientationStatus != null)
      {
         pelvisStatusMessage.setSequenceId(pelvisOrientationStatus.getSequenceId());
         pelvisStatusMessage.setTimestamp(pelvisOrientationStatus.getTimestamp());
         pelvisStatusMessage.setTrajectoryExecutionStatus(pelvisOrientationStatus.getTrajectoryExecutionStatus());
         desiredEndEffectorOrientation.set(pelvisOrientationStatus.getDesiredEndEffectorOrientation());
         actualEndEffectorOrientation.set(pelvisOrientationStatus.getActualEndEffectorOrientation());
      }

      if (pelvisXYStatus != null)
      {
         pelvisStatusMessage.setSequenceId(pelvisXYStatus.getSequenceId());
         pelvisStatusMessage.setTimestamp(pelvisXYStatus.getTimestamp());
         pelvisStatusMessage.setTrajectoryExecutionStatus(pelvisXYStatus.getTrajectoryExecutionStatus());
         desiredEndEffectorPosition.set(pelvisXYStatus.getDesiredEndEffectorPosition());
         actualEndEffectorPosition.set(pelvisXYStatus.getActualEndEffectorPosition());
      }

      if (pelvisHeightStatus != null)
      {
         pelvisStatusMessage.setSequenceId(pelvisHeightStatus.getSequenceId());
         pelvisStatusMessage.setTimestamp(pelvisHeightStatus.getTimestamp());
         pelvisStatusMessage.setTrajectoryExecutionStatus(pelvisHeightStatus.getTrajectoryExecutionStatus());
         desiredEndEffectorPosition.setZ(pelvisHeightStatus.getDesiredEndEffectorPosition().getZ());
         actualEndEffectorPosition.setZ(pelvisHeightStatus.getActualEndEffectorPosition().getZ());
      }

      return pelvisStatusMessage;
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
         controllerCoreCommand.completeLowLevelJointData(feetManager.getJointDesiredData(robotSide));

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

      controllerCoreCommand.addInverseDynamicsCommand(controllerCoreOptimizationSettings.getCommand());
   }

   public ControllerCoreCommand getControllerCoreCommand()
   {
      return controllerCoreCommand;
   }

   public LinearMomentumRateControlModuleInput getLinearMomentumRateControlModuleInput()
   {
      return balanceManager.getLinearMomentumRateControlModuleInput();
   }

   public YoRegistry getYoVariableRegistry()
   {
      return registry;
   }

   /**
    * Returns the currently active walking state. This is used for unit testing.
    *
    * @return WalkingStateEnum
    */
   public PushRecoveryStateEnum getRecoveringStateEnum()
   {
      return stateMachine.getCurrentStateKey();
   }

   @Override
   public boolean isJointLoadBearing(String jointName)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         boolean legLoaded = feetManager.getCurrentConstraintType(robotSide).isLoadBearing();
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
