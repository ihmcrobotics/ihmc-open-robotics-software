package us.ihmc.simpleWholeBodyWalking;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.capturePoint.LinearMomentumRateControlModuleOutput;
import us.ihmc.commonWalkingControlModules.capturePoint.SimpleLinearMomentumRateControlModuleInput;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointLimitEnforcementMethodCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.JointLoadStatusProvider;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.ParameterizedControllerCoreOptimizationSettings;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.stateTransitionConditions.StartWalkingCondition;
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
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.simpleWholeBodyWalking.states.SimpleSingleSupportState;
import us.ihmc.simpleWholeBodyWalking.states.SimpleStandingState;
import us.ihmc.simpleWholeBodyWalking.states.SimpleTransferToStandingState;
import us.ihmc.simpleWholeBodyWalking.states.SimpleTransferToSwingState;
import us.ihmc.simpleWholeBodyWalking.states.SimpleWalkingState;
import us.ihmc.simpleWholeBodyWalking.states.SimpleWalkingStateEnum;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimpleWalkingHighLevelHumanoidController implements JointLoadStatusProvider
{
   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);

   private final YoDouble yoTime;

   private final SimpleControlManagerFactory managerFactory;

   private final SimplePelvisOrientationManager pelvisOrientationManager;
   private final SimpleFeetManager feetManager;
   private final SimpleBalanceManager balanceManager;

   private final ArrayList<RigidBodyControlManager> bodyManagers = new ArrayList<>();
   private final Map<String, RigidBodyControlManager> bodyManagerByJointName = new HashMap<>();
   private final SideDependentList<Set<String>> legJointNames = new SideDependentList<>();

   private final OneDoFJointBasics[] allOneDoFjoints;

   private final FullHumanoidRobotModel fullRobotModel;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WalkingControllerParameters walkingControllerParameters;

   private final SideDependentList<? extends ContactablePlaneBody> feet;

   private final StateMachine<SimpleWalkingStateEnum, SimpleWalkingState> stateMachine;

   private final WalkingMessageHandler walkingMessageHandler;
   private final YoBoolean abortWalkingRequested = new YoBoolean("requestAbortWalking", registry);

   private final YoDouble controlledCoMHeightAcceleration = new YoDouble("controlledCoMHeightAcceleration", registry);

   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final YoBoolean allowUpperBodyMotionDuringLocomotion = new YoBoolean("allowUpperBodyMotionDuringLocomotion", registry);

   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusOutputManager;
   private final SimpleCommandConsumer commandConsumer;

   private final TaskspaceTrajectoryStatusMessage pelvisStatusMessage = new TaskspaceTrajectoryStatusMessage();

   private final JointLimitEnforcementMethodCommand jointLimitEnforcementMethodCommand = new JointLimitEnforcementMethodCommand();
   private final YoBoolean limitCommandSent = new YoBoolean("limitCommandSent", registry);

   private final PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);
   private ControllerCoreOutputReadOnly controllerCoreOutput;

   private final ParameterizedControllerCoreOptimizationSettings controllerCoreOptimizationSettings;

   private boolean firstTick = true;

   public SimpleWalkingHighLevelHumanoidController(CommandInputManager commandInputManager,
                                                   StatusMessageOutputManager statusOutputManager,
                                                   SimpleControlManagerFactory managerFactory,
                                                   WalkingControllerParameters walkingControllerParameters,
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

      RigidBodyBasics head = fullRobotModel.getHead();
      RigidBodyBasics chest = fullRobotModel.getChest();
      RigidBodyBasics pelvis = fullRobotModel.getPelvis();

      pelvisStatusMessage.setEndEffectorName(pelvis.getName());

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
         Arrays.asList(manager.getControlledJoints()).forEach(joint -> bodyManagerByJointName.put(joint.getName(), manager));
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics foot = fullRobotModel.getFoot(robotSide);
         OneDoFJointBasics[] legJoints = MultiBodySystemTools.filterJoints(MultiBodySystemTools.createJointPath(pelvis, foot), OneDoFJointBasics.class);
         Set<String> jointNames = new HashSet<>();
         Arrays.asList(legJoints).forEach(legJoint -> jointNames.add(legJoint.getName()));
         legJointNames.put(robotSide, jointNames);
      }

      this.walkingControllerParameters = walkingControllerParameters;

      balanceManager = managerFactory.getOrCreateBalanceManager();

      this.commandInputManager = commandInputManager;
      this.statusOutputManager = statusOutputManager;

      allowUpperBodyMotionDuringLocomotion.set(walkingControllerParameters.allowUpperBodyMotionDuringLocomotion());

      failureDetectionControlModule = new WalkingFailureDetectionControlModule(controllerToolbox.getContactableFeet(), registry);

      walkingMessageHandler = controllerToolbox.getWalkingMessageHandler();
      commandConsumer = new SimpleCommandConsumer(commandInputManager,
                                                  statusOutputManager,
                                                  controllerToolbox,
                                                  managerFactory,
                                                  walkingControllerParameters,
                                                  registry);

      stateMachine = setupStateMachine();

      double highCoPDampingDuration = walkingControllerParameters.getHighCoPDampingDurationToPreventFootShakies();
      double coPErrorThreshold = walkingControllerParameters.getCoPErrorThresholdForHighCoPDamping();
      boolean enableHighCoPDamping = highCoPDampingDuration > 0.0 && !Double.isInfinite(coPErrorThreshold);
      controllerToolbox.setHighCoPDampingParameters(enableHighCoPDamping, highCoPDampingDuration, coPErrorThreshold);

      String[] jointNamesRestrictiveLimits = walkingControllerParameters.getJointsWithRestrictiveLimits();
      JointLimitParameters limitParameters = walkingControllerParameters.getJointLimitParametersForJointsWithRestictiveLimits();
      OneDoFJointBasics[] jointsWithRestrictiveLimit = MultiBodySystemTools.filterJoints(ScrewTools.findJointsWithNames(allOneDoFjoints,
                                                                                                                        jointNamesRestrictiveLimits),
                                                                                         OneDoFJointBasics.class);
      for (OneDoFJointBasics joint : jointsWithRestrictiveLimit)
      {
         if (limitParameters == null)
         {
            throw new RuntimeException("Must define joint limit parameters if using joints with restrictive limits.");
         }
         jointLimitEnforcementMethodCommand.addLimitEnforcementMethod(joint, JointLimitEnforcement.RESTRICTIVE, limitParameters);
      }

      ControllerCoreOptimizationSettings defaultControllerCoreOptimizationSettings = walkingControllerParameters.getMomentumOptimizationSettings();
      controllerCoreOptimizationSettings = new ParameterizedControllerCoreOptimizationSettings(defaultControllerCoreOptimizationSettings, registry);
   }

   private StateMachine<SimpleWalkingStateEnum, SimpleWalkingState> setupStateMachine()
   {
      StateMachineFactory<SimpleWalkingStateEnum, SimpleWalkingState> factory = new StateMachineFactory<>(SimpleWalkingStateEnum.class);
      factory.setNamePrefix("walking").setRegistry(registry).buildYoClock(yoTime);

      SimpleStandingState standingState = new SimpleStandingState(commandInputManager,
                                                                  walkingMessageHandler,
                                                                  controllerToolbox,
                                                                  managerFactory,
                                                                  failureDetectionControlModule,
                                                                  registry);
      SimpleTransferToStandingState toStandingState = new SimpleTransferToStandingState(walkingMessageHandler,
                                                                                        controllerToolbox,
                                                                                        managerFactory,
                                                                                        failureDetectionControlModule,
                                                                                        registry);
      factory.addState(SimpleWalkingStateEnum.TO_STANDING, toStandingState);
      factory.addState(SimpleWalkingStateEnum.STANDING, standingState);

      SideDependentList<SimpleTransferToSwingState> walkingTransferStates = new SideDependentList<>();

      for (RobotSide transferToSide : RobotSide.values)
      {
         SimpleWalkingStateEnum stateEnum = SimpleWalkingStateEnum.getWalkingTransferState(transferToSide);
         SimpleTransferToSwingState transferState = new SimpleTransferToSwingState(stateEnum,
                                                                                   walkingMessageHandler,
                                                                                   controllerToolbox,
                                                                                   managerFactory,
                                                                                   failureDetectionControlModule,
                                                                                   registry);
         walkingTransferStates.put(transferToSide, transferState);
         factory.addState(stateEnum, transferState);
      }

      SideDependentList<SimpleSingleSupportState> walkingSingleSupportStates = new SideDependentList<>();

      for (RobotSide supportSide : RobotSide.values)
      {
         SimpleWalkingStateEnum stateEnum = SimpleWalkingStateEnum.getWalkingSingleSupportState(supportSide);
         SimpleSingleSupportState singleSupportState = new SimpleSingleSupportState(stateEnum,
                                                                                    walkingMessageHandler,
                                                                                    controllerToolbox,
                                                                                    managerFactory,
                                                                                    walkingControllerParameters,
                                                                                    failureDetectionControlModule,
                                                                                    registry);
         walkingSingleSupportStates.put(supportSide, singleSupportState);
         factory.addState(stateEnum, singleSupportState);
      }

      factory.addDoneTransition(SimpleWalkingStateEnum.TO_STANDING, SimpleWalkingStateEnum.STANDING);

      // Setup start/stop walking conditions
      for (RobotSide robotSide : RobotSide.values)
      {
         SimpleTransferToSwingState transferState = walkingTransferStates.get(robotSide);
         SimpleSingleSupportState singleSupportState = walkingSingleSupportStates.get(robotSide);

         SimpleWalkingStateEnum transferStateEnum = transferState.getStateEnum();
         SimpleWalkingStateEnum singleSupportStateEnum = singleSupportState.getStateEnum();

         // Start walking
         factory.addTransition(Arrays.asList(SimpleWalkingStateEnum.STANDING, SimpleWalkingStateEnum.TO_STANDING),
                               transferStateEnum,
                               new StartWalkingCondition(transferStateEnum.getTransferToSide(), walkingMessageHandler));

         // Stop walking when in transfer
         factory.addTransition(transferStateEnum,
                               SimpleWalkingStateEnum.TO_STANDING,
                               (time) -> !walkingMessageHandler.isNextFootstepFor(transferState.getTransferToSide().getOppositeSide()));

         // Stop walking when in single support
         factory.addTransition(singleSupportStateEnum,
                               SimpleWalkingStateEnum.TO_STANDING,
                               (time) -> singleSupportState.isDone(time) && !walkingMessageHandler.hasUpcomingFootsteps());
      }

      // Setup walking transfer to single support conditions
      for (RobotSide robotSide : RobotSide.values)
      {
         SimpleWalkingState transferState = walkingTransferStates.get(robotSide);
         SimpleWalkingState singleSupportState = walkingSingleSupportStates.get(robotSide);

         SimpleWalkingStateEnum transferStateEnum = transferState.getStateEnum();
         SimpleWalkingStateEnum singleSupportStateEnum = singleSupportState.getStateEnum();

         factory.addDoneTransition(transferStateEnum, singleSupportStateEnum);
      }

      // Setup walking single support to transfer conditions
      for (RobotSide robotSide : RobotSide.values)
      {
         SimpleSingleSupportState singleSupportState = walkingSingleSupportStates.get(robotSide);
         SimpleWalkingStateEnum singleSupportStateEnum = singleSupportState.getStateEnum();

         // Single support to transfer with same side
         {
            SimpleTransferToSwingState transferState = walkingTransferStates.get(robotSide);
            SimpleWalkingStateEnum transferStateEnum = transferState.getStateEnum();

            factory.addTransition(singleSupportStateEnum, transferStateEnum, (time) ->
            {
               if (!singleSupportState.isDone(time))
                  return false;
               return walkingMessageHandler.isNextFootstepFor(transferState.getTransferToSide().getOppositeSide());
            });
         }

         // Single support to transfer with opposite side
         {
            SimpleTransferToSwingState transferState = walkingTransferStates.get(robotSide.getOppositeSide());
            SimpleWalkingStateEnum transferStateEnum = transferState.getStateEnum();

            factory.addTransition(singleSupportStateEnum, transferStateEnum, (time) ->
            {
               if (!singleSupportState.isDone(time))
                  return false;
               return walkingMessageHandler.isNextFootstepFor(transferState.getTransferToSide().getOppositeSide());
            });
         }
      }

      // Setup the abort condition from all states to the toStandingState
      Set<SimpleWalkingStateEnum> allButStandingStates = EnumSet.complementOf(EnumSet.of(SimpleWalkingStateEnum.STANDING, SimpleWalkingStateEnum.TO_STANDING));
      factory.addTransition(allButStandingStates, SimpleWalkingStateEnum.TO_STANDING, new AbortCondition());



      // Update the previous state info for each state using state changed listeners.
      factory.getRegisteredStates().forEach(state -> factory.addStateChangedListener((from, to) -> state.setPreviousWalkingStateEnum(from)));
      factory.addStateChangedListener(controllerToolbox::reportControllerStateChangeToListeners);

      return factory.build(SimpleWalkingStateEnum.TO_STANDING);
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
      controllerCoreCommand.requestReinitialization();
      controllerToolbox.initialize();
      managerFactory.initializeManagers();

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
         privilegedConfigurationCommand.addJoint(fullRobotModel.getLegJoint(robotSide, LegJointName.KNEE_PITCH), PrivilegedConfigurationOption.AT_MID_RANGE);
      }

      initializeContacts();

      for (RobotSide robotSide : RobotSide.values)
      {
         footDesiredCoPs.get(robotSide).setToZero(feet.get(robotSide).getSoleFrame());
         controllerToolbox.setDesiredCenterOfPressure(feet.get(robotSide), footDesiredCoPs.get(robotSide));
      }

      stateMachine.resetToInitialState();

      initializeManagers();

      commandConsumer.avoidManipulationAbortForDuration(RigidBodyControlManager.INITIAL_GO_HOME_TIME);

      firstTick = true;
   }

   private void initializeManagers()
   {
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
      //      balanceManager.initialize();  // already initialized, so don't run it again, or else the state machine gets reset.
      feetManager.initialize();
   }

   private void initializeContacts()
   {
      controllerToolbox.clearContacts();

      for (RobotSide robotSide : RobotSide.values)
      {
         controllerToolbox.resetFootPlaneContactPoint(robotSide);
         feetManager.setFlatFootContactState(robotSide);
      }
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

   private final FrameVector2D desiredICPVelocityAsFrameVector = new FrameVector2D();
   private final FrameVector2D desiredCoMVelocityAsFrameVector = new FrameVector2D();

   private final SideDependentList<FramePoint2D> footDesiredCoPs = new SideDependentList<FramePoint2D>(new FramePoint2D(), new FramePoint2D());
   private final RecyclingArrayList<PlaneContactStateCommand> planeContactStateCommandPool = new RecyclingArrayList<>(4, PlaneContactStateCommand.class);
   private final FramePoint2D capturePoint2d = new FramePoint2D();
   private final FramePoint2D desiredCapturePoint2d = new FramePoint2D();

   public void doAction()
   {
      SimpleWalkingState currentState = stateMachine.getCurrentState();
      commandConsumer.update();
      commandConsumer.consumeChestCommands();
      commandConsumer.consumePelvisHeightCommands();
      commandConsumer.consumeGoHomeMessages();
      commandConsumer.consumeStopAllTrajectoryCommands();
      commandConsumer.consumeFootCommands();
      commandConsumer.consumeAbortWalkingCommands(abortWalkingRequested);
      commandConsumer.consumePelvisCommands(currentState, allowUpperBodyMotionDuringLocomotion.getBooleanValue());
      commandConsumer.consumeManipulationCommands(currentState, allowUpperBodyMotionDuringLocomotion.getBooleanValue());
      commandConsumer.handleAutomaticManipulationAbortOnICPError(currentState);
      commandConsumer.consumePlanarRegionStepConstraintCommand();
      commandConsumer.consumePrepareForLocomotionCommands();

      updateFailureDetection();

      // Do transitions will request ICP planner updates.
      if (!firstTick) // this avoids doing two transitions in a single tick if the initialize reset the state machine.
         stateMachine.doTransitions();
      // This updates the ICP plan continuously.
      balanceManager.update();
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
         commandInputManager.clearAllCommands();
      }

   }

   public void updateManagers(SimpleWalkingState currentState)
   {
      desiredICPVelocityAsFrameVector.setToZero(ReferenceFrame.getWorldFrame());
      desiredCoMVelocityAsFrameVector.setToZero(ReferenceFrame.getWorldFrame());
      balanceManager.getDesiredICPVelocity(desiredICPVelocityAsFrameVector);
      balanceManager.getDesiredCoMVelocity(desiredCoMVelocityAsFrameVector);

      boolean isInDoubleSupport = currentState.isDoubleSupportState();
      double omega0 = controllerToolbox.getOmega0();

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

      pelvisOrientationManager.compute();

      // the comHeightManager can control the pelvis with a feedback controller and doesn't always need the z component of the momentum command. It would be better to remove the coupling between these two modules
      boolean keepCMPInsideSupportPolygon = !bodyManagerIsLoadBearing;
      RobotSide side = currentState.isDoubleSupportState() ? currentState.getTransferToSide() : currentState.getSupportSide();
      balanceManager.compute(side, keepCMPInsideSupportPolygon, true);
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

      TaskspaceTrajectoryStatusMessage pelvisXYStatus = balanceManager.pollPelvisXYTranslationStatusToReport();
      
      // TODO: Get Pelvis height tracking status from SimpleBalanceManager rather than comHeightManager
      TaskspaceTrajectoryStatusMessage pelvisHeightStatus = null;

      TaskspaceTrajectoryStatusMessage mergedPelvisStatus = mergePelvisStatusMessages(pelvisXYStatus, pelvisHeightStatus);

      if (mergedPelvisStatus != null)
      {
         statusOutputManager.reportStatusMessage(mergedPelvisStatus);
      }
   }

   private TaskspaceTrajectoryStatusMessage mergePelvisStatusMessages(TaskspaceTrajectoryStatusMessage pelvisXYStatus,
                                                                      TaskspaceTrajectoryStatusMessage pelvisHeightStatus)
   {
      int numberOfStatus = pelvisXYStatus != null ? 1 : 0;
      numberOfStatus += pelvisHeightStatus != null ? 1 : 0;

      if (numberOfStatus <= 1)
      {
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

      controllerCoreCommand.addInverseDynamicsCommand(controllerCoreOptimizationSettings.getCommand());
   }

   public ControllerCoreCommand getControllerCoreCommand()
   {
      return controllerCoreCommand;
   }

   public SimpleLinearMomentumRateControlModuleInput getLinearMomentumRateControlModuleInput()
   {
      return balanceManager.getLinearMomentumRateControlModuleInput();
   }

   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   /**
    * Returns the currently active walking state. This is used for unit testing.
    *
    * @return WalkingStateEnum
    */
   public SimpleWalkingStateEnum getWalkingStateEnum()
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
