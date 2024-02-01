package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.capturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.capturePoint.LinearMomentumRateControlModuleInput;
import us.ihmc.commonWalkingControlModules.capturePoint.LinearMomentumRateControlModuleOutput;
import us.ihmc.commonWalkingControlModules.captureRegion.MultiStepPushRecoveryControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.JointLoadStatusProvider;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.ParameterizedControllerCoreOptimizationSettings;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.stateTransitionConditions.ContinuePushRecoveryWithNextStepCondition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.stateTransitionConditions.RecoveryTransferToStandingCondition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.stateTransitionConditions.StandingToSwingCondition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.states.PushRecoveryState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.states.PushRecoveryStateEnum;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.states.RecoveringSwingState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.states.RecoveryTransferState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.states.TransferToStandingPushRecoveryState;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.OneDoFJointPrivilegedConfigurationParameters;
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
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class PushRecoveryHighLevelHumanoidController implements JointLoadStatusProvider, SCS2YoGraphicHolder
{
   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);

   private final YoDouble yoTime;

   private final PushRecoveryControlManagerFactory managerFactory;

   private final PelvisOrientationManager pelvisOrientationManager;
   private final FeetManager feetManager;
   private final PushRecoveryBalanceManager balanceManager;
   private final CenterOfMassHeightManager comHeightManager;

   private final ArrayList<RigidBodyControlManager> bodyManagers = new ArrayList<>();
   private final Map<String, RigidBodyControlManager> bodyManagerByJointName = new HashMap<>();
   private final SideDependentList<Set<String>> legJointNames = new SideDependentList<>();

   private final FullHumanoidRobotModel fullRobotModel;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final PushRecoveryControllerParameters pushRecoveryControllerParameters;
   private final OneDoFJointPrivilegedConfigurationParameters kneePrivilegedConfigurationParameters;

   private final SideDependentList<? extends ContactablePlaneBody> feet;

   private final StateMachine<PushRecoveryStateEnum, PushRecoveryState> stateMachine;

   private final WalkingMessageHandler walkingMessageHandler;

   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusOutputManager;

   private final TaskspaceTrajectoryStatusMessage pelvisStatusMessage = new TaskspaceTrajectoryStatusMessage();

   private final PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);
   private ControllerCoreOutputReadOnly controllerCoreOutput;

   private final ParameterizedControllerCoreOptimizationSettings controllerCoreOptimizationSettings;
   private final MultiStepPushRecoveryControlModule pushRecoveryControlModule;

   private final YoBoolean enableHeightFeedbackControl = new YoBoolean("enableHeightFeedbackControl", registry);
   private final YoInteger numberOfRecoveryStepsTaken = new YoInteger("numberOfRecoveryStepsTaken", registry);
   private final IntegerParameter maxNumberOfRecoveryStepsToTake = new IntegerParameter("maxNumberOfRecoveryStepsToTake", registry, 5);

   private boolean firstTick = true;

   public PushRecoveryHighLevelHumanoidController(CommandInputManager commandInputManager,
                                                  StatusMessageOutputManager statusOutputManager,
                                                  PushRecoveryControlManagerFactory managerFactory,
                                                  PushRecoveryControllerParameters pushRecoveryControllerParameters,
                                                  OneDoFJointPrivilegedConfigurationParameters kneePrivilegedConfigurationParameters,
                                                  HighLevelHumanoidControllerToolbox controllerToolbox)
   {
      this.managerFactory = managerFactory;
      this.kneePrivilegedConfigurationParameters = kneePrivilegedConfigurationParameters;

      // Getting parameters from the HighLevelHumanoidControllerToolbox
      this.controllerToolbox = controllerToolbox;
      fullRobotModel = controllerToolbox.getFullRobotModel();
      yoTime = controllerToolbox.getYoTime();

      feet = controllerToolbox.getContactableFeet();
      this.pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      this.feetManager = managerFactory.getOrCreateFeetManager();

      pushRecoveryControlModule = new MultiStepPushRecoveryControlModule(controllerToolbox.getFootContactStates(),
                                                                         controllerToolbox.getBipedSupportPolygons(),
                                                                         controllerToolbox.getReferenceFrames().getSoleZUpFrames(),
                                                                         controllerToolbox.getDefaultFootPolygons().get(RobotSide.LEFT),
                                                                         pushRecoveryControllerParameters,
                                                                         registry,
                                                                         controllerToolbox.getYoGraphicsListRegistry());

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

      stateMachine = setupStateMachine();

      ControllerCoreOptimizationSettings defaultControllerCoreOptimizationSettings = pushRecoveryControllerParameters.getMomentumOptimizationSettings();
      controllerCoreOptimizationSettings = new ParameterizedControllerCoreOptimizationSettings(defaultControllerCoreOptimizationSettings, registry);
   }

   private StateMachine<PushRecoveryStateEnum, PushRecoveryState> setupStateMachine()
   {
      StateMachineFactory<PushRecoveryStateEnum, PushRecoveryState> factory = new StateMachineFactory<>(PushRecoveryStateEnum.class);
      factory.setNamePrefix("pushRecovery").setRegistry(registry).buildYoClock(yoTime);

      TransferToStandingPushRecoveryState toStandingState = new TransferToStandingPushRecoveryState(walkingMessageHandler, controllerToolbox,
                                                                                                    pushRecoveryControllerParameters,
                                                                                                    pushRecoveryControlModule, managerFactory,
                                                                                                    failureDetectionControlModule, registry);
      factory.addState(PushRecoveryStateEnum.TO_STANDING, toStandingState);


      SideDependentList<RecoveryTransferState> recoveryTransferStates = new SideDependentList<>();
      for (RobotSide transferToSide : RobotSide.values)
      {
         PushRecoveryStateEnum stateEnum = PushRecoveryStateEnum.getPushRecoveryTransferState(transferToSide);
         RecoveryTransferState transferState = new RecoveryTransferState(stateEnum,
                                                                         walkingMessageHandler,
                                                                         controllerToolbox,
                                                                         pushRecoveryControllerParameters,
                                                                         managerFactory,
                                                                         pushRecoveryControlModule,
                                                                         failureDetectionControlModule,
                                                                         registry);
         recoveryTransferStates.put(transferToSide, transferState);
         factory.addState(stateEnum, transferState);
      }

      Runnable stepStartedListener = numberOfRecoveryStepsTaken::increment;
      SideDependentList<RecoveringSwingState> recoveringSingleSupportStates = new SideDependentList<>();

      for (RobotSide supportSide : RobotSide.values)
      {
         PushRecoveryStateEnum stateEnum = PushRecoveryStateEnum.getPushRecoverySingleSupportState(supportSide);
         RecoveringSwingState singleSupportState = new RecoveringSwingState(stateEnum,
                                                                            walkingMessageHandler,
                                                                            controllerToolbox, managerFactory,
                                                                            pushRecoveryControlModule,
                                                                            pushRecoveryControllerParameters,
                                                                            stepStartedListener,
                                                                            failureDetectionControlModule,
                                                                            registry);
         recoveringSingleSupportStates.put(supportSide, singleSupportState);
         factory.addState(stateEnum, singleSupportState);
      }

      // Setup start swing conditions
      for (RobotSide supportSide : RobotSide.values)
      {
         factory.addTransition(PushRecoveryStateEnum.TO_STANDING,
                               PushRecoveryStateEnum.getPushRecoverySingleSupportState(supportSide),
                               new StandingToSwingCondition(supportSide.getOppositeSide(), pushRecoveryControlModule));
      }

      // Setup push recovery single support to transfer conditions
      for (RobotSide robotSide : RobotSide.values)
      {
         factory.addDoneTransition(PushRecoveryStateEnum.getPushRecoverySingleSupportState(robotSide),
                                   PushRecoveryStateEnum.getPushRecoveryTransferState(robotSide.getOppositeSide()));
      }

      // Setup push recovery transfer to single support conditions or standing
      for (RobotSide robotSide : RobotSide.values)
      {
         RecoveryTransferState transferState = recoveryTransferStates.get(robotSide);
         PushRecoveryState singleSupportState = recoveringSingleSupportStates.get(robotSide);

         PushRecoveryStateEnum transferStateEnum = transferState.getStateEnum();
         PushRecoveryStateEnum singleSupportStateEnum = singleSupportState.getStateEnum();

         factory.addTransition(transferStateEnum,
                               singleSupportStateEnum,
                               new ContinuePushRecoveryWithNextStepCondition(transferState, pushRecoveryControlModule));

         factory.addTransition(transferStateEnum,
                               PushRecoveryStateEnum.TO_STANDING,
                               new RecoveryTransferToStandingCondition(transferState, pushRecoveryControlModule));
      }

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

   public void reset()
   {
      pushRecoveryControlModule.reset();
   }

   public void initialize()
   {
      reset();

      numberOfRecoveryStepsTaken.set(0);

      commandInputManager.clearAllCommands();
      walkingMessageHandler.clearFootsteps();
      walkingMessageHandler.clearFlamingoCommands();

      balanceManager.initialize();

      privilegedConfigurationCommand.clear();
      privilegedConfigurationCommand.setPrivilegedConfigurationOption(PrivilegedConfigurationOption.AT_ZERO);

      for (RobotSide robotSide : RobotSide.values)
      {
         ArmJointName[] armJointNames = fullRobotModel.getRobotSpecificJointNames().getArmJointNames();
         for (int i = 0; i < armJointNames.length; i++)
            privilegedConfigurationCommand.addJoint(fullRobotModel.getArmJoint(robotSide, armJointNames[i]), PrivilegedConfigurationOption.AT_MID_RANGE);

         OneDoFJointBasics kneeJoint = fullRobotModel.getLegJoint(robotSide, LegJointName.KNEE_PITCH);
         privilegedConfigurationCommand.addJoint(kneeJoint, kneePrivilegedConfigurationParameters);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         footDesiredCoPs.get(robotSide).setToZero(feet.get(robotSide).getSoleFrame());
         controllerToolbox.setDesiredCenterOfPressure(feet.get(robotSide), footDesiredCoPs.get(robotSide));
      }

      stateMachine.resetToInitialState();

      firstTick = true;
   }

   private final FrameVector2D desiredCoMVelocityAsFrameVector = new FrameVector2D();

   private final SideDependentList<FramePoint2D> footDesiredCoPs = new SideDependentList<FramePoint2D>(new FramePoint2D(), new FramePoint2D());
   private final RecyclingArrayList<PlaneContactStateCommand> planeContactStateCommandPool = new RecyclingArrayList<>(4, PlaneContactStateCommand.class);
   private final FramePoint2D capturePoint2d = new FramePoint2D();

   public void doAction()
   {
      PushRecoveryState currentState;

      updateFailureDetection();

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

      //TODO add !pushRecoveryControlModule.isEnabled() to 'if' statement
      if (pushRecoveryControlModule.isRecoveryImpossible() || numberOfRecoveryStepsTaken.getValue() > maxNumberOfRecoveryStepsToTake.getValue())
      {
         walkingMessageHandler.reportControllerFailure(failureDetectionControlModule.getFallingDirection3D());
         controllerToolbox.reportControllerFailureToListeners(failureDetectionControlModule.getFallingDirection2D());
      }
   }

   public void updateManagers(PushRecoveryState currentState)
   {
      desiredCoMVelocityAsFrameVector.setIncludingFrame(balanceManager.getDesiredCoMVelocity());

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

      comHeightManager.compute(balanceManager.getDesiredICPVelocity(),
                               desiredCoMVelocityAsFrameVector,
                               isInDoubleSupport,
                               omega0,
                               feetManager);
      FeedbackControlCommand<?> heightControlCommand = comHeightManager.getHeightControlCommand();

      // the comHeightManager can control the pelvis with a feedback controller and doesn't always need the z component of the momentum command. It would be better to remove the coupling between these two modules
      boolean controlHeightWithMomentum = comHeightManager.getControlHeightWithMomentum() && enableHeightFeedbackControl.getValue();
      boolean keepCMPInsideSupportPolygon = !bodyManagerIsLoadBearing;
      balanceManager.compute(heightControlCommand, keepCMPInsideSupportPolygon, controlHeightWithMomentum);
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
      TaskspaceTrajectoryStatusMessage pelvisHeightStatus = comHeightManager.pollStatusToReport();

      TaskspaceTrajectoryStatusMessage mergedPelvisStatus = mergePelvisStatusMessages(pelvisOrientationStatus, pelvisHeightStatus);

      if (mergedPelvisStatus != null)
      {
         statusOutputManager.reportStatusMessage(mergedPelvisStatus);
      }
   }

   private TaskspaceTrajectoryStatusMessage mergePelvisStatusMessages(TaskspaceTrajectoryStatusMessage pelvisOrientationStatus,
                                                                      TaskspaceTrajectoryStatusMessage pelvisHeightStatus)
   {
      int numberOfStatus = pelvisOrientationStatus != null ? 1 : 0;
      numberOfStatus += pelvisHeightStatus != null ? 1 : 0;

      if (numberOfStatus <= 1)
      {
         if (pelvisOrientationStatus != null)
            return pelvisOrientationStatus;
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

      boolean isHighCoPDampingNeeded = controllerToolbox.estimateIfHighCoPDampingNeeded(footDesiredCoPs);

      for (RobotSide robotSide : RobotSide.values)
      {
         controllerCoreCommand.addFeedbackControlCommand(feetManager.getFeedbackControlCommand(robotSide));
         controllerCoreCommand.addInverseDynamicsCommand(feetManager.getInverseDynamicsCommand(robotSide));
         controllerCoreCommand.completeLowLevelJointData(feetManager.getJointDesiredData(robotSide));

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

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(pushRecoveryControlModule.getSCS2YoGraphics());
      return group;
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

   public PushRecoveryState getRecoveringState()
   {
      return stateMachine.getCurrentState();
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
