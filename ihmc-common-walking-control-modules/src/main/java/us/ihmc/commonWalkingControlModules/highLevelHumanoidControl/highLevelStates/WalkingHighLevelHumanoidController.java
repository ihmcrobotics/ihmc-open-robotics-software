package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.capturePoint.BalanceManager;
import us.ihmc.commonWalkingControlModules.capturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.capturePoint.LinearMomentumRateControlModuleInput;
import us.ihmc.commonWalkingControlModules.capturePoint.LinearMomentumRateControlModuleOutput;
import us.ihmc.commonWalkingControlModules.configurations.HumanoidRobotNaturalPosture;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.NaturalPosture.NaturalPostureManager;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
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
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.TouchdownErrorCompensator;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.WalkingCommandConsumer;
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
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitEnforcement;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.OneDoFJointPrivilegedConfigurationParameters;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
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
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class WalkingHighLevelHumanoidController implements JointLoadStatusProvider
{
   private static final boolean ENABLE_LEG_ELASTICITY_DEBUGGATOR = false;

   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);

   private final YoBoolean turnOnNaturalPostureControl = new YoBoolean("turnOnNaturalPostureControl", registry);
   private final YoBoolean useNaturalPostureCommand = new YoBoolean("useNaturalPostureCommand", registry);
   private final YoBoolean usePelvisPrivilegedPoseCommand = new YoBoolean("usePelvisPrivilegedPoseCommand", registry);
   private final YoBoolean useBodyManagerCommands = new YoBoolean("useBodyManagerCommands", registry);
   private final YoBoolean usePelvisOrientationCommand = new YoBoolean("usePelvisOrientationCommand", registry);

   private final YoDouble yoTime;

   private final HighLevelControlManagerFactory managerFactory;

   private final PelvisOrientationManager pelvisOrientationManager;
   private final NaturalPostureManager naturalPostureManager;
   private final FeetManager feetManager;
   private final BalanceManager balanceManager;
   private final CenterOfMassHeightManager comHeightManager;

   private final TouchdownErrorCompensator touchdownErrorCompensator;

   private final AtomicBoolean shouldKeepInitialContacts = new AtomicBoolean();
   private final ArrayList<RigidBodyControlManager> bodyManagers = new ArrayList<>();
   private final Map<String, RigidBodyControlManager> bodyManagerByJointName = new HashMap<>();
   private final SideDependentList<Set<String>> legJointNames = new SideDependentList<>();

   private final OneDoFJointBasics[] allOneDoFjoints;

   private final FullHumanoidRobotModel fullRobotModel;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WalkingControllerParameters walkingControllerParameters;

   private final SideDependentList<? extends ContactablePlaneBody> feet;

   private final StateMachine<WalkingStateEnum, WalkingState> stateMachine;

   private final WalkingMessageHandler walkingMessageHandler;
   private final YoBoolean abortWalkingRequested = new YoBoolean("requestAbortWalking", registry);

   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final YoBoolean enablePushRecoveryOnFailure = new YoBoolean("enablePushRecoveryOnFailure", registry);

   private final YoBoolean allowUpperBodyMotionDuringLocomotion = new YoBoolean("allowUpperBodyMotionDuringLocomotion", registry);

   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusOutputManager;
   private final WalkingCommandConsumer commandConsumer;

   private final TaskspaceTrajectoryStatusMessage pelvisStatusMessage = new TaskspaceTrajectoryStatusMessage();

   private final JointLimitEnforcementMethodCommand jointLimitEnforcementMethodCommand = new JointLimitEnforcementMethodCommand();
   private final YoBoolean limitCommandSent = new YoBoolean("limitCommandSent", registry);

   private final LegElasticityDebuggator legElasticityDebuggator;

   private final PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);
   private ControllerCoreOutputReadOnly controllerCoreOutput;

   private final DoubleProvider unloadFraction;

   private final ParameterizedControllerCoreOptimizationSettings controllerCoreOptimizationSettings;

   private final ExecutionTimer walkingStateTimer = new ExecutionTimer("walkingStateTimer", registry);
   private final ExecutionTimer managerUpdateTimer = new ExecutionTimer("managerUpdateTimer", registry);
   private final YoBoolean enableHeightFeedbackControl = new YoBoolean("enableHeightFeedbackControl", registry);

   private boolean firstTick = true;

   private HumanoidRobotNaturalPosture naturalPosture;

   private boolean useSpinePitchPrivilegedCommand;
   
   private final YoDouble pPoseSpineRoll = new YoDouble("pPoseSpineRoll", registry);
   private final YoDouble pPoseSpinePitch = new YoDouble("pPoseSpinePitch", registry);
   private final YoDouble pPoseSpineYaw = new YoDouble("pPoseSpineYaw", registry);
   private final YoDouble pPoseSpineRollKp = new YoDouble("pPoseSpineRollKp", registry);
   private final YoDouble pPoseSpineRollKdFactor = new YoDouble("pPoseSpineRollKdFactor", registry);
   private final YoDouble pPoseSpinePitchKp = new YoDouble("pPoseSpinePitchKp", registry);
   private final YoDouble pPoseSpinePitchKdFactor = new YoDouble("pPoseSpinePitchKdFactor", registry);
   private final YoDouble pPoseSpineYawKp = new YoDouble("pPoseSpineYawKp", registry);
   private final YoDouble pPoseSpineYawKdFactor = new YoDouble("pPoseSpineYawKdFactor", registry);
   private final YoDouble pPoseShoulderPitch = new YoDouble("pPoseShoulderPitch",registry);
   private final YoDouble pPoseShoulderRoll = new YoDouble("pPoseShoulderRoll",registry);
   private final YoDouble pPoseShoulderYaw = new YoDouble("pPoseShoulderYaw",registry);
   private final YoDouble pPoseElbow = new YoDouble("pPoseElbow",registry);
   private final YoDouble pPoseShoulderPitchKp = new YoDouble("pPoseShoulderPitchKp",registry);
   private final YoDouble pPoseShoulderRollKp = new YoDouble("pPoseShoulderRollKp",registry);
   private final YoDouble pPoseShoulderYawKp = new YoDouble("pPoseShoulderYawKp",registry);
   private final YoDouble pPoseElbowKp = new YoDouble("pPoseElbowKp",registry);
   private final YoDouble pPoseShoulderPitchKdFactor = new YoDouble("pPoseShoulderPitchKdFactor",registry);
   private final YoDouble pPoseShoulderRollKdFactor = new YoDouble("pPoseShoulderRollKdFactor",registry);
   private final YoDouble pPoseShoulderYawKdFactor = new YoDouble("pPoseShoulderYawKdFactor",registry);
   private final YoDouble pPoseElbowKdFactor = new YoDouble("pPoseElbowKdFactor",registry);
   
//   private final YoDouble pPoseHipPitch = new YoDouble("pPoseHipPitch", registry);
   private final YoDouble pPoseHipPitchKp = new YoDouble("pPoseHipPitchKp", registry);
   private final YoDouble pPoseHipPitchKdFactor = new YoDouble("pPoseHipPitchKdFactor", registry);
//   private final YoDouble pPoseHipRoll = new YoDouble("pPoseHipRoll", registry);
   private final YoDouble pPoseHipRollKp = new YoDouble("pPoseHipRollKp", registry);
   private final YoDouble pPoseHipRollKdFactor = new YoDouble("pPoseHipRollKdFactor", registry);
//   private final YoDouble pPoseHipYaw = new YoDouble("pPoseHipYaw", registry);
   private final YoDouble pPoseHipYawKp = new YoDouble("pPoseHipYawKp", registry);
   private final YoDouble pPoseHipYawKdFactor = new YoDouble("pPoseHipYawKdFactor", registry);
//   private final YoDouble pPoseKnee = new YoDouble("pPoseKnee", registry);
   private final YoDouble pPoseKneeKp = new YoDouble("pPoseKneeKp", registry);
   private final YoDouble pPoseKneeKdFactor = new YoDouble("pPoseKneeKdFactor", registry);
   

   public WalkingHighLevelHumanoidController(CommandInputManager commandInputManager,
                                             StatusMessageOutputManager statusOutputManager,
                                             HighLevelControlManagerFactory managerFactory,
                                             WalkingControllerParameters walkingControllerParameters,
                                             HighLevelHumanoidControllerToolbox controllerToolbox)
   {
      turnOnNaturalPostureControl.set(true);
      useNaturalPostureCommand.set(true);
      usePelvisPrivilegedPoseCommand.set(false);
      useBodyManagerCommands.set(true);
      usePelvisOrientationCommand.set(true);

      this.managerFactory = managerFactory;

      // Getting parameters from the HighLevelHumanoidControllerToolbox
      this.controllerToolbox = controllerToolbox;
      fullRobotModel = controllerToolbox.getFullRobotModel();
      yoTime = controllerToolbox.getYoTime();

      feet = controllerToolbox.getContactableFeet();

      allOneDoFjoints = MultiBodySystemTools.filterJoints(controllerToolbox.getControlledJoints(), OneDoFJointBasics.class);

      this.pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      this.naturalPostureManager = managerFactory.getOrCreateNaturalPostureManager();
      this.feetManager = managerFactory.getOrCreateFeetManager();

      RigidBodyBasics head = fullRobotModel.getHead();
      RigidBodyBasics chest = fullRobotModel.getChest();
      RigidBodyBasics pelvis = fullRobotModel.getPelvis();

      pelvisStatusMessage.setEndEffectorName(pelvis.getName());

      unloadFraction = walkingControllerParameters.enforceSmoothFootUnloading() ? new DoubleParameter("unloadFraction", registry, 0.5) : null;

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
         RigidBodyBasics handBaseBody = chest != null ? chest : pelvis;
         RigidBodyControlManager handManager = managerFactory.getOrCreateRigidBodyManager(hand, handBaseBody, handControlFrame, chestBodyFrame);
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

      this.walkingControllerParameters = walkingControllerParameters;

      balanceManager = managerFactory.getOrCreateBalanceManager();
      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();

      this.commandInputManager = commandInputManager;
      this.statusOutputManager = statusOutputManager;

      allowUpperBodyMotionDuringLocomotion.set(walkingControllerParameters.allowUpperBodyMotionDuringLocomotion());
      enableHeightFeedbackControl.set(walkingControllerParameters.enableHeightFeedbackControl());

      failureDetectionControlModule = controllerToolbox.getFailureDetectionControlModule();

      walkingMessageHandler = controllerToolbox.getWalkingMessageHandler();
      commandConsumer = new WalkingCommandConsumer(commandInputManager,
                                                   statusOutputManager,
                                                   controllerToolbox,
                                                   managerFactory,
                                                   walkingControllerParameters,
                                                   registry);

      touchdownErrorCompensator = new TouchdownErrorCompensator(walkingMessageHandler, controllerToolbox.getReferenceFrames().getSoleFrames(), registry);
      stateMachine = setupStateMachine();

      double highCoPDampingDuration = walkingControllerParameters.getHighCoPDampingDurationToPreventFootShakies();
      double coPErrorThreshold = walkingControllerParameters.getCoPErrorThresholdForHighCoPDamping();
      boolean enableHighCoPDamping = highCoPDampingDuration > 0.0 && !Double.isInfinite(coPErrorThreshold);
      controllerToolbox.setHighCoPDampingParameters(enableHighCoPDamping, highCoPDampingDuration, coPErrorThreshold);

      String[] jointNamesRestrictiveLimits = walkingControllerParameters.getJointsWithRestrictiveLimits();
      OneDoFJointBasics[] jointsWithRestrictiveLimit = MultiBodySystemTools.filterJoints(ScrewTools.findJointsWithNames(allOneDoFjoints,
                                                                                                                        jointNamesRestrictiveLimits),
                                                                                         OneDoFJointBasics.class);
      for (OneDoFJointBasics joint : jointsWithRestrictiveLimit)
      {
         JointLimitParameters limitParameters = walkingControllerParameters.getJointLimitParametersForJointsWithRestrictiveLimits(joint.getName());
         if (limitParameters == null)
            throw new RuntimeException("Must define joint limit parameters for joint " + joint.getName() + " if using joints with restrictive limits.");
         jointLimitEnforcementMethodCommand.addLimitEnforcementMethod(joint, JointLimitEnforcement.RESTRICTIVE, limitParameters);
      }

      if (ENABLE_LEG_ELASTICITY_DEBUGGATOR)
      {
         // This guy can be used to add shearing forces to the feet while measuring the distance between them. Really useful to debug and identify elasticity in the legs.
         legElasticityDebuggator = new LegElasticityDebuggator(controllerToolbox.getReferenceFrames(),
                                                               new SideDependentList<>(side -> feet.get(side).getRigidBody()),
                                                               yoTime,
                                                               registry);
      }
      else
      {
         legElasticityDebuggator = null;
      }

      ControllerCoreOptimizationSettings defaultControllerCoreOptimizationSettings = walkingControllerParameters.getMomentumOptimizationSettings();
      controllerCoreOptimizationSettings = new ParameterizedControllerCoreOptimizationSettings(defaultControllerCoreOptimizationSettings, registry);

      // Why do we create another NP object here? Can we just use the one in the NPmanager?
//      this.naturalPosture = walkingControllerParameters.getNaturalPosture(fullRobotModel, registry, controllerToolbox.getYoGraphicsListRegistry());
      this.naturalPosture = this.naturalPostureManager.getRobotNaturalPosture();
      if (naturalPosture != null)
         naturalPosture.initialize();

      // privileged configuration for upper body
      useSpinePitchPrivilegedCommand = true;
      
      pPoseSpineRoll.set(0.0);
      pPoseSpinePitch.set(0.0);
      pPoseSpineYaw.set(0.0);
      pPoseShoulderPitch.set(0.0);
      pPoseShoulderRoll.set(0);
      pPoseShoulderYaw.set(0);
      pPoseElbow.set(0);

      pPoseSpineRollKp.set(50.0);
      pPoseSpinePitchKp.set(50.0);
      pPoseSpineYawKp.set(300.0);
      pPoseShoulderPitchKp.set(80.0);
      pPoseShoulderRollKp.set(80.0);
      pPoseShoulderYawKp.set(80.0);
      pPoseElbowKp.set(30.0);

      pPoseSpineRollKdFactor.set(0.15);
      pPoseSpinePitchKdFactor.set(0.15);
      pPoseSpineYawKdFactor.set(0.15);
      pPoseShoulderPitchKdFactor.set(0.15);
      pPoseShoulderRollKdFactor.set(0.15);
      pPoseShoulderYawKdFactor.set(0.15);
      pPoseElbowKdFactor.set(0.15);

      // privileged configuration for lower body
      pPoseHipPitchKp.set(100);
      pPoseHipPitchKdFactor.set(0.2);
      pPoseHipRollKp.set(100);
      pPoseHipRollKdFactor.set(0.2);
      pPoseHipYawKp.set(100);
      pPoseHipYawKdFactor.set(0.2);
      pPoseKneeKp.set(100);
      pPoseKneeKdFactor.set(0.2);
   }

   private StateMachine<WalkingStateEnum, WalkingState> setupStateMachine()
   {
      StateMachineFactory<WalkingStateEnum, WalkingState> factory = new StateMachineFactory<>(WalkingStateEnum.class);
      factory.setNamePrefix("walking").setRegistry(registry).buildYoClock(yoTime);

      StandingState standingState = new StandingState(commandInputManager,
                                                      walkingMessageHandler,
                                                      touchdownErrorCompensator,
                                                      controllerToolbox,
                                                      managerFactory,
                                                      failureDetectionControlModule,
                                                      walkingControllerParameters,
                                                      registry);
      TransferToStandingState toStandingState = new TransferToStandingState(walkingMessageHandler,
                                                                            touchdownErrorCompensator,
                                                                            controllerToolbox,
                                                                            managerFactory,
                                                                            failureDetectionControlModule,
                                                                            registry);
      factory.addState(WalkingStateEnum.TO_STANDING, toStandingState);
      factory.addState(WalkingStateEnum.STANDING, standingState);

      SideDependentList<TransferToWalkingSingleSupportState> walkingTransferStates = new SideDependentList<>();

      DoubleProvider minimumTransferTime = new DoubleParameter("MinimumTransferTime", registry, walkingControllerParameters.getMinimumTransferTime());
      DoubleProvider rhoMin = () -> controllerCoreOptimizationSettings.getRhoMin();
      for (RobotSide transferToSide : RobotSide.values)
      {
         WalkingStateEnum stateEnum = WalkingStateEnum.getWalkingTransferState(transferToSide);
         TransferToWalkingSingleSupportState transferState = new TransferToWalkingSingleSupportState(stateEnum,
                                                                                                     walkingMessageHandler,
                                                                                                     touchdownErrorCompensator,
                                                                                                     controllerToolbox,
                                                                                                     managerFactory,
                                                                                                     walkingControllerParameters,
                                                                                                     failureDetectionControlModule,
                                                                                                     minimumTransferTime,
                                                                                                     unloadFraction,
                                                                                                     rhoMin,
                                                                                                     registry);
         walkingTransferStates.put(transferToSide, transferState);
         factory.addState(stateEnum, transferState);
      }

      SideDependentList<WalkingSingleSupportState> walkingSingleSupportStates = new SideDependentList<>();

      for (RobotSide supportSide : RobotSide.values)
      {
         WalkingStateEnum stateEnum = WalkingStateEnum.getWalkingSingleSupportState(supportSide);
         WalkingSingleSupportState singleSupportState = new WalkingSingleSupportState(stateEnum,
                                                                                      walkingMessageHandler,
                                                                                      touchdownErrorCompensator,
                                                                                      controllerToolbox,
                                                                                      managerFactory,
                                                                                      walkingControllerParameters,
                                                                                      failureDetectionControlModule,
                                                                                      registry);
         walkingSingleSupportStates.put(supportSide, singleSupportState);
         factory.addState(stateEnum, singleSupportState);
      }

      SideDependentList<TransferToFlamingoStanceState> flamingoTransferStates = new SideDependentList<>();

      for (RobotSide transferToSide : RobotSide.values)
      {
         WalkingStateEnum stateEnum = WalkingStateEnum.getFlamingoTransferState(transferToSide);
         TransferToFlamingoStanceState transferState = new TransferToFlamingoStanceState(stateEnum,
                                                                                         walkingMessageHandler,
                                                                                         controllerToolbox,
                                                                                         managerFactory,
                                                                                         failureDetectionControlModule,
                                                                                         null,
                                                                                         rhoMin,
                                                                                         registry);
         flamingoTransferStates.put(transferToSide, transferState);
         factory.addState(stateEnum, transferState);
      }

      SideDependentList<FlamingoStanceState> flamingoSingleSupportStates = new SideDependentList<>();

      for (RobotSide supportSide : RobotSide.values)
      {
         WalkingStateEnum stateEnum = WalkingStateEnum.getFlamingoSingleSupportState(supportSide);
         FlamingoStanceState singleSupportState = new FlamingoStanceState(stateEnum,
                                                                          walkingMessageHandler,
                                                                          controllerToolbox,
                                                                          managerFactory,
                                                                          failureDetectionControlModule,
                                                                          registry);
         flamingoSingleSupportStates.put(supportSide, singleSupportState);
         factory.addState(stateEnum, singleSupportState);
      }

      factory.addDoneTransition(WalkingStateEnum.TO_STANDING, WalkingStateEnum.STANDING);

      // Setup start/stop walking conditions
      for (RobotSide robotSide : RobotSide.values)
      {
         TransferToWalkingSingleSupportState transferState = walkingTransferStates.get(robotSide);
         SingleSupportState singleSupportState = walkingSingleSupportStates.get(robotSide);

         WalkingStateEnum transferStateEnum = transferState.getStateEnum();
         WalkingStateEnum singleSupportStateEnum = singleSupportState.getStateEnum();

         // Start walking
         factory.addTransition(Arrays.asList(WalkingStateEnum.STANDING, WalkingStateEnum.TO_STANDING),
                               transferStateEnum,
                               new StartWalkingCondition(transferStateEnum.getTransferToSide(), walkingMessageHandler));

         // Stop walking when in transfer
         factory.addTransition(transferStateEnum, WalkingStateEnum.TO_STANDING, new StopWalkingFromTransferCondition(transferState, walkingMessageHandler));

         // Stop walking when in single support
         factory.addTransition(singleSupportStateEnum,
                               WalkingStateEnum.TO_STANDING,
                               new StopWalkingFromSingleSupportCondition(singleSupportState, walkingMessageHandler));
      }

      // Setup walking transfer to single support conditions
      for (RobotSide robotSide : RobotSide.values)
      {
         WalkingState transferState = walkingTransferStates.get(robotSide);
         WalkingState singleSupportState = walkingSingleSupportStates.get(robotSide);

         WalkingStateEnum transferStateEnum = transferState.getStateEnum();
         WalkingStateEnum singleSupportStateEnum = singleSupportState.getStateEnum();

         factory.addDoneTransition(transferStateEnum, singleSupportStateEnum);
      }

      // Setup walking single support to transfer conditions
      for (RobotSide robotSide : RobotSide.values)
      {
         SingleSupportState singleSupportState = walkingSingleSupportStates.get(robotSide);
         WalkingStateEnum singleSupportStateEnum = singleSupportState.getStateEnum();

         // Single support to transfer with same side
         {
            TransferToWalkingSingleSupportState transferState = walkingTransferStates.get(robotSide);
            WalkingStateEnum transferStateEnum = transferState.getStateEnum();

            factory.addTransition(singleSupportStateEnum,
                                  transferStateEnum,
                                  new SingleSupportToTransferToCondition(singleSupportState, transferState, walkingMessageHandler));
         }

         // Single support to transfer with opposite side
         {
            TransferToWalkingSingleSupportState transferState = walkingTransferStates.get(robotSide.getOppositeSide());
            WalkingStateEnum transferStateEnum = transferState.getStateEnum();

            factory.addTransition(singleSupportStateEnum,
                                  transferStateEnum,
                                  new SingleSupportToTransferToCondition(singleSupportState, transferState, walkingMessageHandler));
         }
      }

      // Setup start/stop flamingo conditions
      for (RobotSide robotSide : RobotSide.values)
      {
         TransferToFlamingoStanceState transferState = flamingoTransferStates.get(robotSide);
         FlamingoStanceState singleSupportState = flamingoSingleSupportStates.get(robotSide);

         WalkingStateEnum transferStateEnum = transferState.getStateEnum();
         WalkingStateEnum singleSupportStateEnum = singleSupportState.getStateEnum();

         // Start flamingo
         factory.addTransition(Arrays.asList(WalkingStateEnum.STANDING, WalkingStateEnum.TO_STANDING),
                               transferStateEnum,
                               new StartFlamingoCondition(transferState.getTransferToSide(), walkingMessageHandler));

         // Stop flamingo
         factory.addTransition(singleSupportStateEnum, WalkingStateEnum.TO_STANDING, new StopFlamingoCondition(singleSupportState, walkingMessageHandler));
      }

      // Setup tranfer to flamingo stance condition
      for (RobotSide robotSide : RobotSide.values)
      {
         TransferToFlamingoStanceState transferState = flamingoTransferStates.get(robotSide);
         FlamingoStanceState singleSupportState = flamingoSingleSupportStates.get(robotSide);

         WalkingStateEnum transferStateEnum = transferState.getStateEnum();
         WalkingStateEnum singleSupportStateEnum = singleSupportState.getStateEnum();

         factory.addDoneTransition(transferStateEnum, singleSupportStateEnum);
      }

      // Setup the abort condition from all states to the toStandingState
      Set<WalkingStateEnum> allButStandingStates = EnumSet.complementOf(EnumSet.of(WalkingStateEnum.STANDING, WalkingStateEnum.TO_STANDING));
      factory.addTransition(allButStandingStates, WalkingStateEnum.TO_STANDING, new AbortCondition());

      // Setup transition condition for push recovery, when recovering from double support.
      Set<WalkingStateEnum> allDoubleSupportStates = Stream.of(WalkingStateEnum.values).filter(state -> state.isDoubleSupport()).collect(Collectors.toSet());

      // Update the previous state info for each state using state changed listeners.
      factory.getRegisteredStates().forEach(state -> factory.addStateChangedListener((from, to) -> state.setPreviousWalkingStateEnum(from)));
      factory.addStateChangedListener((from, to) -> controllerToolbox.reportControllerStateChangeToListeners(from, to));

      return factory.build(WalkingStateEnum.TO_STANDING);
   }

   public void setControllerCoreOutput(ControllerCoreOutputReadOnly controllerCoreOutput)
   {
      this.controllerCoreOutput = controllerCoreOutput;
   }

   public void setLinearMomentumRateControlModuleOutput(LinearMomentumRateControlModuleOutput output)
   {
      balanceManager.setLinearMomentumRateControlModuleOutput(output);
   }

   public void setShouldKeepInitialContacts(boolean shouldKeepInitialContacts)
   {
      this.shouldKeepInitialContacts.set(shouldKeepInitialContacts);
   }

   public void initialize()
   {
      controllerCoreCommand.requestReinitialization();

      if (!shouldKeepInitialContacts.getAndSet(false))
      {
         // Contact states need to be initialized before controllerToolbox initialize, as it updates the support polygons stuff.
         initializeContacts();
      }

      controllerToolbox.initialize();
      managerFactory.initializeManagers();

      commandInputManager.clearAllCommands();
      walkingMessageHandler.clearFootsteps();
      walkingMessageHandler.clearFootTrajectory();
      
      updatePrivilegedConfigurationCommand();

      for (RobotSide robotSide : RobotSide.values)
      {
         footDesiredCoPs.get(robotSide).setToZero(feet.get(robotSide).getSoleFrame());
         controllerToolbox.setDesiredCenterOfPressure(feet.get(robotSide), footDesiredCoPs.get(robotSide));
      }

      for (WalkingStateEnum stateName : WalkingStateEnum.values)
      {
         if (stateMachine.getState(stateName) != null)
            stateMachine.getState(stateName).setPreviousWalkingStateEnum(null);
      }
      stateMachine.resetToInitialState(false);

      initializeManagers();

      commandConsumer.avoidManipulationAbortForDuration(RigidBodyControlManager.INITIAL_GO_HOME_TIME);

      firstTick = true;
   }

   
   private OneDoFJointPrivilegedConfigurationParameters spineRollPrivilegedConfigurationParameters()
   {
      OneDoFJointBasics spineRoll = fullRobotModel.getOneDoFJointByName("spineRoll");            

      OneDoFJointPrivilegedConfigurationParameters jointParameters = new OneDoFJointPrivilegedConfigurationParameters();
      jointParameters.setConfigurationGain(pPoseSpineRollKp.getValue());
      jointParameters.setVelocityGain(pPoseSpineRollKdFactor.getValue()*pPoseSpineRollKp.getValue());
      jointParameters.setWeight(1);
      jointParameters.setMaxAcceleration(Double.POSITIVE_INFINITY);
      jointParameters.setPrivilegedConfigurationOption(null);
      jointParameters.setPrivilegedConfiguration(pPoseSpineRoll.getDoubleValue());

      privilegedConfigurationCommand.addJoint(spineRoll, jointParameters);

      return jointParameters;
   }
   private OneDoFJointPrivilegedConfigurationParameters spinePitchPrivilegedConfigurationParameters()
   {
      OneDoFJointBasics spinePitch = fullRobotModel.getOneDoFJointByName("spinePitch");            

      OneDoFJointPrivilegedConfigurationParameters jointParameters = new OneDoFJointPrivilegedConfigurationParameters();
      jointParameters.setConfigurationGain(pPoseSpinePitchKp.getValue());
      jointParameters.setVelocityGain(pPoseSpinePitchKdFactor.getValue()*pPoseSpinePitchKp.getValue());
      jointParameters.setWeight(1);
      jointParameters.setMaxAcceleration(Double.POSITIVE_INFINITY);
      jointParameters.setPrivilegedConfigurationOption(null);
      jointParameters.setPrivilegedConfiguration(pPoseSpinePitch.getDoubleValue());

      privilegedConfigurationCommand.addJoint(spinePitch, jointParameters);

      return jointParameters;
   }
   private OneDoFJointPrivilegedConfigurationParameters spineYawPrivilegedConfigurationParameters()
   {
      OneDoFJointBasics spineYaw = fullRobotModel.getOneDoFJointByName("spineYaw");            

      OneDoFJointPrivilegedConfigurationParameters jointParameters = new OneDoFJointPrivilegedConfigurationParameters();
      jointParameters.setConfigurationGain(pPoseSpineYawKp.getValue());
      jointParameters.setVelocityGain(pPoseSpineYawKdFactor.getValue()*pPoseSpineYawKp.getValue());
      jointParameters.setWeight(1);
      jointParameters.setMaxAcceleration(Double.POSITIVE_INFINITY);
      jointParameters.setPrivilegedConfigurationOption(null);
      jointParameters.setPrivilegedConfiguration(pPoseSpineYaw.getDoubleValue());

      privilegedConfigurationCommand.addJoint(spineYaw, jointParameters);

      return jointParameters;
   }

   private OneDoFJointPrivilegedConfigurationParameters createAndAddJointPrivilegedConfigurationParameters(RobotSide robotSide, ArmJointName armJointName, double privilegedAngle)
   {
      OneDoFJointBasics armJoint = fullRobotModel.getArmJoint(robotSide, armJointName);            

      OneDoFJointPrivilegedConfigurationParameters jointParameters = new OneDoFJointPrivilegedConfigurationParameters();
      jointParameters.setConfigurationGain(40.0);//40.0);
      jointParameters.setVelocityGain(6.0);//6.0);
      jointParameters.setWeight(1);//5.0);
      jointParameters.setMaxAcceleration(Double.POSITIVE_INFINITY);
      jointParameters.setPrivilegedConfigurationOption(null);
      jointParameters.setPrivilegedConfiguration(privilegedAngle);

      privilegedConfigurationCommand.addJoint(armJoint, jointParameters);

      return jointParameters;
   }

   private OneDoFJointPrivilegedConfigurationParameters createAndAddJointPrivilegedConfigurationParameters(RobotSide robotSide, ArmJointName armJointName, 
                                                                                                           double privilegedAngle, 
                                                                                                           double pgain,
                                                                                                           double dgain)
   {
      OneDoFJointBasics armJoint = fullRobotModel.getArmJoint(robotSide, armJointName); 
      System.out.println(armJoint);

      OneDoFJointPrivilegedConfigurationParameters jointParameters = new OneDoFJointPrivilegedConfigurationParameters();
      jointParameters.setConfigurationGain(pgain);
      jointParameters.setVelocityGain(dgain);
      jointParameters.setWeight(1.0);
      jointParameters.setMaxAcceleration(Double.POSITIVE_INFINITY);
      jointParameters.setPrivilegedConfigurationOption(null);
      jointParameters.setPrivilegedConfiguration(privilegedAngle);

      privilegedConfigurationCommand.addJoint(armJoint, jointParameters);

      return jointParameters;
   }

   private OneDoFJointPrivilegedConfigurationParameters createAndAddJointPrivilegedConfigurationParameters(RobotSide robotSide, LegJointName legJointName, 
                                                                                                           double privilegedAngle, 
                                                                                                           double pgain,
                                                                                                           double dgain)
   {
      OneDoFJointBasics legJoint = fullRobotModel.getLegJoint(robotSide, legJointName);            

      OneDoFJointPrivilegedConfigurationParameters jointParameters = new OneDoFJointPrivilegedConfigurationParameters();
      jointParameters.setConfigurationGain(pgain);
      jointParameters.setVelocityGain(dgain);
      jointParameters.setWeight(1.0);
      jointParameters.setMaxAcceleration(Double.POSITIVE_INFINITY);
      jointParameters.setPrivilegedConfigurationOption(null);
      jointParameters.setPrivilegedConfiguration(privilegedAngle);

      privilegedConfigurationCommand.addJoint(legJoint, jointParameters);

      return jointParameters;
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
      //      naturalPostureManager.initialize();
      //      balanceManager.initialize();  // already initialized, so don't run it again, or else the state machine gets reset.
      feetManager.initialize();
      comHeightManager.initialize();
      feetManager.resetHeightCorrectionParametersForSingularityAvoidance();
   }

   /**
    * Request the controller to set all its desireds to match the current configuration of the robot.
    * <p>
    * Calling that method right after {@link #initialize()} will cause the controller to maintain the
    * current robot configuration.
    * </p>
    */
   public void requestImmediateTransitionToStandingAndHoldCurrent()
   {
      stateMachine.performTransition(WalkingStateEnum.STANDING);

      for (int managerIdx = 0; managerIdx < bodyManagers.size(); managerIdx++)
      {
         RigidBodyControlManager bodyManager = bodyManagers.get(managerIdx);
         if (bodyManager != null)
            bodyManager.hold();
      }

      pelvisOrientationManager.setToHoldCurrentInWorldFrame();
      comHeightManager.initializeDesiredHeightToCurrent();
      balanceManager.requestICPPlannerToHoldCurrentCoM();
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

   private final FrameVector2D desiredCoMVelocityAsFrameVector = new FrameVector2D();

   private final SideDependentList<FramePoint2D> footDesiredCoPs = new SideDependentList<FramePoint2D>(new FramePoint2D(), new FramePoint2D());
   private final RecyclingArrayList<PlaneContactStateCommand> planeContactStateCommandPool = new RecyclingArrayList<>(4, PlaneContactStateCommand.class);
   private final FramePoint2D capturePoint2d = new FramePoint2D();

   public void doAction()
   {
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
      commandConsumer.consumeEnvironmentalModelingCommands();
      commandConsumer.consumePrepareForLocomotionCommands();

      updateFailureDetection();

      walkingStateTimer.startMeasurement();
      // Do transitions will request ICP planner updates.
      if (!firstTick) // this avoids doing two transitions in a single tick if the initialize reset the state machine.
         stateMachine.doTransitions();
      // Do action is relying on the ICP plan being valid.
      stateMachine.doAction();
      walkingStateTimer.stopMeasurement();

      currentState = stateMachine.getCurrentState();

      managerUpdateTimer.startMeasurement();
      updateManagers(currentState);
      reportStatusMessages();
      managerUpdateTimer.stopMeasurement();

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

      if (ENABLE_LEG_ELASTICITY_DEBUGGATOR)
         legElasticityDebuggator.update();

      if (naturalPosture != null)
      {
         if (firstTick)
         {
            // use the built-in pose:
            naturalPosture.setNaturalPostureOffset(naturalPosture.getNominalStandingPoseQoffset());
         }

         naturalPosture.compute(fullRobotModel.getPelvis().getBodyFixedFrame().getTransformToWorldFrame().getRotation());

         firstTick = false;
      }
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

         boolean isInSwing = stateMachine.getCurrentStateKey() == WalkingStateEnum.WALKING_LEFT_SUPPORT
               || stateMachine.getCurrentStateKey() == WalkingStateEnum.WALKING_RIGHT_SUPPORT;
         if (enablePushRecoveryOnFailure.getBooleanValue() && !isInSwing)
         {
            commandInputManager.submitMessage(HumanoidMessageTools.createHighLevelStateMessage(HighLevelControllerName.PUSH_RECOVERY));
         }
         else
         {
            walkingMessageHandler.reportControllerFailure(failureDetectionControlModule.getFallingDirection3D());
            controllerToolbox.reportControllerFailureToListeners(failureDetectionControlModule.getFallingDirection2D());
         }
      }
   }

   public void updateManagers(WalkingState currentState)
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
      if (naturalPostureManager != null)
         naturalPostureManager.compute();

      comHeightManager.compute(balanceManager.getDesiredICPVelocity(),
                               desiredCoMVelocityAsFrameVector,
                               isInDoubleSupport,
                               omega0,
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
      if (turnOnNaturalPostureControl.getValue())
      {
         useNaturalPostureCommand.set(true);
         usePelvisPrivilegedPoseCommand.set(true);
         usePelvisOrientationCommand.set(false);
         useBodyManagerCommands.set(false);
//         comHeightManager.setControlHeightWithMomentum(false);  //TODO(GMN): Why is this here?
      }
      
      planeContactStateCommandPool.clear();
      
      // Privileged configuration:
      updatePrivilegedConfigurationCommand();
      controllerCoreCommand.addInverseDynamicsCommand(privilegedConfigurationCommand);
      
      // Joint limits:
      if (!limitCommandSent.getBooleanValue())
      {
         controllerCoreCommand.addInverseDynamicsCommand(jointLimitEnforcementMethodCommand);
         limitCommandSent.set(true);
      }

      boolean isHighCoPDampingNeeded = controllerToolbox.estimateIfHighCoPDampingNeeded(footDesiredCoPs);

      // Foot control:
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

      // Body managers:
      if (useBodyManagerCommands.getValue())
      {
         for (int managerIdx = 0; managerIdx < bodyManagers.size(); managerIdx++)
         {
            RigidBodyControlManager bodyManager = bodyManagers.get(managerIdx);
            if (bodyManager != null)
            {
               controllerCoreCommand.addFeedbackControlCommand(bodyManager.getFeedbackControlCommand());
               controllerCoreCommand.addInverseDynamicsCommand(bodyManager.getInverseDynamicsCommand());
            }
         }
      }
      
      // Natural posture:
      if ((naturalPostureManager != null) && (useNaturalPostureCommand.getValue()))
      {
         controllerCoreCommand.addInverseDynamicsCommand(naturalPostureManager.getQPObjectiveCommand());
      }
      
      // Privileged pelvis control:
      if ((naturalPostureManager != null) && (usePelvisPrivilegedPoseCommand.getValue()))
      {
         controllerCoreCommand.addInverseDynamicsCommand(naturalPostureManager.getPelvisPrivilegedPoseCommand());
      }
      
      // Higher-level pelvis control:
      if (usePelvisOrientationCommand.getValue())
      {
         controllerCoreCommand.addFeedbackControlCommand(pelvisOrientationManager.getFeedbackControlCommand());
         //      controllerCoreCommand.addInverseDynamicsCommand(pelvisOrientationManager.getInverseDynamicsCommand());
      }
      
      // CoM height control:
      controllerCoreCommand.addFeedbackControlCommand(comHeightManager.getFeedbackControlCommand());

      controllerCoreCommand.addInverseDynamicsCommand(controllerCoreOptimizationSettings.getCommand());

      if (ENABLE_LEG_ELASTICITY_DEBUGGATOR)
         controllerCoreCommand.addInverseDynamicsCommand(legElasticityDebuggator.getInverseDynamicsCommand());
   }

   private void updatePrivilegedConfigurationCommand()
   {
      privilegedConfigurationCommand.clear();
      privilegedConfigurationCommand.enable();
      privilegedConfigurationCommand.setPrivilegedConfigurationOption(PrivilegedConfigurationOption.AT_ZERO);

      //TODO: This is hardcoded here. It should be moved to a parameter setting instead. This is not the long term place for it.
      spineRollPrivilegedConfigurationParameters();
      if (useSpinePitchPrivilegedCommand)
         spinePitchPrivilegedConfigurationParameters();
      spineYawPrivilegedConfigurationParameters();
      
      RobotSide side = RobotSide.LEFT;
      createAndAddJointPrivilegedConfigurationParameters(side, ArmJointName.SHOULDER_PITCH, 
                                                         pPoseShoulderPitch.getDoubleValue(), 
                                                         pPoseShoulderPitchKp.getDoubleValue(), 
                                                         pPoseShoulderPitchKdFactor.getDoubleValue()*pPoseShoulderPitchKp.getDoubleValue());
      createAndAddJointPrivilegedConfigurationParameters(side, ArmJointName.SHOULDER_ROLL, 
                                                         pPoseShoulderRoll.getDoubleValue(), 
                                                         pPoseShoulderRollKp.getDoubleValue(), 
                                                         pPoseShoulderRollKdFactor.getDoubleValue()*pPoseShoulderRollKp.getDoubleValue());
      createAndAddJointPrivilegedConfigurationParameters(side, ArmJointName.SHOULDER_YAW, 
                                                         pPoseShoulderYaw.getDoubleValue(), 
                                                         pPoseShoulderYawKp.getDoubleValue(), 
                                                         pPoseShoulderYawKdFactor.getDoubleValue()*pPoseShoulderYawKp.getDoubleValue());
      createAndAddJointPrivilegedConfigurationParameters(side, ArmJointName.ELBOW_PITCH, 
                                                         pPoseElbow.getDoubleValue(), 
                                                         pPoseElbowKp.getDoubleValue(), 
                                                         pPoseElbowKdFactor.getDoubleValue()*pPoseElbowKp.getDoubleValue());

      side = RobotSide.RIGHT;
      createAndAddJointPrivilegedConfigurationParameters(side, ArmJointName.SHOULDER_PITCH, 
                                                         -pPoseShoulderPitch.getDoubleValue(), 
                                                         pPoseShoulderPitchKp.getDoubleValue(), 
                                                         pPoseShoulderPitchKdFactor.getDoubleValue()*pPoseShoulderPitchKp.getDoubleValue());
      createAndAddJointPrivilegedConfigurationParameters(side, ArmJointName.SHOULDER_ROLL, 
                                                         pPoseShoulderRoll.getDoubleValue(), 
                                                         pPoseShoulderRollKp.getDoubleValue(), 
                                                         pPoseShoulderRollKdFactor.getDoubleValue()*pPoseShoulderRollKp.getDoubleValue());
      createAndAddJointPrivilegedConfigurationParameters(side, ArmJointName.SHOULDER_YAW, 
                                                         pPoseShoulderYaw.getDoubleValue(), 
                                                         pPoseShoulderYawKp.getDoubleValue(), 
                                                         pPoseShoulderYawKdFactor.getDoubleValue()*pPoseShoulderYawKp.getDoubleValue());
      createAndAddJointPrivilegedConfigurationParameters(side, ArmJointName.ELBOW_PITCH, 
                                                         -pPoseElbow.getDoubleValue(), 
                                                         pPoseElbowKp.getDoubleValue(), 
                                                         pPoseElbowKdFactor.getDoubleValue()*pPoseElbowKp.getDoubleValue());

      createAndAddJointPrivilegedConfigurationParameters(RobotSide.LEFT, ArmJointName.WRIST_YAW, 0.0);
      createAndAddJointPrivilegedConfigurationParameters(RobotSide.RIGHT, ArmJointName.WRIST_YAW, 0.0);
      createAndAddJointPrivilegedConfigurationParameters(RobotSide.LEFT, ArmJointName.WRIST_ROLL, 0.0);
      createAndAddJointPrivilegedConfigurationParameters(RobotSide.RIGHT, ArmJointName.WRIST_ROLL, 0.0);
      createAndAddJointPrivilegedConfigurationParameters(RobotSide.LEFT, ArmJointName.FIRST_WRIST_PITCH, 0.0);
      createAndAddJointPrivilegedConfigurationParameters(RobotSide.RIGHT, ArmJointName.FIRST_WRIST_PITCH, 0.0);

      // GMN: Probably should have privileged configuration set for entire robot
//      for (RobotSide robotSide : RobotSide.values)
//      {
//         OneDoFJointBasics kneeJoint = fullRobotModel.getLegJoint(robotSide, LegJointName.KNEE_PITCH);
//
//         privilegedConfigurationCommand.addJoint(kneeJoint, walkingControllerParameters.getKneePrivilegedConfigurationParameters());
//      }
      
//      side = RobotSide.LEFT;
//      createAndAddJointPrivilegedConfigurationParameters(side, LegJointName.HIP_PITCH, 
//                                                         pPoseHipPitch.getDoubleValue(), 
//                                                         pPoseHipPitchKp.getDoubleValue(), 
//                                                         pPoseHipPitchKdFactor.getDoubleValue()*pPoseHipPitchKp.getDoubleValue());
//      createAndAddJointPrivilegedConfigurationParameters(side, LegJointName.HIP_ROLL, 
//                                                         pPoseHipRoll.getDoubleValue(), 
//                                                         pPoseHipRollKp.getDoubleValue(), 
//                                                         pPoseHipRollKdFactor.getDoubleValue()*pPoseHipRollKp.getDoubleValue());
//      createAndAddJointPrivilegedConfigurationParameters(side, LegJointName.HIP_YAW, 
//                                                         pPoseHipYaw.getDoubleValue(), 
//                                                         pPoseHipYawKp.getDoubleValue(), 
//                                                         pPoseHipYawKdFactor.getDoubleValue()*pPoseHipYawKp.getDoubleValue());
//      createAndAddJointPrivilegedConfigurationParameters(side, LegJointName.KNEE_PITCH, 
//                                                         pPoseKnee.getDoubleValue(), 
//                                                         pPoseKneeKp.getDoubleValue(), 
//                                                         pPoseKneeKdFactor.getDoubleValue()*pPoseKneeKp.getDoubleValue());
//
//      side = RobotSide.RIGHT;
//      createAndAddJointPrivilegedConfigurationParameters(side, LegJointName.HIP_PITCH, 
//                                                         -pPoseHipPitch.getDoubleValue(), 
//                                                         pPoseHipPitchKp.getDoubleValue(), 
//                                                         pPoseHipPitchKdFactor.getDoubleValue()*pPoseHipPitchKp.getDoubleValue());
//      createAndAddJointPrivilegedConfigurationParameters(side, LegJointName.HIP_ROLL, 
//                                                         -pPoseHipRoll.getDoubleValue(), 
//                                                         pPoseHipRollKp.getDoubleValue(), 
//                                                         pPoseHipRollKdFactor.getDoubleValue()*pPoseHipRollKp.getDoubleValue());
//      createAndAddJointPrivilegedConfigurationParameters(side, LegJointName.HIP_YAW, 
//                                                         -pPoseHipYaw.getDoubleValue(), 
//                                                         pPoseHipYawKp.getDoubleValue(), 
//                                                         pPoseHipYawKdFactor.getDoubleValue()*pPoseHipYawKp.getDoubleValue());
//      createAndAddJointPrivilegedConfigurationParameters(side, LegJointName.KNEE_PITCH, 
//                                                         -pPoseKnee.getDoubleValue(), 
//                                                         pPoseKneeKp.getDoubleValue(), 
//                                                         pPoseKneeKdFactor.getDoubleValue()*pPoseKneeKp.getDoubleValue());
      
      for (RobotSide robotSide : RobotSide.values)
      {
         createAndAddJointPrivilegedConfigurationParameters(robotSide,
                                                            LegJointName.HIP_PITCH,
                                                            -0.25,
                                                            pPoseHipPitchKp.getDoubleValue(),
                                                            pPoseHipPitchKdFactor.getDoubleValue() * pPoseHipPitchKp.getDoubleValue());
         createAndAddJointPrivilegedConfigurationParameters(robotSide,
                                                            LegJointName.HIP_ROLL,
                                                            0.0,
                                                            pPoseHipRollKp.getDoubleValue(),
                                                            pPoseHipRollKdFactor.getDoubleValue() * pPoseHipRollKp.getDoubleValue());
         createAndAddJointPrivilegedConfigurationParameters(robotSide,
                                                            LegJointName.HIP_YAW,
                                                            0.0,
                                                            pPoseHipYawKp.getDoubleValue(),
                                                            pPoseHipYawKdFactor.getDoubleValue() * pPoseHipYawKp.getDoubleValue());
         createAndAddJointPrivilegedConfigurationParameters(robotSide,
                                                            LegJointName.KNEE_PITCH,
                                                            0.5,
                                                            pPoseKneeKp.getDoubleValue(),
                                                            pPoseKneeKdFactor.getDoubleValue() * pPoseKneeKp.getDoubleValue());
      }
      
      createAndAddJointPrivilegedConfigurationParameters(RobotSide.LEFT, LegJointName.ANKLE_ROLL, 0.0, 4.0, 0.6);
      createAndAddJointPrivilegedConfigurationParameters(RobotSide.RIGHT, LegJointName.ANKLE_ROLL, 0.0, 4.0, 0.6);
      createAndAddJointPrivilegedConfigurationParameters(RobotSide.LEFT, LegJointName.ANKLE_PITCH, 0.0, 4.0, 0.6);
      createAndAddJointPrivilegedConfigurationParameters(RobotSide.RIGHT, LegJointName.ANKLE_PITCH, 0.0, 4.0, 0.6);
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
   public WalkingStateEnum getWalkingStateEnum()
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
