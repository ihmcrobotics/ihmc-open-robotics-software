package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.capturePoint.BalanceManager;
import us.ihmc.commonWalkingControlModules.capturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.capturePoint.LinearMomentumRateControlModuleInput;
import us.ihmc.commonWalkingControlModules.capturePoint.LinearMomentumRateControlModuleOutput;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.naturalPosture.NaturalPostureManager;
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
import us.ihmc.commonWalkingControlModules.staticEquilibrium.WholeBodyContactState;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
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
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class WalkingHighLevelHumanoidController implements JointLoadStatusProvider, SCS2YoGraphicHolder
{
   private static final boolean ENABLE_LEG_ELASTICITY_DEBUGGATOR = false;
   private static final boolean CONSTRAIN_COP_WITH_MULTI_CONTACT_STABILITY_REGION = true;

   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);

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

   private final YoBoolean isUpperBodyLoadBearing = new YoBoolean("isUpperBodyLoadBearing", registry);

   private final PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);
   private ControllerCoreOutputReadOnly controllerCoreOutput;

   private final DoubleProvider unloadFraction;

   private final ParameterizedControllerCoreOptimizationSettings controllerCoreOptimizationSettings;

   private final ExecutionTimer walkingStateTimer = new ExecutionTimer("walkingStateTimer", registry);
   private final ExecutionTimer managerUpdateTimer = new ExecutionTimer("managerUpdateTimer", registry);
   private final YoBoolean enableHeightFeedbackControl = new YoBoolean("enableHeightFeedbackControl", registry);

   private boolean firstTick = true;

   public WalkingHighLevelHumanoidController(CommandInputManager commandInputManager,
                                             StatusMessageOutputManager statusOutputManager,
                                             HighLevelControlManagerFactory managerFactory,
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
      this.naturalPostureManager = managerFactory.getOrCreateNaturalPostureManager();
      this.feetManager = managerFactory.getOrCreateFeetManager();

      RigidBodyBasics head = fullRobotModel.getHead();
      RigidBodyBasics chest = fullRobotModel.getChest();
      RigidBodyBasics pelvis = fullRobotModel.getPelvis();

      pelvisStatusMessage.setEndEffectorName(pelvis.getName());

      unloadFraction = walkingControllerParameters.enforceSmoothFootUnloading() != null ? new DoubleParameter("unloadFraction", registry, 0.5) : null;

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
         Arrays.stream(manager.getControlledJoints()).forEach(joint -> bodyManagerByJointName.put(joint.getName(), manager));
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

      touchdownErrorCompensator = new TouchdownErrorCompensator(walkingMessageHandler, controllerToolbox.getContactableFeet(), registry);
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
         /*
          This guy can be used to add shearing forces to the feet while measuring the distance between them.
          Really useful to debug and identify elasticity in the legs.
         */
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
                                                                          walkingControllerParameters,
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

      for (int i = 0; i < bodyManagers.size(); i++)
      {
         if (bodyManagers.get(i) == null)
            continue;

         // Controller core output informs load-bearing state of load status
         bodyManagers.get(i).setControllerCoreOutput(controllerCoreOutput);
      }
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
      commandConsumer.clearAllCommands();
      walkingMessageHandler.clearFootsteps();
      walkingMessageHandler.clearFlamingoCommands();

      if (naturalPostureManager == null || !naturalPostureManager.isEnabled())
      {
         privilegedConfigurationCommand.clear();
         privilegedConfigurationCommand.setPrivilegedConfigurationOption(PrivilegedConfigurationOption.AT_ZERO);

         for (RobotSide robotSide : RobotSide.values)
         {
            ArmJointName[] armJointNames = fullRobotModel.getRobotSpecificJointNames().getArmJointNames();
            for (int i = 0; i < armJointNames.length; i++)
               privilegedConfigurationCommand.addJoint(fullRobotModel.getArmJoint(robotSide, armJointNames[i]), PrivilegedConfigurationOption.AT_MID_RANGE);

            OneDoFJointBasics kneeJoint = fullRobotModel.getLegJoint(robotSide, LegJointName.KNEE_PITCH);

            privilegedConfigurationCommand.addJoint(kneeJoint, walkingControllerParameters.getKneePrivilegedConfigurationParameters());
         }
      }

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
      // balanceManager.initialize(); // already initialized, so don't run it again or else the state machine gets reset.
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
      commandConsumer.consumeChestCommands(currentState, allowUpperBodyMotionDuringLocomotion.getBooleanValue());
      commandConsumer.consumePelvisHeightCommands(currentState, allowUpperBodyMotionDuringLocomotion.getBooleanValue());
      commandConsumer.consumeGoHomeMessages();
      commandConsumer.consumeFootLoadBearingCommands(currentState);
      commandConsumer.consumeStopAllTrajectoryCommands();
      commandConsumer.consumeFootCommands();
      commandConsumer.consumeAbortWalkingCommands(abortWalkingRequested);
      commandConsumer.consumePelvisCommands(currentState, allowUpperBodyMotionDuringLocomotion.getBooleanValue());
      commandConsumer.consumeManipulationCommands(currentState, allowUpperBodyMotionDuringLocomotion.getBooleanValue());
      commandConsumer.handleAutomaticManipulationAbortOnICPError(currentState);
      commandConsumer.consumeLoadBearingCommands();
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

      currentState.handleChangeInContactState();

      submitControllerCoreCommands(currentState);

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

      updateAndPublishFootstepQueueStatus();

      CapturabilityBasedStatus capturabilityBasedStatus = balanceManager.updateAndReturnCapturabilityBasedStatus();
      packHandLoadBearingStatuses(capturabilityBasedStatus);
      statusOutputManager.reportStatusMessage(capturabilityBasedStatus);

      if (ENABLE_LEG_ELASTICITY_DEBUGGATOR)
         legElasticityDebuggator.update();

      firstTick = false;
   }

   private void updateAndPublishFootstepQueueStatus()
   {
      Footstep footstepBeingExecuted = null;
      FootstepTiming footstepTimingBeingExecuted = null;
      List<StepConstraintRegion> stepConstraintsBeingExecuted = null;
      boolean isFirstStepInSwing = false;

      WalkingStateEnum currentStateKey = stateMachine.getCurrentStateKey();
      // WHen we are currently in swing, that footstep has been removed from the walking message handler. We must then get it from the balance manager.
      if (currentStateKey == WalkingStateEnum.WALKING_RIGHT_SUPPORT || currentStateKey == WalkingStateEnum.WALKING_LEFT_SUPPORT)
      {
         footstepBeingExecuted = balanceManager.getFootstep(0);
         footstepTimingBeingExecuted = balanceManager.getFootstepTiming(0);
         stepConstraintsBeingExecuted = balanceManager.getCurrentStepConstraints();
         isFirstStepInSwing = true;
      }

      statusOutputManager.reportStatusMessage(walkingMessageHandler.updateAndReturnFootstepQueueStatus(footstepBeingExecuted,
                                                                                                       footstepTimingBeingExecuted,
                                                                                                       stepConstraintsBeingExecuted,
                                                                                                       balanceManager.getTimeIntoCurrentSupportSequence(),
                                                                                                       isFirstStepInSwing));
   }

   public void updateFailureDetection()
   {
      capturePoint2d.setIncludingFrame(balanceManager.getCapturePoint());
      failureDetectionControlModule.checkIfRobotIsFalling(capturePoint2d, balanceManager.getDesiredICP());

      if (failureDetectionControlModule.isRobotFalling())
      {
         walkingMessageHandler.clearFootsteps();
         walkingMessageHandler.clearFlamingoCommands();

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

      boolean isUpperBodyLoadBearing = false;
      FrameConvexPolygon2DReadOnly multiContactStabilityRegion = null;

      for (int managerIdx = 0; managerIdx < bodyManagers.size(); managerIdx++)
      {
         RigidBodyControlManager bodyManager = bodyManagers.get(managerIdx);
         if (bodyManager != null)
         {
            bodyManager.compute();

            if (bodyManager.isLoadBearing())
               isUpperBodyLoadBearing = true;
         }
      }

      this.isUpperBodyLoadBearing.set(isUpperBodyLoadBearing);

      if (isUpperBodyLoadBearing)
      {
         updateWholeBodyContactState();
         controllerToolbox.updateMultiContactCoMRegion();
         if (CONSTRAIN_COP_WITH_MULTI_CONTACT_STABILITY_REGION && controllerToolbox.getMultiContactRegionCalculator().hasSolvedWholeRegion())
            multiContactStabilityRegion = controllerToolbox.getMultiContactRegionCalculator().getFeasibleCoMRegion();
      }
      else
      {
         controllerToolbox.getMultiContactRegionCalculator().clear();
      }

      pelvisOrientationManager.compute();
      if (naturalPostureManager != null && naturalPostureManager.isEnabled())
      {
         naturalPostureManager.compute();
      }

      comHeightManager.compute(balanceManager.getDesiredICPVelocity(), desiredCoMVelocityAsFrameVector, isInDoubleSupport, omega0, feetManager);
      FeedbackControlCommand<?> heightControlCommand = comHeightManager.getHeightControlCommand();

      /*
       * The comHeightManager can control the pelvis with a feedback controller and doesn't always need
       * the z component of the momentum command. It would be better to remove the coupling between these
       * two modules.
       */
      boolean controlHeightWithMomentum = comHeightManager.getControlHeightWithMomentum() && enableHeightFeedbackControl.getValue();
      RobotSide supportLeg = currentState.isDoubleSupportState() ? currentState.getTransferToSide() : currentState.getSupportSide();
      balanceManager.compute(supportLeg, heightControlCommand, multiContactStabilityRegion, controlHeightWithMomentum);
   }

   private void updateWholeBodyContactState()
   {
      if (!hasContactStateChanged())
      {
         return;
      }

      WholeBodyContactState wholeBodyContactState = controllerToolbox.getWholeBodyContactState();
      wholeBodyContactState.clear();

      // Set feet contact points
      for (RobotSide robotSide : RobotSide.values)
         wholeBodyContactState.addContactPoints(controllerToolbox.getFootContactState(robotSide));

      // Set upper body contact points
      for (int i = 0; i < bodyManagers.size(); i++)
         bodyManagers.get(i).updateWholeBodyContactState(wholeBodyContactState);

      wholeBodyContactState.updateContactPoints();
      wholeBodyContactState.updateJointIndices();

      controllerToolbox.onWholeBodyContactsChanged();
   }

   private void packHandLoadBearingStatuses(CapturabilityBasedStatus capturabilityBasedStatus)
   {
      packBodyLoadStatus(fullRobotModel.getHand(RobotSide.LEFT), capturabilityBasedStatus.getLeftHandContactPoints(), capturabilityBasedStatus.getLeftHandContactNormal());
      packBodyLoadStatus(fullRobotModel.getHand(RobotSide.RIGHT), capturabilityBasedStatus.getRightHandContactPoints(), capturabilityBasedStatus.getRightHandContactNormal());
   }

   private void packBodyLoadStatus(RigidBodyBasics rigidBody, RecyclingArrayList<Point3D> contactPointList, Vector3D contactNormalToPack)
   {
      contactPointList.clear();
      contactNormalToPack.setToZero();

      for (int i = 0; i < bodyManagers.size(); i++)
      {
         if (bodyManagers.get(i).getBodyToControl() == rigidBody)
         {
            bodyManagers.get(i).packContactData(contactPointList, contactNormalToPack);
         }
      }
   }

   private boolean hasContactStateChanged()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         // peek because other parts of the controller (WalkingSingleSupportState#handleChangeInContactState) use the notification.
         // it is polled in this class at the end of doAction
         if (controllerToolbox.getFootContactState(robotSide).peekContactHasChangedNotification())
            return true;
      }
      for (int i = 0; i < bodyManagers.size(); i++)
      {
         // poll because this is the only class that listens to the notification
         if (bodyManagers.get(i).pollContactHasChangedNotification())
            return true;
      }
      return false;
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
            statusMessage = bodyManager.pollWrenchStatusToReport();
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

   private void submitControllerCoreCommands(WalkingState currentState)
   {
      planeContactStateCommandPool.clear();

      controllerCoreCommand.addInverseDynamicsCommand(currentState.getInverseDynamicsCommand());

      if (naturalPostureManager != null && naturalPostureManager.isEnabled())
      {
         //TODO could this be cleaner?
         controllerCoreCommand.addInverseDynamicsCommand(naturalPostureManager.getPrivilegedConfigurationController()
                                                                              .getInverseDynamicsCommand());
         controllerCoreCommand.addFeedbackControlCommand(naturalPostureManager.getPrivilegedConfigurationController()
                                                                              .getFeedbackControlCommand());
      }
      else
      {
         controllerCoreCommand.addInverseDynamicsCommand(privilegedConfigurationCommand);
      }

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
      if (naturalPostureManager == null || naturalPostureManager.getUseBodyManagerCommands())
      {
         for (int managerIdx = 0; managerIdx < bodyManagers.size(); managerIdx++)
         {
            RigidBodyControlManager bodyManager = bodyManagers.get(managerIdx);
            if (bodyManager != null)
            {
               controllerCoreCommand.addFeedbackControlCommand(bodyManager.getFeedbackControlCommand());
               controllerCoreCommand.addInverseDynamicsCommand(bodyManager.getInverseDynamicsCommand());

               JointDesiredOutputListReadOnly jointDesiredData = bodyManager.getJointDesiredData();
               if (jointDesiredData != null)
               {
                  controllerCoreCommand.completeLowLevelJointData(jointDesiredData);
               }
            }
         }
      }

      // Natural posture:
      if (naturalPostureManager != null && naturalPostureManager.isEnabled())
      {
         controllerCoreCommand.addInverseDynamicsCommand(naturalPostureManager.getQPObjectiveCommand());
         controllerCoreCommand.addInverseDynamicsCommand(naturalPostureManager.getJointLimitEnforcementCommand());
         limitCommandSent.set(false);
      }

      // Privileged pelvis control:
      if (naturalPostureManager != null && naturalPostureManager.getUsePelvisPrivilegedPoseCommand())
      {
         controllerCoreCommand.addInverseDynamicsCommand(naturalPostureManager.getPelvisPrivilegedPoseCommand());
      }

      // Higher-level pelvis control:
      if (naturalPostureManager == null || naturalPostureManager.getUsePelvisOrientationCommand())
      {
         controllerCoreCommand.addFeedbackControlCommand(pelvisOrientationManager.getFeedbackControlCommand());
      }

      // CoM height control:
      controllerCoreCommand.addFeedbackControlCommand(comHeightManager.getFeedbackControlCommand());

      controllerCoreCommand.addInverseDynamicsCommand(controllerCoreOptimizationSettings.getCommand());

      if (ENABLE_LEG_ELASTICITY_DEBUGGATOR)
         controllerCoreCommand.addInverseDynamicsCommand(legElasticityDebuggator.getInverseDynamicsCommand());
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
         boolean legLoaded = feetManager.getCurrentControlState(robotSide).isLoadBearing();
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

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      return null;
   }

   public BalanceManager getBalanceManager()
   {
      return balanceManager;
   }
}
