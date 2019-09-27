package us.ihmc.avatar.kinematicsSimulation;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxHelper;
import us.ihmc.commonWalkingControlModules.capturePoint.BalanceManager;
import us.ihmc.commonWalkingControlModules.capturePoint.LinearMomentumRateControlModule;
import us.ihmc.commonWalkingControlModules.configurations.ICPPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisHeightControlState;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.SettableFootSwitch;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.MessageUnpackingTools;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemStateIntegrator;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachineClock;
import us.ihmc.robotics.taskExecutor.StateExecutor;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.functional.FunctionalTools;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;
import us.ihmc.wholeBodyController.DRCControllerThread;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.parameters.ParameterLoaderHelper;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;

public class AvatarKinematicsSimulation
{
   private static final double DT = UnitConversions.hertzToSeconds(70);
   public static final double PLAYBACK_SPEED = 10.0;
   private final DRCRobotModel robotModel;
   private final ExceptionHandlingThreadScheduler scheduler = new ExceptionHandlingThreadScheduler(getClass().getSimpleName(),
                                                                                                   DefaultExceptionHandler.PRINT_MESSAGE,
                                                                                                   5);
   private final ExceptionHandlingThreadScheduler yoVariableScheduler = new ExceptionHandlingThreadScheduler(getClass().getSimpleName(),
                                                                                                             DefaultExceptionHandler.PRINT_MESSAGE,
                                                                                                             5);
   private final Ros2Node ros2Node;
   private final IHMCROS2Publisher<RobotConfigurationData> robotConfigurationDataPublisher;
   private final IHMCROS2Publisher<CapturabilityBasedStatus> capturabilityBasedStatusPublisher;
   private final IHMCROS2Publisher<WalkingStatusMessage> walkingStatusPublisher;
   private final IHMCROS2Publisher<FootstepStatusMessage> footstepStatusPublisher;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private double yoVariableServerTime = 0.0;
   private YoVariableServer yoVariableServer;
   private ScheduledFuture<?> yoVariableServerScheduled;

   private final double gravityZ = 9.81;
   private final YoDouble yoTime;

   private final FloatingJointBasics rootJoint;
   private final FullHumanoidRobotModel fullRobotModel;
   private final CommonHumanoidReferenceFrames referenceFrames;
   private final SideDependentList<SettableFootSwitch> footSwitches = new SideDependentList<>();

   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WholeBodyControllerCore controllerCore;
   private final LinearMomentumRateControlModule linearMomentumRateControlModule;
   private final HighLevelControlManagerFactory managerFactory;
   private final WalkingHighLevelHumanoidController walkingController;

   private final CommandInputManager walkingInputManager = new CommandInputManager("walking_preview_internal",
                                                                                   ControllerAPIDefinition.getControllerSupportedCommands());
   private final StatusMessageOutputManager walkingOutputManager = new StatusMessageOutputManager(
         ControllerAPIDefinition.getControllerSupportedStatusMessages());

   private final StateExecutor taskExecutor = new StateExecutor(StateMachineClock.dummyClock()); // should be dummy?
   private final MultiBodySystemStateIntegrator integrator = new MultiBodySystemStateIntegrator();

   private CapturabilityBasedStatus capturabilityBasedStatus = new CapturabilityBasedStatus();
   private BalanceManager balanceManager;

   public static void createForManualTest(DRCRobotModel robotModel, boolean createYoVariableServer)
   {
      create(robotModel, createYoVariableServer, PubSubImplementation.FAST_RTPS);
   }

   public static void createForAutomatedTest(DRCRobotModel robotModel, boolean createYoVariableServer)
   {
      create(robotModel, createYoVariableServer, PubSubImplementation.INTRAPROCESS);
   }

   private static void create(DRCRobotModel robotModel, boolean createYoVariableServer, PubSubImplementation pubSubImplementation)
   {
      new AvatarKinematicsSimulation(robotModel, createYoVariableServer, pubSubImplementation);
   }

   public AvatarKinematicsSimulation(DRCRobotModel robotModel, boolean createYoVariableServer, PubSubImplementation pubSubImplementation)
   {
      this.robotModel = robotModel;

      // instantiate some existing controller ROS2 API?
      ros2Node = ROS2Tools.createRos2Node(pubSubImplementation, ROS2Tools.HUMANOID_CONTROLLER.getNodeName("kinematic"));

      robotConfigurationDataPublisher = new IHMCROS2Publisher<>(ros2Node,
                                                                RobotConfigurationData.class,
                                                                robotModel.getSimpleRobotName(),
                                                                ROS2Tools.HUMANOID_CONTROLLER);
      capturabilityBasedStatusPublisher = new IHMCROS2Publisher<>(ros2Node,
                                                                  CapturabilityBasedStatus.class,
                                                                  robotModel.getSimpleRobotName(),
                                                                  ROS2Tools.HUMANOID_CONTROLLER);
      walkingStatusPublisher = new IHMCROS2Publisher<>(ros2Node,
                                                       WalkingStatusMessage.class,
                                                       robotModel.getSimpleRobotName(),
                                                       ROS2Tools.HUMANOID_CONTROLLER);
      footstepStatusPublisher = new IHMCROS2Publisher<>(ros2Node,
                                                        FootstepStatusMessage.class,
                                                        robotModel.getSimpleRobotName(),
                                                        ROS2Tools.HUMANOID_CONTROLLER);

      DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.0, 0.0);

      String robotName = robotModel.getSimpleRobotName();
      fullRobotModel = robotModel.createFullRobotModel();
      referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      yoTime = new YoDouble("timeInPreview", registry);

      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      ICPWithTimeFreezingPlannerParameters capturePointPlannerParameters = robotModel.getCapturePointPlannerParameters();

      // Create registries to match controller so the XML gets loaded properly.
      YoVariableRegistry drcControllerThreadRegistry = new YoVariableRegistry("DRCControllerThread");
      YoVariableRegistry drcMomentumBasedControllerRegistry = new YoVariableRegistry("DRCMomentumBasedController");
      YoVariableRegistry humanoidHighLevelControllerManagerRegistry = new YoVariableRegistry("HumanoidHighLevelControllerManager");
      YoVariableRegistry managerParentRegistry = new YoVariableRegistry("HighLevelHumanoidControllerFactory");
      YoVariableRegistry walkingParentRegistry = new YoVariableRegistry("WalkingControllerState");
      registry.addChild(drcControllerThreadRegistry);
      drcControllerThreadRegistry.addChild(drcMomentumBasedControllerRegistry);
      drcMomentumBasedControllerRegistry.addChild(humanoidHighLevelControllerManagerRegistry);
      humanoidHighLevelControllerManagerRegistry.addChild(walkingParentRegistry);
      humanoidHighLevelControllerManagerRegistry.addChild(managerParentRegistry);

      controllerToolbox = createHighLevelControllerToolbox(robotModel, yoGraphicsListRegistry);
      humanoidHighLevelControllerManagerRegistry.addChild(controllerToolbox.getYoVariableRegistry());
      setupWalkingMessageHandler(walkingControllerParameters, capturePointPlannerParameters, yoGraphicsListRegistry);
      rootJoint = fullRobotModel.getRootJoint();

      // Initializes this desired robot to the most recent robot configuration data received from the walking controller.
      KinematicsToolboxHelper.setRobotStateFromRawData(robotInitialSetup.getInitialPelvisPose(), robotInitialSetup.getInitialJointAngles(),
                                                       rootJoint,
                                                       FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel));

      managerFactory = new HighLevelControlManagerFactory(managerParentRegistry);
      managerFactory.setHighLevelHumanoidControllerToolbox(controllerToolbox);
      managerFactory.setCapturePointPlannerParameters(capturePointPlannerParameters);
      managerFactory.setWalkingControllerParameters(walkingControllerParameters);

      walkingController = new WalkingHighLevelHumanoidController(walkingInputManager,
                                                                 walkingOutputManager,
                                                                 managerFactory,
                                                                 walkingControllerParameters,
                                                                 controllerToolbox);
      walkingParentRegistry.addChild(walkingController.getYoVariableRegistry());

      // create controller network subscriber here!!
      RealtimeRos2Node realtimeRos2Node = ROS2Tools.createRealtimeRos2Node(DomainFactory.PubSubImplementation.FAST_RTPS,
                                                                           ROS2Tools.HUMANOID_CONTROLLER.getNodeName("atlas"));
      ROS2Tools.MessageTopicNameGenerator subscriberTopicNameGenerator = ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);
      ROS2Tools.MessageTopicNameGenerator publisherTopicNameGenerator = ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      ControllerNetworkSubscriber controllerNetworkSubscriber = new ControllerNetworkSubscriber(subscriberTopicNameGenerator,
                                                                                                walkingInputManager,
                                                                                                publisherTopicNameGenerator,
                                                                                                walkingOutputManager,
                                                                                                realtimeRos2Node);
      controllerNetworkSubscriber.registerSubcriberWithMessageUnpacker(WholeBodyTrajectoryMessage.class, 9,
                                                                       MessageUnpackingTools.createWholeBodyTrajectoryMessageUnpacker());
      controllerNetworkSubscriber.addMessageCollectors(ControllerAPIDefinition.createDefaultMessageIDExtractor(), 3);
      controllerNetworkSubscriber.addMessageValidator(ControllerAPIDefinition.createDefaultMessageValidation());
      realtimeRos2Node.spin();

      WholeBodyControlCoreToolbox controlCoreToolbox = createControllerCoretoolbox(walkingControllerParameters, yoGraphicsListRegistry);

      FeedbackControlCommandList feedbackControlTemplate = managerFactory.createFeedbackControlTemplate();
      JointDesiredOutputList jointDesiredOutputList = new JointDesiredOutputList(controllerToolbox.getControlledOneDoFJoints());

      controllerCore = new WholeBodyControllerCore(controlCoreToolbox, feedbackControlTemplate, jointDesiredOutputList, walkingParentRegistry);
      walkingController.setControllerCoreOutput(controllerCore.getOutputForHighLevelController());

      double controlDT = controllerToolbox.getControlDT();
      RigidBodyBasics elevator = fullRobotModel.getElevator();
      YoDouble yoTime = controllerToolbox.getYoTime();
      SideDependentList<ContactableFoot> contactableFeet = controllerToolbox.getContactableFeet();
      linearMomentumRateControlModule = new LinearMomentumRateControlModule(referenceFrames,
                                                                            contactableFeet,
                                                                            elevator,
                                                                            walkingControllerParameters,
                                                                            yoTime,
                                                                            gravityZ,
                                                                            controlDT,
                                                                            walkingParentRegistry,
                                                                            yoGraphicsListRegistry);

      balanceManager = managerFactory.getOrCreateBalanceManager();

      ParameterLoaderHelper.loadParameters(this, robotModel, drcControllerThreadRegistry);

      YoVariable<?> defaultHeight = registry.getVariable(PelvisHeightControlState.class.getSimpleName(),
                                                         PelvisHeightControlState.class.getSimpleName() + "DefaultHeight");
      if (Double.isNaN(defaultHeight.getValueAsDouble()))
      {
         throw new RuntimeException("Need to load a default height.");
      }

      FunctionalTools.runIfTrue(createYoVariableServer, this::createYoVariableServer);

      initialize();

      scheduler.schedule(this::controllerTick, Conversions.secondsToNanoseconds(DT / PLAYBACK_SPEED), TimeUnit.NANOSECONDS);
   }

   private HighLevelHumanoidControllerToolbox createHighLevelControllerToolbox(DRCRobotModel robotModel, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      double omega0 = robotModel.getWalkingControllerParameters().getOmega0();

      ContactableBodiesFactory<RobotSide> contactableBodiesFactory = createContactableBodiesFactory(robotModel);
      SideDependentList<ContactableFoot> feet = new SideDependentList<>(contactableBodiesFactory.createFootContactableFeet());
      List<ContactablePlaneBody> additionalContacts = contactableBodiesFactory.createAdditionalContactPoints();
      contactableBodiesFactory.disposeFactory();

      List<ContactablePlaneBody> allContactableBodies = new ArrayList<>(additionalContacts);
      allContactableBodies.addAll(feet.values());

      double robotMass = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator());

      for (RobotSide robotSide : RobotSide.values)
      {
         SettableFootSwitch footSwitch = new SettableFootSwitch(feet.get(robotSide), robotMass, 2, registry);
         footSwitch.setFootContactState(true);
         footSwitches.put(robotSide, footSwitch);
      }

      JointBasics[] jointsToIgnore = DRCControllerThread.createListOfJointsToIgnore(fullRobotModel, robotModel, robotModel.getSensorInformation());

      return new HighLevelHumanoidControllerToolbox(fullRobotModel,
                                                    referenceFrames,
                                                    footSwitches,
                                                    null,
                                                    yoTime,
                                                    gravityZ,
                                                    omega0,
                                                    feet,
                                                    DT,
                                                    Collections.emptyList(),
                                                    allContactableBodies,
                                                    yoGraphicsListRegistry,
                                                    jointsToIgnore);
   }

   private void setupWalkingMessageHandler(WalkingControllerParameters walkingControllerParameters, ICPPlannerParameters icpPlannerParameters,
                                           YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      double defaultTransferTime = walkingControllerParameters.getDefaultTransferTime();
      double defaultSwingTime = walkingControllerParameters.getDefaultSwingTime();
      double defaultInitialTransferTime = walkingControllerParameters.getDefaultInitialTransferTime();
      double defaultFinalTransferTime = walkingControllerParameters.getDefaultFinalTransferTime();
      double defaultSwingDurationShiftFraction = icpPlannerParameters.getSwingDurationShiftFraction();
      double defaultSwingSplitFraction = icpPlannerParameters.getSwingSplitFraction();
      double defaultTransferSplitFraction = icpPlannerParameters.getTransferSplitFraction();
      WalkingMessageHandler walkingMessageHandler = new WalkingMessageHandler(defaultTransferTime,
                                                                              defaultSwingTime,
                                                                              defaultInitialTransferTime,
                                                                              defaultFinalTransferTime,
                                                                              defaultSwingDurationShiftFraction,
                                                                              defaultSwingSplitFraction,
                                                                              defaultTransferSplitFraction,
                                                                              defaultTransferSplitFraction,
                                                                              controllerToolbox.getContactableFeet(),
                                                                              walkingOutputManager,
                                                                              yoTime,
                                                                              yoGraphicsListRegistry,
                                                                              controllerToolbox.getYoVariableRegistry());
      controllerToolbox.setWalkingMessageHandler(walkingMessageHandler);
   }

   private WholeBodyControlCoreToolbox createControllerCoretoolbox(WalkingControllerParameters walkingControllerParameters,
                                                                   YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      JointBasics[] controlledJoints = controllerToolbox.getControlledJoints();
      MomentumOptimizationSettings momentumOptimizationSettings = walkingControllerParameters.getMomentumOptimizationSettings();
      JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters = walkingControllerParameters.getJointPrivilegedConfigurationParameters();
      FeedbackControllerSettings feedbackControllerSettings = walkingControllerParameters.getFeedbackControllerSettings();

      WholeBodyControlCoreToolbox controlCoreToolbox = new WholeBodyControlCoreToolbox(DT,
                                                                                       gravityZ,
                                                                                       fullRobotModel.getRootJoint(),
                                                                                       controlledJoints,
                                                                                       controllerToolbox.getCenterOfMassFrame(),
                                                                                       momentumOptimizationSettings,
                                                                                       yoGraphicsListRegistry,
                                                                                       registry);

      controlCoreToolbox.setJointPrivilegedConfigurationParameters(jointPrivilegedConfigurationParameters);
      controlCoreToolbox.setFeedbackControllerSettings(feedbackControllerSettings);
      controlCoreToolbox.setupForInverseDynamicsSolver(controllerToolbox.getContactablePlaneBodies());

      return controlCoreToolbox;
   }

   private ContactableBodiesFactory<RobotSide> createContactableBodiesFactory(DRCRobotModel robotModel)
   {
      RobotContactPointParameters<RobotSide> contactPointParameters = robotModel.getContactPointParameters();
      ContactableBodiesFactory<RobotSide> contactableBodiesFactory = new ContactableBodiesFactory<>();
      ArrayList<String> additionalContactRigidBodyNames = contactPointParameters.getAdditionalContactRigidBodyNames();
      ArrayList<String> additionaContactNames = contactPointParameters.getAdditionalContactNames();
      ArrayList<RigidBodyTransform> additionalContactTransforms = contactPointParameters.getAdditionalContactTransforms();

      contactableBodiesFactory.setFootContactPoints(contactPointParameters.getFootContactPoints());
      contactableBodiesFactory.setToeContactParameters(contactPointParameters.getControllerToeContactPoints(),
                                                       contactPointParameters.getControllerToeContactLines());
      for (int i = 0; i < contactPointParameters.getAdditionalContactNames().size(); i++)
         contactableBodiesFactory.addAdditionalContactPoint(additionalContactRigidBodyNames.get(i),
                                                            additionaContactNames.get(i),
                                                            additionalContactTransforms.get(i));

      contactableBodiesFactory.setFullRobotModel(fullRobotModel);
      contactableBodiesFactory.setReferenceFrames(referenceFrames);

      return contactableBodiesFactory;
   }

   public void initialize()
   {
      zeroMotion();

      referenceFrames.updateFrames();
      controllerCore.initialize();
      controllerToolbox.update();
      walkingController.initialize();

      taskExecutor.clear();
      taskExecutor.submit(new KinematicsWalkingInitializeTask(walkingController, controllerToolbox.getFootContactStates())); // few ticks to get it going
   }

   public void doControl()
   {
      if (walkingInputManager.isNewCommandAvailable(FootstepDataListCommand.class)) // TODO: listen to general commands
      {
         FootstepDataListCommand foostepCommand = walkingInputManager.pollNewestCommand(FootstepDataListCommand.class);
         taskExecutor.submit(new KinematicsWalkingFootstepSequenceTask(fullRobotModel.getRootJoint(),
                                                                       foostepCommand,
                                                                       walkingInputManager,
                                                                       walkingOutputManager,
                                                                       controllerToolbox.getFootContactStates(), balanceManager,
                                                                       footSwitches,
                                                                       walkingStatusPublisher,
                                                                       footstepStatusPublisher));
      }
      else
      {
         yoTime.add(DT);
         fullRobotModel.updateFrames();
         referenceFrames.updateFrames();
         controllerToolbox.update();
      }

      if (balanceManager != null)
      {
         capturabilityBasedStatus = balanceManager.updateAndReturnCapturabilityBasedStatus();
      }

      if (taskExecutor.isDone())  // keep robot from drifting when no tasks are present
      {
         zeroMotion();
      }
      else
      {
         State currentTask = taskExecutor.getCurrentTask();
         if (currentTask instanceof KinematicsWalkingInitializeTask)
         {

         }
         else if (currentTask instanceof KinematicsWalkingFootstepSequenceTask)
         {
            KinematicsWalkingFootstepSequenceTask footstepSequenceTask = (KinematicsWalkingFootstepSequenceTask) currentTask;
         }

         taskExecutor.doControl();

         walkingController.doAction();

         linearMomentumRateControlModule.setInputFromWalkingStateMachine(walkingController.getLinearMomentumRateControlModuleInput());
         if (!linearMomentumRateControlModule.computeControllerCoreCommands())
         {
            controllerToolbox.reportControllerFailureToListeners(new FrameVector2D());
         }
         walkingController.setLinearMomentumRateControlModuleOutput(linearMomentumRateControlModule.getOutputForWalkingStateMachine());

         ControllerCoreCommand controllerCoreCommand = walkingController.getControllerCoreCommand();
         controllerCoreCommand.addInverseDynamicsCommand(linearMomentumRateControlModule.getMomentumRateCommand());
         if (!taskExecutor.isDone())
         {
            controllerCoreCommand.addInverseDynamicsCommand(((KinematicsWalkingTask) taskExecutor.getCurrentTask()).getOutput());
         }

         controllerCore.submitControllerCoreCommand(controllerCoreCommand);
         controllerCore.compute();

         linearMomentumRateControlModule.setInputFromControllerCore(controllerCore.getControllerCoreOutput());
         linearMomentumRateControlModule.computeAchievedCMP();

         rootJoint.setJointAcceleration(0, controllerCore.getOutputForRootJoint().getDesiredAcceleration());
         JointDesiredOutputListReadOnly jointDesiredOutputList = controllerCore.getOutputForLowLevelController();

         for (OneDoFJointBasics joint : controllerToolbox.getControlledOneDoFJoints())
         {
            JointDesiredOutputReadOnly jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(joint);
            joint.setQdd(jointDesiredOutput.getDesiredAcceleration());
         }

         integrator.setIntegrationDT(DT);
         integrator.doubleIntegrateFromAcceleration(Arrays.asList(controllerToolbox.getControlledJoints()));
      }
   }

   private void zeroMotion()
   {
      for (JointBasics joint : fullRobotModel.getElevator().childrenSubtreeIterable())
      {
         joint.setJointAccelerationToZero();
         joint.setJointTwistToZero();
      }
   }

   private void controllerTick()
   {
      doControl();

      robotConfigurationDataPublisher.publish(extractRobotConfigurationData(fullRobotModel));

      capturabilityBasedStatusPublisher.publish(capturabilityBasedStatus);
   }

   private void createYoVariableServer()
   {
      yoVariableServer = new YoVariableServer(getClass(), robotModel.getLogModelProvider(), new DataServerSettings(false), 0.01);
      yoVariableServer.setMainRegistry(registry, robotModel.createFullRobotModel().getElevator(), yoGraphicsListRegistry);
      ThreadTools.startAThread(() -> yoVariableServer.start(), getClass().getSimpleName() + "YoVariableServer");

      yoVariableServerScheduled = yoVariableScheduler.schedule(this::yoVariableUpdateThread, 1, TimeUnit.MILLISECONDS);
   }

   private void yoVariableUpdateThread()
   {
      if (!Thread.interrupted())
      {
         yoVariableServerTime += Conversions.millisecondsToSeconds(1);
         yoVariableServer.update(Conversions.secondsToNanoseconds(yoVariableServerTime));
      }
   }

   public static RobotConfigurationData extractRobotConfigurationData(FullHumanoidRobotModel fullRobotModel)
   {
      fullRobotModel.updateFrames();
      OneDoFJointBasics[] joints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
      RobotConfigurationData robotConfigurationData = RobotConfigurationDataFactory.create(joints,
                                                                                           fullRobotModel.getForceSensorDefinitions(),
                                                                                           fullRobotModel.getIMUDefinitions());
      RobotConfigurationDataFactory.packJointState(robotConfigurationData, Arrays.stream(joints).collect(Collectors.toList()));
      robotConfigurationData.getRootTranslation().set(fullRobotModel.getRootJoint().getJointPose().getPosition());
      robotConfigurationData.getRootOrientation().set(fullRobotModel.getRootJoint().getJointPose().getOrientation());
      return robotConfigurationData;
   }
}
