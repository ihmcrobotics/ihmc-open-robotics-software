package us.ihmc.avatar.kinematicsSimulation;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxHelper;
import us.ihmc.commonWalkingControlModules.capturePoint.LinearMomentumRateControlModule;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisHeightControlState;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.SettableFootSwitch;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.MessageUnpackingTools;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
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
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

public class HumanoidKinematicsSimulation
{
   private static final double DT = UnitConversions.hertzToSeconds(70);
   private static final double PLAYBACK_SPEED = 10.0;
   private static final double GRAVITY_Z = 9.81;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ExceptionHandlingThreadScheduler scheduler
           = new ExceptionHandlingThreadScheduler(getClass().getSimpleName(), ExceptionHandlingThreadScheduler.DEFAULT_HANDLER, 5);
   private final ExceptionHandlingThreadScheduler yoVariableScheduler
           = new ExceptionHandlingThreadScheduler(getClass().getSimpleName(), ExceptionHandlingThreadScheduler.DEFAULT_HANDLER, 5);
   private final Ros2Node ros2Node;
   private final IHMCROS2Publisher<RobotConfigurationData> robotConfigurationDataPublisher;
//   private final IHMCROS2Publisher<CapturabilityBasedStatus> capturabilityBasedStatusPublisher;
//   private final IHMCROS2Publisher<WalkingStatusMessage> walkingStatusPublisher;
//   private final IHMCROS2Publisher<FootstepStatusMessage> footstepStatusPublisher;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private double yoVariableServerTime = 0.0;
   private ScheduledFuture<?> yoVariableServerScheduled;
   private final YoDouble yoTime;

   private final FullHumanoidRobotModel fullRobotModel;
   private final CommonHumanoidReferenceFrames referenceFrames;
   private final SideDependentList<SettableFootSwitch> footSwitches = new SideDependentList<>();

   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WholeBodyControllerCore controllerCore;
   private final LinearMomentumRateControlModule linearMomentumRateControlModule;
   private final HighLevelControlManagerFactory managerFactory;
   private final WalkingHighLevelHumanoidController walkingController;

   private final CommandInputManager walkingInputManager
           = new CommandInputManager("walking_preview_internal", ControllerAPIDefinition.getControllerSupportedCommands());
   private final StatusMessageOutputManager walkingOutputManager
           = new StatusMessageOutputManager(ControllerAPIDefinition.getControllerSupportedStatusMessages());

   private final MultiBodySystemStateIntegrator integrator = new MultiBodySystemStateIntegrator();

   private CapturabilityBasedStatus capturabilityBasedStatus = new CapturabilityBasedStatus();
   private AtomicReference<WalkingStatus> latestWalkingStatus = new AtomicReference<>();
   private SideDependentList<HumanoidKinematicsSimulationContactStateHolder> contactStateHolders = new SideDependentList<>();
   private RobotSide currentSwingSide = null;
   private InverseDynamicsCommandList inverseDynamicsContactHolderCommandList = new InverseDynamicsCommandList();

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
      new HumanoidKinematicsSimulation(robotModel, createYoVariableServer, pubSubImplementation);
   }

   public HumanoidKinematicsSimulation(DRCRobotModel robotModel, boolean createYoVariableServer, PubSubImplementation pubSubImplementation)
   {
      // instantiate some existing controller ROS2 API?
      ros2Node = ROS2Tools.createRos2Node(pubSubImplementation, ROS2Tools.HUMANOID_CONTROLLER.getNodeName("kinematic"));

      robotConfigurationDataPublisher = new IHMCROS2Publisher<>(ros2Node,
                                                                RobotConfigurationData.class,
                                                                robotModel.getSimpleRobotName(),
                                                                ROS2Tools.HUMANOID_CONTROLLER);
//      capturabilityBasedStatusPublisher = new IHMCROS2Publisher<>(ros2Node,
//                                                                  CapturabilityBasedStatus.class,
//                                                                  robotModel.getSimpleRobotName(),
//                                                                  ROS2Tools.HUMANOID_CONTROLLER);
//      walkingStatusPublisher = new IHMCROS2Publisher<>(ros2Node,
//                                                       WalkingStatusMessage.class,
//                                                       robotModel.getSimpleRobotName(),
//                                                       ROS2Tools.HUMANOID_CONTROLLER);
//      footstepStatusPublisher = new IHMCROS2Publisher<>(ros2Node,
//                                                        FootstepStatusMessage.class,
//                                                        robotModel.getSimpleRobotName(),
//                                                        ROS2Tools.HUMANOID_CONTROLLER);

      String robotName = robotModel.getSimpleRobotName();
      fullRobotModel = robotModel.createFullRobotModel();
      referenceFrames = new HumanoidReferenceFrames(fullRobotModel);

      // Create registries to match controller so the XML gets loaded properly.
      yoTime = new YoDouble("timeInPreview", registry);
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

      ContactableBodiesFactory<RobotSide> contactableBodiesFactory = new ContactableBodiesFactory<>();
      RobotContactPointParameters<RobotSide> contactPointParameters = robotModel.getContactPointParameters();
      contactableBodiesFactory.setFootContactPoints(contactPointParameters.getFootContactPoints());
      contactableBodiesFactory.setToeContactParameters(contactPointParameters.getControllerToeContactPoints(),
                                                       contactPointParameters.getControllerToeContactLines());
      for (int i = 0; i < contactPointParameters.getAdditionalContactNames().size(); i++)
      {
         contactableBodiesFactory.addAdditionalContactPoint(contactPointParameters.getAdditionalContactRigidBodyNames().get(i),
                                                            contactPointParameters.getAdditionalContactNames().get(i),
                                                            contactPointParameters.getAdditionalContactTransforms().get(i));
      }
      contactableBodiesFactory.setFullRobotModel(fullRobotModel);
      contactableBodiesFactory.setReferenceFrames(referenceFrames);
      SideDependentList<ContactableFoot> feet = new SideDependentList<>(contactableBodiesFactory.createFootContactableFeet());
      List<ContactablePlaneBody> allContactableBodies = new ArrayList<>(contactableBodiesFactory.createAdditionalContactPoints());
      allContactableBodies.addAll(feet.values());
      contactableBodiesFactory.disposeFactory();

      double totalRobotWeight = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator());
      for (RobotSide robotSide : RobotSide.values)
      {
         SettableFootSwitch footSwitch = new SettableFootSwitch(feet.get(robotSide), totalRobotWeight, 2, registry);
         footSwitch.setFootContactState(true);
         footSwitches.put(robotSide, footSwitch);
      }

      JointBasics[] jointsToIgnore = DRCControllerThread.createListOfJointsToIgnore(fullRobotModel, robotModel, robotModel.getSensorInformation());

      controllerToolbox = new HighLevelHumanoidControllerToolbox(fullRobotModel,
                                                                 referenceFrames,
                                                                 footSwitches,
                                                                 null,
                                                                 yoTime,
                                                                 GRAVITY_Z,
                                                                 robotModel.getWalkingControllerParameters().getOmega0(),
                                                                 feet,
                                                                 DT,
                                                                 Collections.emptyList(),
                                                                 allContactableBodies,
                                                                 yoGraphicsListRegistry,
                                                                 jointsToIgnore);
      humanoidHighLevelControllerManagerRegistry.addChild(controllerToolbox.getYoVariableRegistry());
      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      ICPWithTimeFreezingPlannerParameters capturePointPlannerParameters = robotModel.getCapturePointPlannerParameters();
      WalkingMessageHandler walkingMessageHandler = new WalkingMessageHandler(walkingControllerParameters.getDefaultTransferTime(),
                                                                              walkingControllerParameters.getDefaultSwingTime(),
                                                                              walkingControllerParameters.getDefaultInitialTransferTime(),
                                                                              walkingControllerParameters.getDefaultFinalTransferTime(),
                                                                              capturePointPlannerParameters.getSwingDurationShiftFraction(),
                                                                              capturePointPlannerParameters.getSwingSplitFraction(),
                                                                              capturePointPlannerParameters.getTransferSplitFraction(),
                                                                              capturePointPlannerParameters.getTransferSplitFraction(),
                                                                              controllerToolbox.getContactableFeet(),
                                                                              walkingOutputManager,
                                                                              yoTime,
                                                                              yoGraphicsListRegistry,
                                                                              controllerToolbox.getYoVariableRegistry());
      controllerToolbox.setWalkingMessageHandler(walkingMessageHandler);

      // Initializes this desired robot to the most recent robot configuration data received from the walking controller.
      DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.0, 0.0);
      KinematicsToolboxHelper.setRobotStateFromRawData(robotInitialSetup.getInitialPelvisPose(), robotInitialSetup.getInitialJointAngles(),
                                                       fullRobotModel.getRootJoint(),
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
      controllerNetworkSubscriber.registerSubcriberWithMessageUnpacker(WholeBodyTrajectoryMessage.class,
                                                                       9,
                                                                       MessageUnpackingTools.createWholeBodyTrajectoryMessageUnpacker());
      controllerNetworkSubscriber.addMessageCollectors(ControllerAPIDefinition.createDefaultMessageIDExtractor(), 3);
      controllerNetworkSubscriber.addMessageValidator(ControllerAPIDefinition.createDefaultMessageValidation());
      realtimeRos2Node.spin();

      WholeBodyControlCoreToolbox controlCoreToolbox = new WholeBodyControlCoreToolbox(DT,
                                                                                       GRAVITY_Z,
                                                                                       fullRobotModel.getRootJoint(),
                                                                                       controllerToolbox.getControlledJoints(),
                                                                                       controllerToolbox.getCenterOfMassFrame(),
                                                                                       walkingControllerParameters.getMomentumOptimizationSettings(),
                                                                                       yoGraphicsListRegistry,
                                                                                       registry);
      controlCoreToolbox.setJointPrivilegedConfigurationParameters(walkingControllerParameters.getJointPrivilegedConfigurationParameters());
      controlCoreToolbox.setFeedbackControllerSettings(walkingControllerParameters.getFeedbackControllerSettings());
      controlCoreToolbox.setupForInverseDynamicsSolver(controllerToolbox.getContactablePlaneBodies());

      controllerCore = new WholeBodyControllerCore(controlCoreToolbox,
                                                   managerFactory.createFeedbackControlTemplate(),
                                                   new JointDesiredOutputList(controllerToolbox.getControlledOneDoFJoints()),
                                                   walkingParentRegistry);
      walkingController.setControllerCoreOutput(controllerCore.getOutputForHighLevelController());

      linearMomentumRateControlModule = new LinearMomentumRateControlModule(referenceFrames,
                                                                            controllerToolbox.getContactableFeet(),
                                                                            fullRobotModel.getElevator(),
                                                                            walkingControllerParameters,
                                                                            controllerToolbox.getYoTime(),
                                                                            GRAVITY_Z,
                                                                            controllerToolbox.getControlDT(),
                                                                            walkingParentRegistry,
                                                                            yoGraphicsListRegistry);

      ParameterLoaderHelper.loadParameters(this, robotModel, drcControllerThreadRegistry);

      YoVariable<?> defaultHeight = registry.getVariable(PelvisHeightControlState.class.getSimpleName(),
                                                         PelvisHeightControlState.class.getSimpleName() + "DefaultHeight");
      if (Double.isNaN(defaultHeight.getValueAsDouble()))
      {
         throw new RuntimeException("Need to load a default height.");
      }

      if (createYoVariableServer)
      {
         YoVariableServer yoVariableServer = new YoVariableServer(getClass(), robotModel.getLogModelProvider(), new DataServerSettings(false), 0.01);
         yoVariableServer.setMainRegistry(registry, robotModel.createFullRobotModel().getElevator(), yoGraphicsListRegistry);
         ThreadTools.startAThread(() -> yoVariableServer.start(), getClass().getSimpleName() + "YoVariableServer");
         yoVariableServerScheduled = yoVariableScheduler.schedule(() ->
         {
            if (!Thread.interrupted())
            {
               yoVariableServerTime += Conversions.millisecondsToSeconds(1);
               yoVariableServer.update(Conversions.secondsToNanoseconds(yoVariableServerTime));
            }
         }, 1, TimeUnit.MILLISECONDS);
      }

      walkingOutputManager.attachStatusMessageListener(FootstepStatusMessage.class, this::processFootstepStatus);
      walkingOutputManager.attachStatusMessageListener(WalkingStatusMessage.class, this::processWalkingStatus);

      initialize();

      scheduler.schedule(this::controllerTick, Conversions.secondsToNanoseconds(DT / PLAYBACK_SPEED), TimeUnit.NANOSECONDS);
   }

   public void initialize()
   {
      zeroMotion();

      referenceFrames.updateFrames();
      controllerCore.initialize();
      walkingController.initialize();

      walkingController.requestImmediateTransitionToStandingAndHoldCurrent();

      for (RobotSide robotSide : RobotSide.values)
      {
         contactStateHolders.put(robotSide, HumanoidKinematicsSimulationContactStateHolder.holdAtCurrent(controllerToolbox.getFootContactStates().get(robotSide)));
      }

//      for (RobotSide robotSide : RobotSide.values)
//      {
//         contactStateHolders.put(robotSide, HumanoidKinematicsSimulationContactStateHolder.holdAtCurrent(controllerToolbox.getFootContactStates().get(robotSide)));
//      }

//      taskExecutor.clear();
      // few ticks to get it going
//      taskExecutor.submit(new InitializeTask());
   }

   private void controllerTick()
   {
      doControl();

      robotConfigurationDataPublisher.publish(extractRobotConfigurationData(fullRobotModel));

//      capturabilityBasedStatusPublisher.publish(capturabilityBasedStatus);
   }

   public void doControl()
   {
      //      if (walkingInputManager.isNewCommandAvailable(FootstepDataListCommand.class)) // TODO: listen to general commands
      //      {
      //         taskExecutor.submit(new FootstepSequenceTask());
      //      }
      //      else
      //      {
      yoTime.add(DT);
      fullRobotModel.updateFrames();
      referenceFrames.updateFrames();
      controllerToolbox.update();
      //      }

      capturabilityBasedStatus = managerFactory.getOrCreateBalanceManager().updateAndReturnCapturabilityBasedStatus();

      //      if (taskExecutor.isDone())  // keep robot from drifting when no tasks are present
      //      {
      //         zeroMotion();
      //      }
      //      else
      //      {

      // walking state?
      //      if (latestWalkingStatus.get() == null || latestWalkingStatus.get() == WalkingStatus.COMPLETED)
      //      {
      //
      //      }

      // update contact state holders
      //      for (RobotSide robotSide : RobotSide.values)
      //      {
      //         contactStateHolders.put(robotSide, HumanoidKinematicsSimulationContactStateHolder.holdAtCurrent(controllerToolbox.getFootContactStates().get(robotSide)));
      //      }

      //      if (currentSwingSide != null)
      //      { // Testing for the end of swing purely relying on the swing time:
      //         if (managerFactory.getOrCreateBalanceManager().isICPPlanDone())
      //            footSwitches.get(currentSwingSide).setFootContactState(true);
      //      }
      //      else
      //      {
      //
      //      }

      //         taskExecutor.doControl(); // TODO: If not walking, hold contact

      inverseDynamicsContactHolderCommandList.clear();
      for (RobotSide side : contactStateHolders.sides())
      {
         contactStateHolders.get(side).doControl();
         inverseDynamicsContactHolderCommandList.addCommand(contactStateHolders.get(side).getOutput());
      }

      walkingController.doAction();

      linearMomentumRateControlModule.setInputFromWalkingStateMachine(walkingController.getLinearMomentumRateControlModuleInput());
      if (!linearMomentumRateControlModule.computeControllerCoreCommands())
      {
         controllerToolbox.reportControllerFailureToListeners(new FrameVector2D());
      }
      walkingController.setLinearMomentumRateControlModuleOutput(linearMomentumRateControlModule.getOutputForWalkingStateMachine());

      ControllerCoreCommand controllerCoreCommand = walkingController.getControllerCoreCommand();
      controllerCoreCommand.addInverseDynamicsCommand(linearMomentumRateControlModule.getMomentumRateCommand());
      controllerCoreCommand.addInverseDynamicsCommand(inverseDynamicsContactHolderCommandList);

      controllerCore.submitControllerCoreCommand(controllerCoreCommand);
      controllerCore.compute();

      linearMomentumRateControlModule.setInputFromControllerCore(controllerCore.getControllerCoreOutput());
      linearMomentumRateControlModule.computeAchievedCMP();

      fullRobotModel.getRootJoint().setJointAcceleration(0, controllerCore.getOutputForRootJoint().getDesiredAcceleration());
      JointDesiredOutputListReadOnly jointDesiredOutputList = controllerCore.getOutputForLowLevelController();

      for (OneDoFJointBasics joint : controllerToolbox.getControlledOneDoFJoints())
      {
         JointDesiredOutputReadOnly jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(joint);
         joint.setQdd(jointDesiredOutput.getDesiredAcceleration());
      }

      integrator.setIntegrationDT(DT);
      integrator.doubleIntegrateFromAcceleration(Arrays.asList(controllerToolbox.getControlledJoints()));
   }

   private void zeroMotion()
   {
      for (JointBasics joint : fullRobotModel.getElevator().childrenSubtreeIterable())
      {
         joint.setJointAccelerationToZero();
         joint.setJointTwistToZero();
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

   private void processFootstepStatus(FootstepStatusMessage statusMessage)
   {
      RobotSide side = RobotSide.fromByte(statusMessage.getRobotSide());
      FootstepStatus status = FootstepStatus.fromByte(statusMessage.getFootstepStatus());
      FramePose3D desiredFootstep = new FramePose3D(worldFrame,
                                                    statusMessage.getDesiredFootPositionInWorld(),
                                                    statusMessage.getDesiredFootOrientationInWorld());

      switch (status)
      {
         case STARTED:
            contactStateHolders.remove(side);
            footSwitches.get(side).setFootContactState(false);
            currentSwingSide = side;
            break;
         case COMPLETED:
//            numberOfFootstepsRemaining--;
            contactStateHolders.put(side, new HumanoidKinematicsSimulationContactStateHolder(controllerToolbox.getFootContactStates().get(side), desiredFootstep));
            currentSwingSide = null;
            break;
         default:
            throw new RuntimeException("Unexpected status: " + status);
      }

//      footstepStatusPublisher.publish(statusMessage);
   }

   private void processWalkingStatus(WalkingStatusMessage status)
   {
      latestWalkingStatus.set(WalkingStatus.fromByte(status.getWalkingStatus()));
//      walkingStatusPublisher.publish(status);
   }
}
