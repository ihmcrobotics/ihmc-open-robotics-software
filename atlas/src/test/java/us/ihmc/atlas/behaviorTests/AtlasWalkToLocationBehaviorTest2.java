package us.ihmc.atlas.behaviorTests;

import controller_msgs.msg.dds.*;
import org.junit.jupiter.api.*;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.factory.AvatarSimulation;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.avatar.testTools.ScriptedFootstepGenerator;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.IHMCHumanoidBehaviorManager;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WalkToLocationBehavior;
import us.ihmc.humanoidBehaviors.dispatcher.BehaviorControlModeSubscriber;
import us.ihmc.humanoidBehaviors.dispatcher.BehaviorDispatcher;
import us.ihmc.humanoidBehaviors.dispatcher.HumanoidBehaviorTypeSubscriber;
import us.ihmc.humanoidBehaviors.utilities.CapturePointUpdatable;
import us.ihmc.humanoidBehaviors.utilities.WristForceSensorFilteredUpdatable;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.humanoidRobotics.communication.subscribers.CapturabilityBasedStatusSubscriber;
import us.ihmc.humanoidRobotics.communication.subscribers.HumanoidRobotDataReceiver;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.*;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

@Tag("humanoid-behaviors")
public class AtlasWalkToLocationBehaviorTest2
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private SimulationConstructionSet scs;
   private DRCRobotModel robotModel;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcBehaviorTestHelper != null)
      {
         drcBehaviorTestHelper.closeAndDispose();
         drcBehaviorTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @AfterAll
   public static void printMemoryUsageAfterClass()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(AtlasWalkToLocationBehaviorTest2.class + " after class.");
   }

   private static final boolean DEBUG = false;

   // Atlas typically achieves between 0.02-0.03 position threshold
   private final double POSITION_THRESHOLD = 0.06;

   // Atlas typically achieves between .005-0.1 orientation threshold (more accurate when turning in place at final target)
   private final double ORIENTATION_THRESHOLD = 0.2;

   private DRCBehaviorTestHelper drcBehaviorTestHelper;

   @BeforeEach
   public void setUp()
   {
      FlatGroundEnvironment testEnvironment = new FlatGroundEnvironment();

      String robotName = BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
      DRCStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
      DRCNetworkModuleParameters networkModuleParameters = null;
      boolean automaticallySpawnSimulation = true;

      WalkingControllerParameters walkingControlParameters = robotModel.getWalkingControllerParameters();

      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, testEnvironment);

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      ScriptedFootstepGenerator scriptedFootstepGenerator = new ScriptedFootstepGenerator(referenceFrames, fullRobotModel, walkingControlParameters);

      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(false, false, simulationTestingParameters);

      DRCNetworkModuleParameters networkProcessorParameters = new DRCNetworkModuleParameters();
      networkProcessorParameters.enableNetworkProcessor(false);
      networkProcessorParameters.enableLocalControllerCommunicator(true);

      List<Class<? extends Command<?, ?>>> controllerSupportedCommands = ControllerAPIDefinition.getControllerSupportedCommands();

      Map<Class<?>, IHMCROS2Publisher> defaultControllerPublishers = new HashMap<>();
      Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.INTRAPROCESS, "ihmc_simulation_test_helper");
      for (Class<? extends Command<?, ?>> command : controllerSupportedCommands)
      {
         Class<?> messageClass = ROS2Tools.newMessageInstance(command).getMessageClass();
         MessageTopicNameGenerator generator = ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);
         IHMCROS2Publisher<?> defaultPublisher = ROS2Tools.createPublisher(ros2Node, messageClass, generator);
         defaultControllerPublishers.put(messageClass, defaultPublisher);
      }

      MessageTopicNameGenerator controllerGenerator = ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);
      defaultControllerPublishers
            .put(WholeBodyTrajectoryMessage.class, ROS2Tools.createPublisher(ros2Node, WholeBodyTrajectoryMessage.class, controllerGenerator));
      defaultControllerPublishers.put(MessageCollection.class, ROS2Tools.createPublisher(ros2Node, MessageCollection.class, controllerGenerator));
      defaultControllerPublishers.put(ValkyrieHandFingerTrajectoryMessage.class,
                                      ROS2Tools.createPublisher(ros2Node, ValkyrieHandFingerTrajectoryMessage.class, controllerGenerator));

      OffsetAndYawRobotInitialSetup startingLocationOffset = selectedLocation.getStartingLocationOffset();
      if (networkModuleParameters == null)
      {
         networkModuleParameters = new DRCNetworkModuleParameters();
         networkModuleParameters.enableNetworkProcessor(false);
      }
      simulationStarter.setRunMultiThreaded(simulationTestingParameters.getRunMultiThreaded());
      simulationStarter.setUsePerfectSensors(simulationTestingParameters.getUsePefectSensors());
      simulationStarter.setStartingLocationOffset(startingLocationOffset);
      simulationStarter.setGuiInitialSetup(guiInitialSetup);
      simulationStarter.setInitializeEstimatorToActual(true);

      networkProcessorParameters.enableLocalControllerCommunicator(true);
      simulationStarter.createSimulation(networkProcessorParameters, automaticallySpawnSimulation, false);

      scs = simulationStarter.getSimulationConstructionSet();
      HumanoidFloatingRootJointRobot sdfRobot = simulationStarter.getSDFRobot();
      AvatarSimulation avatarSimulation = simulationStarter.getAvatarSimulation();

      YoDouble yoTimeRobot = sdfRobot.getYoTime();
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      YoDouble yoTimeBehaviorDispatcher = new YoDouble("yoTimeBehaviorDispatcher", registry);

      FullHumanoidRobotModel anotherFullRobotModel = robotModel.createFullRobotModel();
      YoDouble yoTimeLastFullRobotModelUpdate = new YoDouble("yoTimeRobotModelUpdate", registry);

      ForceSensorDataHolder forceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(anotherFullRobotModel.getForceSensorDefinitions()));
      HumanoidRobotDataReceiver robotDataReceiver = new HumanoidRobotDataReceiver(anotherFullRobotModel, forceSensorDataHolder);
      ROS2Tools.createCallbackSubscription(ros2Node, RobotConfigurationData.class, ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName),
                                           s -> robotDataReceiver.receivedPacket(s.takeNextData()));

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      CapturabilityBasedStatusSubscriber capturabilityBasedStatusSubsrciber = new CapturabilityBasedStatusSubscriber();
      ROS2Tools.createCallbackSubscription(ros2Node, CapturabilityBasedStatus.class, ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName),
                                           s -> capturabilityBasedStatusSubsrciber.receivedPacket(s.takeNextData()));

      CapturePointUpdatable capturePointUpdatable = new CapturePointUpdatable(capturabilityBasedStatusSubsrciber, yoGraphicsListRegistry, registry);
      ArrayList<Updatable> updatables = new ArrayList<>();
      updatables.add(capturePointUpdatable);

      SideDependentList<WristForceSensorFilteredUpdatable> wristForceSensorUpdatables = null;
      if (robotModel.getSensorInformation().getWristForceSensorNames() != null && !robotModel.getSensorInformation().getWristForceSensorNames().isEmpty())
      {
         wristForceSensorUpdatables = new SideDependentList<>();

         for (RobotSide robotSide : RobotSide.values)
         {
            WristForceSensorFilteredUpdatable wristSensorUpdatable = new WristForceSensorFilteredUpdatable(robotModel.getSimpleRobotName(), robotSide,
                                                                                                           fullRobotModel, robotModel.getSensorInformation(),
                                                                                                           robotDataReceiver.getForceSensorDataHolder(),
                                                                                                           IHMCHumanoidBehaviorManager.BEHAVIOR_YO_VARIABLE_SERVER_DT,
                                                                                                           ros2Node, registry);

            wristForceSensorUpdatables.put(robotSide, wristSensorUpdatable);
         }
         updatables.add(wristForceSensorUpdatables.get(RobotSide.LEFT));
         updatables.add(wristForceSensorUpdatables.get(RobotSide.RIGHT));
      }

      BehaviorControlModeSubscriber desiredBehaviorControlSubscriber = new BehaviorControlModeSubscriber();
      ROS2Tools.createCallbackSubscription(ros2Node, BehaviorControlModePacket.class, IHMCHumanoidBehaviorManager.getSubscriberTopicNameGenerator(robotName),
                                           s -> desiredBehaviorControlSubscriber.receivedPacket(s.takeNextData()));

      HumanoidBehaviorTypeSubscriber desiredBehaviorSubscriber = new HumanoidBehaviorTypeSubscriber();
      ROS2Tools.createCallbackSubscription(ros2Node, HumanoidBehaviorTypePacket.class, IHMCHumanoidBehaviorManager.getSubscriberTopicNameGenerator(robotName),
                                           s -> desiredBehaviorSubscriber.receivedPacket(s.takeNextData()));

      YoVariableServer yoVariableServer = null;
      yoGraphicsListRegistry.setYoGraphicsUpdatedRemotely(false);

      BehaviorDispatcher<HumanoidBehaviorType> behaviorDispatcher = new BehaviorDispatcher<>(robotName, yoTimeBehaviorDispatcher, robotDataReceiver,
                                                                                             desiredBehaviorControlSubscriber, desiredBehaviorSubscriber,
                                                                                             ros2Node, yoVariableServer, HumanoidBehaviorType.class,
                                                                                             HumanoidBehaviorType.STOP, registry, yoGraphicsListRegistry);

      behaviorDispatcher.addUpdatable(capturePointUpdatable);

      if (wristForceSensorUpdatables != null)
      {
         behaviorDispatcher.addUpdatable(wristForceSensorUpdatables.get(RobotSide.LEFT));
         behaviorDispatcher.addUpdatable(wristForceSensorUpdatables.get(RobotSide.RIGHT));
      }

      behaviorDispatcher.finalizeStateMachine();

      IHMCROS2Publisher<HumanoidBehaviorTypePacket> humanoidBehabiorTypePublisher = ROS2Tools
            .createPublisher(ros2Node, HumanoidBehaviorTypePacket.class, IHMCHumanoidBehaviorManager.getSubscriberTopicNameGenerator(robotName));
   }

   @Test
   public void testWalkAtAngleAndFinishAlignedWithInitialOrientation() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      LogTools.info("Initializing Sim");

      AtlasTestYoVariables variables = new AtlasTestYoVariables(scs);
      GoalOrientedTestConductor conductor = new GoalOrientedTestConductor(scs, simulationTestingParameters);

      conductor.addDurationGoal(variables.getYoTime(), 1.0);
      conductor.simulate();

      //      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      //      assertTrue(success);
      //      PrintTools.debug(this, "Initializing Behavior");
      //      double walkDistance = RandomNumbers.nextDouble(new Random(), 1.0, 2.0);
      //      double walkAngleDegrees = RandomNumbers.nextDouble(new Random(), 45.0);
      //      Vector2D walkDirection = new Vector2D(Math.cos(Math.toRadians(walkAngleDegrees)), Math.sin(Math.toRadians(walkAngleDegrees)));
      //      FramePose2D desiredMidFeetPose2d = copyAndOffsetCurrentMidfeetPose2d(walkDistance, walkDirection);
      //      WalkToLocationBehavior walkToLocationBehavior = createAndSetupWalkToLocationBehavior(desiredMidFeetPose2d);
      //      PrintTools.debug(this, "Starting to Execute Behavior");
      //      success = drcBehaviorTestHelper.executeBehaviorUntilDone(walkToLocationBehavior);
      //      assertTrue(success);
      //      PrintTools.debug(this, "Behavior Should be done");
      //      assertCurrentMidFeetPoseIsWithinThreshold(desiredMidFeetPose2d);
      //      assertTrue(walkToLocationBehavior.isDone());
      //      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private FramePose2D copyAndOffsetCurrentMidfeetPose2d(double walkDistance, Vector2D walkDirection)
   {
      FramePose2D desiredMidFeetPose = getCurrentMidFeetPose2dCopy();
      walkDirection.normalize();
      desiredMidFeetPose.setX(desiredMidFeetPose.getX() + walkDistance * walkDirection.getX());
      desiredMidFeetPose.setY(desiredMidFeetPose.getY() + walkDistance * walkDirection.getY());

      return desiredMidFeetPose;
   }

   private WalkToLocationBehavior createAndSetupWalkToLocationBehavior(FramePose2D desiredMidFeetPose)
   {
      return createAndSetupWalkToLocationBehavior(desiredMidFeetPose, 0.0);
   }

   private WalkToLocationBehavior createAndSetupWalkToLocationBehavior(FramePose2D desiredMidFeetPose, double walkingOrientationRelativeToPathDirection)
   {
      final WalkToLocationBehavior walkToLocationBehavior = createNewWalkToLocationBehavior();
      walkToLocationBehavior.initialize();
      walkToLocationBehavior.setWalkingOrientationRelativeToPathDirection(walkingOrientationRelativeToPathDirection);
      walkToLocationBehavior.setTarget(desiredMidFeetPose);
      assertTrue(walkToLocationBehavior.hasInputBeenSet());

      return walkToLocationBehavior;
   }

   private WalkToLocationBehavior createNewWalkToLocationBehavior()
   {
      Ros2Node ros2Node = drcBehaviorTestHelper.getRos2Node();
      FullHumanoidRobotModel fullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();
      HumanoidReferenceFrames referenceFrames = drcBehaviorTestHelper.getReferenceFrames();
      WalkingControllerParameters walkingControllerParams = robotModel.getWalkingControllerParameters();
      final WalkToLocationBehavior walkToLocationBehavior = new WalkToLocationBehavior(drcBehaviorTestHelper.getRobotName(), ros2Node, fullRobotModel,
                                                                                       referenceFrames, walkingControllerParams);

      return walkToLocationBehavior;
   }

   private FramePose2D getCurrentMidFeetPose2dCopy()
   {
      drcBehaviorTestHelper.updateRobotModel();
      ReferenceFrame midFeetFrame = drcBehaviorTestHelper.getReferenceFrames().getMidFeetZUpFrame();
      FramePose3D midFeetPose = new FramePose3D();
      midFeetPose.setToZero(midFeetFrame);
      midFeetPose.changeFrame(ReferenceFrame.getWorldFrame());
      FramePose2D ret = new FramePose2D();
      ret.setIncludingFrame(midFeetPose.getReferenceFrame(), midFeetPose.getX(), midFeetPose.getY(), midFeetPose.getYaw());

      return ret;
   }

   private void assertCurrentMidFeetPoseIsWithinThreshold(FramePose2D desiredMidFeetPose)
   {
      FramePose2D currentMidFeetPose = getCurrentMidFeetPose2dCopy();
      assertPosesAreWithinThresholds(desiredMidFeetPose, currentMidFeetPose);
   }

   private void assertPosesAreWithinThresholds(FramePose2D desiredPose, FramePose2D actualPose)
   {
      assertPosesAreWithinThresholds(desiredPose, actualPose, POSITION_THRESHOLD);
   }

   private void assertPosesAreWithinThresholds(FramePose2D desiredPose, FramePose2D actualPose, double positionThreshold)
   {
      assertPosesAreWithinThresholds(desiredPose, actualPose, positionThreshold, ORIENTATION_THRESHOLD);
   }

   private void assertPosesAreWithinThresholds(FramePose2D desiredPose, FramePose2D actualPose, double positionThreshold, double orientationThreshold)
   {
      double positionDistance = desiredPose.getPositionDistance(actualPose);
      double orientationDistance = desiredPose.getOrientationDistance(actualPose);
      if (DEBUG)
      {
         PrintTools.debug(this, " desired Midfeet Pose :\n" + desiredPose + "\n");
         PrintTools.debug(this, " actual Midfeet Pose :\n" + actualPose + "\n");
         PrintTools.debug(this, " positionDistance = " + positionDistance);
         PrintTools.debug(this, " orientationDistance = " + orientationDistance);
      }

      assertEquals("Pose position error :" + positionDistance + " exceeds threshold: " + positionThreshold, 0.0, positionDistance, positionThreshold);
      assertEquals("Pose orientation error :" + orientationDistance + " exceeds threshold: " + orientationThreshold, 0.0, orientationDistance,
                   orientationThreshold);
   }
}
