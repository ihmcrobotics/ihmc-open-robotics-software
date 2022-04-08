package us.ihmc.avatar.roughTerrainWalking;

import static us.ihmc.robotics.Assert.assertTrue;
import static us.ihmc.robotics.Assert.fail;

import java.io.IOException;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepPlannerParametersPacket;
import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.ToolboxStateMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.HumanoidNetworkProcessor;
import us.ihmc.avatar.networkProcessor.HumanoidNetworkProcessorParameters;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.communication.FootstepPlannerAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.tools.FootstepPlannerMessageTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.humanoidRobotics.communication.subscribers.HumanoidRobotDataReceiver;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.simulation.TimeConsumer;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimFloatingJointBasics;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.TwoBollardEnvironment;
import us.ihmc.simulationConstructionSetTools.util.planarRegions.PlanarRegionsListExamples;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;

public abstract class AvatarBipedalFootstepPlannerEndToEndTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final boolean keepSCSUp = false;

   protected SCS2AvatarTestingSimulation simulationTestHelper;
   private HumanoidNetworkProcessorParameters networkModuleParameters;
   private HumanoidNetworkProcessor networkProcessor;
   protected HumanoidRobotDataReceiver humanoidRobotDataReceiver;

   private FootstepPlanningModule footstepPlanningModule;
   private IHMCRealtimeROS2Publisher<FootstepPlanningRequestPacket> footstepPlanningRequestPublisher;
   private IHMCRealtimeROS2Publisher<ToolboxStateMessage> toolboxStatePublisher;
   private IHMCRealtimeROS2Publisher<FootstepPlannerParametersPacket> footstepPlannerParametersPublisher;

   private RealtimeROS2Node ros2Node;

   protected final TwoBollardEnvironment bollardEnvironment = new TwoBollardEnvironment(BOLLARD_DISTANCE);
   private PlanarRegionsList cinderBlockField;
   private PlanarRegionsList steppingStoneField;
   private PlanarRegionsList bollardPlanarRegions;
   private PlanarRegionsList flatGround;

   public static final double CINDER_BLOCK_START_X = 0.0;
   public static final double CINDER_BLOCK_START_Y = 0.0;
   public static final double CINDER_BLOCK_HEIGHT = 0.1;
   public static final double CINDER_BLOCK_SIZE = 0.4;
   public static final int CINDER_BLOCK_COURSE_WIDTH_X_IN_NUMBER_OF_BLOCKS = 5;
   public static final int CINDER_BLOCK_COURSE_LENGTH_Y_IN_NUMBER_OF_BLOCKS = 6;
   public static final double CINDER_BLOCK_HEIGHT_VARIATION = 0.0;
   public static final double CINDER_BLOCK_FIELD_PLATFORM_LENGTH = 0.6;
   protected static final double BOLLARD_DISTANCE = 0.85;

   public static final double STEPPING_STONE_PATH_RADIUS = 3.5;

   private volatile boolean planCompleted = false;
   private AtomicReference<FootstepPlanningToolboxOutputStatus> outputStatus;

   @BeforeEach
   public void setup() throws IOException
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(CINDER_BLOCK_START_X, CINDER_BLOCK_START_Y, 0.001);
      PlanarRegionsListExamples.generateCinderBlockField(generator,
                                                         CINDER_BLOCK_SIZE,
                                                         CINDER_BLOCK_HEIGHT,
                                                         CINDER_BLOCK_COURSE_WIDTH_X_IN_NUMBER_OF_BLOCKS,
                                                         CINDER_BLOCK_COURSE_LENGTH_Y_IN_NUMBER_OF_BLOCKS,
                                                         CINDER_BLOCK_HEIGHT_VARIATION);
      cinderBlockField = generator.getPlanarRegionsList();
      steppingStoneField = PlanarRegionsListExamples.generateSteppingStonesEnvironment(STEPPING_STONE_PATH_RADIUS);
      bollardPlanarRegions = bollardEnvironment.getPlanarRegionsList();

      generator.reset();
      generator.addRectangle(5.0, 5.0);
      flatGround = generator.getPlanarRegionsList();

      networkModuleParameters = new HumanoidNetworkProcessorParameters();
      networkModuleParameters.setUseFootstepPlanningToolboxModule(true);

      ros2Node = ROS2Tools.createRealtimeROS2Node(PubSubImplementation.INTRAPROCESS, "ihmc_footstep_planner_test");
      footstepPlanningModule = FootstepPlanningModuleLauncher.createModule(getRobotModel(), PubSubImplementation.INTRAPROCESS);

      footstepPlanningRequestPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node,
                                                                            FootstepPlanningRequestPacket.class,
                                                                            FootstepPlannerAPI.inputTopic(getSimpleRobotName()));
      footstepPlannerParametersPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node,
                                                                              FootstepPlannerParametersPacket.class,
                                                                              FootstepPlannerAPI.inputTopic(getSimpleRobotName()));

      toolboxStatePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, ToolboxStateMessage.class, FootstepPlannerAPI.inputTopic(getSimpleRobotName()));

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    FootstepPlanningToolboxOutputStatus.class,
                                                    FootstepPlannerAPI.outputTopic(getSimpleRobotName()),
                                                    s -> setOutputStatus(s.takeNextData()));

      FullHumanoidRobotModel fullHumanoidRobotModel = getRobotModel().createFullRobotModel();
      ForceSensorDataHolder forceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(fullHumanoidRobotModel.getForceSensorDefinitions()));
      humanoidRobotDataReceiver = new HumanoidRobotDataReceiver(fullHumanoidRobotModel, forceSensorDataHolder);
      planCompleted = false;

      simulationTestingParameters.setKeepSCSUp(!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer() && keepSCSUp);
      outputStatus = new AtomicReference<>(null);

      ros2Node.spin();
   }

   @AfterEach
   public void tearDown()
   {
      cinderBlockField = null;
      steppingStoneField = null;
      networkModuleParameters = null;

      ros2Node.destroy();
      footstepPlanningModule.closeAndDispose();

      ros2Node = null;
      footstepPlanningModule = null;

      planCompleted = false;

      humanoidRobotDataReceiver = null;

      if (networkProcessor != null)
      {
         networkProcessor.closeAndDispose();
         networkProcessor = null;
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Disabled
   @Test
   public void testShortCinderBlockFieldWithAStar()
   {
      double courseLength = CINDER_BLOCK_COURSE_WIDTH_X_IN_NUMBER_OF_BLOCKS * CINDER_BLOCK_SIZE + CINDER_BLOCK_FIELD_PLATFORM_LENGTH;
      DRCStartingLocation startingLocation = () -> new OffsetAndYawRobotInitialSetup(0.0, 0.0, 0.007);
      FramePose3D goalPose = new FramePose3D(ReferenceFrame.getWorldFrame(), new Pose3D(courseLength, 0.0, 0.0, 0.0, 0.0, 0.0));

      setupSimulation(cinderBlockField, startingLocation);
      simulationTestHelper.start();
      runEndToEndTestAndKeepSCSUpIfRequested(false, cinderBlockField, goalPose);
   }

   @Disabled
   @Test
   public void testShortCinderBlockFieldWithVisibilityGraph()
   {
      double courseLength = CINDER_BLOCK_COURSE_WIDTH_X_IN_NUMBER_OF_BLOCKS * CINDER_BLOCK_SIZE + CINDER_BLOCK_FIELD_PLATFORM_LENGTH;
      DRCStartingLocation startingLocation = () -> new OffsetAndYawRobotInitialSetup(0.0, 0.0, 0.007);
      FramePose3D goalPose = new FramePose3D(ReferenceFrame.getWorldFrame(), new Pose3D(courseLength, 0.0, 0.0, 0.0, 0.0, 0.0));

      setupSimulation(cinderBlockField, startingLocation);
      simulationTestHelper.start();
      runEndToEndTestAndKeepSCSUpIfRequested(true, cinderBlockField, goalPose);
   }

   @Tag("video")
   @Test
   public void testSteppingStonesWithAStar()
   {
      DRCStartingLocation startingLocation = () -> new OffsetAndYawRobotInitialSetup(0.0, -0.75, 0.007, 0.5 * Math.PI);
      FramePose3D goalPose = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                             new Pose3D(STEPPING_STONE_PATH_RADIUS + 0.5, STEPPING_STONE_PATH_RADIUS, 0.0, 0.0, 0.0, 0.0));

      setupSimulation(steppingStoneField, startingLocation);
      simulationTestHelper.start();
      boolean planBodyPath = false;

      runEndToEndTestAndKeepSCSUpIfRequested(planBodyPath, steppingStoneField, goalPose);
   }

   @Disabled
   @Test
   public void testWalkingOnFlatGround()
   {
      DRCStartingLocation startingLocation = () -> new OffsetAndYawRobotInitialSetup(-1.0, 0.0, 0.007, 0.0);
      FramePose3D goalPose = new FramePose3D();
      goalPose.setX(1.0);
      setupSimulation(flatGround, startingLocation);
      simulationTestHelper.start();
      boolean planBodyPath = false;

      runEndToEndTestAndKeepSCSUpIfRequested(planBodyPath, null, goalPose);
   }

   @Disabled
   @Test
   public void testWalkingBetweenBollardsAStarPlanner()
   {
      DRCStartingLocation startingLocation = () -> new OffsetAndYawRobotInitialSetup(-1.5, 0.0, 0.007, 0.0);
      FramePose3D goalPose = new FramePose3D();
      goalPose.setX(1.5);
      setupSimulation(bollardPlanarRegions, startingLocation);
      TimeConsumer collisionChecker = getCollisionChecker(500);
      simulationTestHelper.start();
      simulationTestHelper.getSimulationSession().addAfterPhysicsCallback(collisionChecker);

      FootstepPlannerParametersReadOnly parameters = getRobotModel().getFootstepPlannerParameters();
      FootstepPlannerParametersPacket parametersPacket = new FootstepPlannerParametersPacket();
      FootstepPlannerMessageTools.copyParametersToPacket(parametersPacket, parameters);
      parametersPacket.setCheckForBodyBoxCollisions(true);
      footstepPlannerParametersPublisher.publish(parametersPacket);
      boolean planBodyPath = false;

      runEndToEndTestAndKeepSCSUpIfRequested(planBodyPath, bollardPlanarRegions, goalPose);
   }

   private void runEndToEndTestAndKeepSCSUpIfRequested(boolean planBodyPath, PlanarRegionsList planarRegionsList, FramePose3D goalPose)
   {
      try
      {
         runEndToEndTest(planBodyPath, planarRegionsList, goalPose);
         if (simulationTestingParameters.getKeepSCSUp())
         {
            ThreadTools.sleepForever();
         }
      }
      catch (Exception e)
      {
         e.printStackTrace();
         if (simulationTestingParameters.getKeepSCSUp())
         {
            ThreadTools.sleepForever();
         }
         else
         {
            fail(e.getMessage());
         }
      }
   }

   private void setupSimulation(PlanarRegionsList planarRegionsList, DRCStartingLocation startingLocation)
   {
      CommonAvatarEnvironmentInterface simulationEnvironment = createCommonAvatarInterface(planarRegionsList);
      DRCRobotModel robotModel = getRobotModel();
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(robotModel,
                                                                                                                                             simulationEnvironment,
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(startingLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      new HumanoidNetworkProcessor(robotModel, PubSubImplementation.INTRAPROCESS);
      simulationTestHelper.createSubscriberFromController(RobotConfigurationData.class, humanoidRobotDataReceiver::receivedPacket);
      simulationTestHelper.createSubscriberFromController(WalkingStatusMessage.class, this::listenForWalkingComplete);
   }

   private void runEndToEndTest(boolean planBodyPath, PlanarRegionsList planarRegionsList, FramePose3D goalPose) throws Exception
   {
      ToolboxStateMessage wakeUpMessage = MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP);
      toolboxStatePublisher.publish(wakeUpMessage);
      ThreadTools.sleep(1000);

      while (!humanoidRobotDataReceiver.framesHaveBeenSetUp())
      {
         simulationTestHelper.simulateNow(1.0);
         humanoidRobotDataReceiver.updateRobotModel();
      }

      RobotSide initialStanceSide = RobotSide.LEFT;
      FramePose3D leftSolePose = new FramePose3D(humanoidRobotDataReceiver.getReferenceFrames().getSoleFrame(RobotSide.LEFT));
      FramePose3D rightSolePose = new FramePose3D(humanoidRobotDataReceiver.getReferenceFrames().getSoleFrame(RobotSide.RIGHT));
      leftSolePose.changeFrame(ReferenceFrame.getWorldFrame());
      rightSolePose.changeFrame(ReferenceFrame.getWorldFrame());

      YoGraphicsListRegistry graphicsListRegistry = createStartAndGoalGraphics(leftSolePose, goalPose);
      simulationTestHelper.addYoGraphicsListRegistry(graphicsListRegistry);
      double stanceWidth = footstepPlanningModule.getFootstepPlannerParameters().getIdealFootstepWidth();

      FootstepPlanningRequestPacket requestPacket = FootstepPlannerMessageTools.createFootstepPlanningRequestPacket(initialStanceSide,
                                                                                                                    leftSolePose,
                                                                                                                    rightSolePose,
                                                                                                                    goalPose,
                                                                                                                    stanceWidth,
                                                                                                                    planBodyPath);
      if (planarRegionsList != null)
      {
         PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList);
         requestPacket.getPlanarRegionsListMessage().set(planarRegionsListMessage);
      }

      footstepPlanningRequestPublisher.publish(requestPacket);

      while (outputStatus.get() == null)
      {
         simulationTestHelper.simulateNow(1.0);
      }

      FootstepPlanningToolboxOutputStatus outputStatus = this.outputStatus.get();
      if (!FootstepPlanningResult.fromByte(outputStatus.getFootstepPlanningResult()).validForExecution())
      {
         throw new RuntimeException("Footstep plan not valid for execution: " + outputStatus.getFootstepPlanningResult());
      }

      IHMCROS2Publisher<FootstepDataListMessage> publisher = simulationTestHelper.createPublisherForController(FootstepDataListMessage.class);

      planCompleted = false;
      if (outputStatus.getFootstepDataList().getFootstepDataList().size() > 0)
      {
         publisher.publish(outputStatus.getFootstepDataList());

         while (!planCompleted)
         {
            simulationTestHelper.simulateNow(1.0);
         }
      }

      SimFloatingJointBasics rootJoint = simulationTestHelper.getRobot().getFloatingRootJoint();
      Point3D rootJointPosition = new Point3D(rootJoint.getJointPose().getPosition());

      double errorThreshold = 0.3;
      double xPositionErrorMagnitude = Math.abs(rootJointPosition.getX() - goalPose.getX());
      double yPositionErrorMagnitude = Math.abs(rootJointPosition.getY() - goalPose.getY());
      assertTrue(xPositionErrorMagnitude < errorThreshold);
      assertTrue(yPositionErrorMagnitude < errorThreshold);
   }

   private YoGraphicsListRegistry createStartAndGoalGraphics(FramePose3D initialStancePose, FramePose3D goalPose)
   {
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      YoGraphicsList graphicsList = new YoGraphicsList("testViz");

      YoFramePoseUsingYawPitchRoll yoInitialStancePose = new YoFramePoseUsingYawPitchRoll("initialStancePose",
                                                                                          initialStancePose.getReferenceFrame(),
                                                                                          simulationTestHelper.getRootRegistry());
      yoInitialStancePose.set(initialStancePose);

      YoFramePoseUsingYawPitchRoll yoGoalPose = new YoFramePoseUsingYawPitchRoll("goalStancePose",
                                                                                 goalPose.getReferenceFrame(),
                                                                                 simulationTestHelper.getRootRegistry());
      yoGoalPose.set(goalPose);

      YoGraphicCoordinateSystem startPoseGraphics = new YoGraphicCoordinateSystem("startPose", yoInitialStancePose, 13.0);
      YoGraphicCoordinateSystem goalPoseGraphics = new YoGraphicCoordinateSystem("goalPose", yoGoalPose, 13.0);

      graphicsList.add(startPoseGraphics);
      graphicsList.add(goalPoseGraphics);
      return graphicsListRegistry;
   }

   private static CommonAvatarEnvironmentInterface createCommonAvatarInterface(PlanarRegionsList planarRegionsList)
   {
      double allowablePenetrationThickness = 0.05;
      boolean generateGroundPlane = false;
      return new PlanarRegionsListDefinedEnvironment("testEnvironment", planarRegionsList, allowablePenetrationThickness, generateGroundPlane);
   }

   private void listenForWalkingComplete(WalkingStatusMessage walkingStatusMessage)
   {
      if (walkingStatusMessage.getWalkingStatus() == WalkingStatus.COMPLETED.toByte())
      {
         planCompleted = true;
      }
   }

   private void setOutputStatus(FootstepPlanningToolboxOutputStatus packet)
   {
      outputStatus.set(packet);
   }

   public abstract class CollisionCheckerScript implements TimeConsumer
   {
      final int simTicksPerCollisionCheck;
      int counter = 0;

      public CollisionCheckerScript(int simTicksPerCollisionCheck)
      {
         this.simTicksPerCollisionCheck = simTicksPerCollisionCheck;
      }

      @Override
      public void accept(double time)
      {
         if (counter++ > simTicksPerCollisionCheck)
         {
            counter = 0;
            if (collisionDetected())
               fail();
         }
      }

      protected abstract boolean collisionDetected();
   }

   protected TimeConsumer getCollisionChecker(int simTicksPerCollisionCheck)
   {
      return new CollisionCheckerScript(simTicksPerCollisionCheck)
      {
         @Override
         protected boolean collisionDetected()
         {
            return false;
         }
      };
   }
}
