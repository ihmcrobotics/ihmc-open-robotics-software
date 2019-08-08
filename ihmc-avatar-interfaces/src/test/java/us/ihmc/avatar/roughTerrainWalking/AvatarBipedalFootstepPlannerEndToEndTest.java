package us.ihmc.avatar.roughTerrainWalking;

import controller_msgs.msg.dds.*;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule.FootstepPlanningToolboxModule;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.ToolboxState;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.communication.FootstepPlannerCommunicationProperties;
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
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.TwoBollardEnvironment;
import us.ihmc.simulationConstructionSetTools.util.planarRegions.PlanarRegionsListExamples;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.scripts.Script;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;

import java.io.IOException;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.robotics.Assert.*;

public abstract class AvatarBipedalFootstepPlannerEndToEndTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final boolean keepSCSUp = false;
   private static final int timeout = 120000; // to easily keep scs up. unfortunately can't be set programmatically, has to be a constant

   protected DRCSimulationTestHelper drcSimulationTestHelper;
   private DRCNetworkModuleParameters networkModuleParameters;
   protected HumanoidRobotDataReceiver humanoidRobotDataReceiver;

   private FootstepPlanningToolboxModule toolboxModule;
   private IHMCRealtimeROS2Publisher<FootstepPlanningRequestPacket> footstepPlanningRequestPublisher;
   private IHMCRealtimeROS2Publisher<ToolboxStateMessage> toolboxStatePublisher;
   private IHMCRealtimeROS2Publisher<FootstepPlannerParametersPacket> footstepPlannerParametersPublisher;

   private RealtimeRos2Node ros2Node;

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
   private BlockingSimulationRunner blockingSimulationRunner;

   @BeforeEach
   public void setup() throws IOException
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(CINDER_BLOCK_START_X, CINDER_BLOCK_START_Y, 0.001);
      PlanarRegionsListExamples.generateCinderBlockField(generator, CINDER_BLOCK_SIZE, CINDER_BLOCK_HEIGHT,
                                                                            CINDER_BLOCK_COURSE_WIDTH_X_IN_NUMBER_OF_BLOCKS,
                                                                            CINDER_BLOCK_COURSE_LENGTH_Y_IN_NUMBER_OF_BLOCKS, CINDER_BLOCK_HEIGHT_VARIATION);
      cinderBlockField = generator.getPlanarRegionsList();
      steppingStoneField = PlanarRegionsListExamples.generateSteppingStonesEnvironment(STEPPING_STONE_PATH_RADIUS);
      bollardPlanarRegions = bollardEnvironment.getPlanarRegionsList();

      generator.reset();
      generator.addRectangle(5.0, 5.0);
      flatGround = generator.getPlanarRegionsList();

      networkModuleParameters = new DRCNetworkModuleParameters();
      networkModuleParameters.enableFootstepPlanningToolbox(true);
      networkModuleParameters.enableLocalControllerCommunicator(true);
      networkModuleParameters.enableNetworkProcessor(true);

      ros2Node = ROS2Tools.createRealtimeRos2Node(PubSubImplementation.INTRAPROCESS, "ihmc_footstep_planner_test");
      toolboxModule = new FootstepPlanningToolboxModule(getRobotModel(), null, true, PubSubImplementation.INTRAPROCESS);
      footstepPlanningRequestPublisher = ROS2Tools.createPublisher(ros2Node, FootstepPlanningRequestPacket.class,
                                                                   FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(getSimpleRobotName()));
      footstepPlannerParametersPublisher = ROS2Tools.createPublisher(ros2Node, FootstepPlannerParametersPacket.class,
                                                                     FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(getSimpleRobotName()));

      toolboxStatePublisher = ROS2Tools
            .createPublisher(ros2Node, ToolboxStateMessage.class, FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(getSimpleRobotName()));

      ROS2Tools.createCallbackSubscription(ros2Node, FootstepPlanningToolboxOutputStatus.class,
                                           FootstepPlannerCommunicationProperties.publisherTopicNameGenerator(getSimpleRobotName()),
                                           s -> setOutputStatus(s.takeNextData()));

      FullHumanoidRobotModel fullHumanoidRobotModel = getRobotModel().createFullRobotModel();
      ForceSensorDataHolder forceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(fullHumanoidRobotModel.getForceSensorDefinitions()));
      humanoidRobotDataReceiver = new HumanoidRobotDataReceiver(fullHumanoidRobotModel, forceSensorDataHolder);
      planCompleted = false;

      simulationTestingParameters.setKeepSCSUp(keepSCSUp && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer());
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
      toolboxModule.destroy();

      ros2Node = null;
      toolboxModule = null;

      planCompleted = false;

      humanoidRobotDataReceiver = null;
      blockingSimulationRunner = null;

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Tag("humanoid-rough-terrain")
   @Disabled
   @Test
   public void testShortCinderBlockFieldWithAStar()
   {
      double courseLength = CINDER_BLOCK_COURSE_WIDTH_X_IN_NUMBER_OF_BLOCKS * CINDER_BLOCK_SIZE + CINDER_BLOCK_FIELD_PLATFORM_LENGTH;
      DRCStartingLocation startingLocation = () -> new OffsetAndYawRobotInitialSetup(0.0, 0.0, 0.007);
      FramePose3D goalPose = new FramePose3D(ReferenceFrame.getWorldFrame(), new Pose3D(courseLength, 0.0, 0.0, 0.0, 0.0, 0.0));

      setupSimulation(cinderBlockField, startingLocation);
      drcSimulationTestHelper.createSimulation("FootstepPlannerEndToEndTest");
      runEndToEndTestAndKeepSCSUpIfRequested(FootstepPlannerType.A_STAR, cinderBlockField, goalPose);
   }

   @Tag("humanoid-rough-terrain")
   @Disabled
   @Test
   public void testShortCinderBlockFieldWithVisibilityGraph()
   {
      double courseLength = CINDER_BLOCK_COURSE_WIDTH_X_IN_NUMBER_OF_BLOCKS * CINDER_BLOCK_SIZE + CINDER_BLOCK_FIELD_PLATFORM_LENGTH;
      DRCStartingLocation startingLocation = () -> new OffsetAndYawRobotInitialSetup(0.0, 0.0, 0.007);
      FramePose3D goalPose = new FramePose3D(ReferenceFrame.getWorldFrame(), new Pose3D(courseLength, 0.0, 0.0, 0.0, 0.0, 0.0));

      setupSimulation(cinderBlockField, startingLocation);
      drcSimulationTestHelper.createSimulation("FootstepPlannerEndToEndTest");
      runEndToEndTestAndKeepSCSUpIfRequested(FootstepPlannerType.VIS_GRAPH_WITH_A_STAR, cinderBlockField, goalPose);
   }

   @Tag("humanoid-rough-terrain")
   @Disabled
   @Test
   public void testShortCinderBlockFieldWithPlanarRegionBipedalPlanner()
   {
      double courseLength = CINDER_BLOCK_COURSE_WIDTH_X_IN_NUMBER_OF_BLOCKS * CINDER_BLOCK_SIZE + CINDER_BLOCK_FIELD_PLATFORM_LENGTH;
      DRCStartingLocation startingLocation = () -> new OffsetAndYawRobotInitialSetup(0.0, 0.0, 0.007);
      FramePose3D goalPose = new FramePose3D(ReferenceFrame.getWorldFrame(), new Pose3D(courseLength, 0.0, 0.0, 0.0, 0.0, 0.0));

      setupSimulation(cinderBlockField, startingLocation);
      drcSimulationTestHelper.createSimulation("FootstepPlannerEndToEndTest");
      runEndToEndTestAndKeepSCSUpIfRequested(FootstepPlannerType.PLANAR_REGION_BIPEDAL, cinderBlockField, goalPose);
   }

   @Tag("video")
   @Test
   public void testSteppingStonesWithAStar()
   {
      DRCStartingLocation startingLocation = () -> new OffsetAndYawRobotInitialSetup(0.0, -0.75, 0.007, 0.5 * Math.PI);
      FramePose3D goalPose = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                             new Pose3D(STEPPING_STONE_PATH_RADIUS + 0.5, STEPPING_STONE_PATH_RADIUS, 0.0, 0.0, 0.0, 0.0));

      setupSimulation(steppingStoneField, startingLocation);
      drcSimulationTestHelper.createSimulation("FootstepPlannerEndToEndTest");
      runEndToEndTestAndKeepSCSUpIfRequested(FootstepPlannerType.A_STAR, steppingStoneField, goalPose);
   }

   @Tag("humanoid-rough-terrain")
   @Disabled
   @Test
   public void testSteppingStonesWithPlanarRegionBipedalPlanner()
   {
      DRCStartingLocation startingLocation = () -> new OffsetAndYawRobotInitialSetup(0.0, -0.75, 0.007, 0.5 * Math.PI);
      FramePose3D goalPose = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                             new Pose3D(STEPPING_STONE_PATH_RADIUS + 0.5, STEPPING_STONE_PATH_RADIUS, 0.0, 0.0, 0.0, 0.0));

      setupSimulation(steppingStoneField, startingLocation);
      drcSimulationTestHelper.createSimulation("FootstepPlannerEndToEndTest");
      runEndToEndTestAndKeepSCSUpIfRequested(FootstepPlannerType.PLANAR_REGION_BIPEDAL, steppingStoneField, goalPose);
   }

   @Tag("humanoid-rough-terrain")
   @Disabled
   @Test
   public void testWalkingOnFlatGround()
   {
      DRCStartingLocation startingLocation = () -> new OffsetAndYawRobotInitialSetup(-1.0, 0.0, 0.007, 0.0);
      FramePose3D goalPose = new FramePose3D();
      goalPose.setX(1.0);
      setupSimulation(flatGround, startingLocation);
      drcSimulationTestHelper.createSimulation("FootstepPlannerEndToEndTest");
      runEndToEndTestAndKeepSCSUpIfRequested(FootstepPlannerType.A_STAR, null, goalPose);
   }

   @Tag("humanoid-rough-terrain")
   @Disabled
   @Test
   public void testWalkingBetweenBollardsAStarPlanner()
   {
      DRCStartingLocation startingLocation = () -> new OffsetAndYawRobotInitialSetup(-1.5, 0.0, 0.007, 0.0);
      FramePose3D goalPose = new FramePose3D();
      goalPose.setX(1.5);
      setupSimulation(bollardPlanarRegions, startingLocation);
      CollisionCheckerScript collisionChecker = getCollisionChecker(500);
      drcSimulationTestHelper.createSimulation("FootstepPlannerEndToEndTest");
      drcSimulationTestHelper.getSimulationConstructionSet().addScript(collisionChecker);

      FootstepPlannerParametersReadOnly parameters = getRobotModel().getFootstepPlannerParameters();
      FootstepPlannerParametersPacket parametersPacket = new FootstepPlannerParametersPacket();
      FootstepPlannerMessageTools.copyParametersToPacket(parametersPacket, parameters);
      parametersPacket.setCheckForBodyBoxCollisions(true);
      parametersPacket.setReturnBestEffortPlan(false);
      footstepPlannerParametersPublisher.publish(parametersPacket);

      runEndToEndTestAndKeepSCSUpIfRequested(FootstepPlannerType.A_STAR, bollardPlanarRegions, goalPose);
   }

   private void runEndToEndTestAndKeepSCSUpIfRequested(FootstepPlannerType plannerType, PlanarRegionsList planarRegionsList, FramePose3D goalPose)
   {
      try
      {
         runEndToEndTest(plannerType, planarRegionsList, goalPose);
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
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel);
      drcSimulationTestHelper.setTestEnvironment(simulationEnvironment);
      drcSimulationTestHelper.setNetworkProcessorParameters(networkModuleParameters);
      drcSimulationTestHelper.setStartingLocation(startingLocation);
      drcSimulationTestHelper.createSubscriberFromController(RobotConfigurationData.class, humanoidRobotDataReceiver::receivedPacket);
      drcSimulationTestHelper.createSubscriberFromController(WalkingStatusMessage.class, this::listenForWalkingComplete);
   }

   private void runEndToEndTest(FootstepPlannerType plannerType, PlanarRegionsList planarRegionsList, FramePose3D goalPose) throws Exception
   {
      blockingSimulationRunner = drcSimulationTestHelper.getBlockingSimulationRunner();
      ToolboxStateMessage wakeUpMessage = MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP);
      toolboxStatePublisher.publish(wakeUpMessage);
      ThreadTools.sleep(1000);

      while (!humanoidRobotDataReceiver.framesHaveBeenSetUp())
      {
         blockingSimulationRunner.simulateAndBlock(1.0);
         humanoidRobotDataReceiver.updateRobotModel();
      }

      ReferenceFrame soleFrame = humanoidRobotDataReceiver.getReferenceFrames().getSoleFrame(RobotSide.LEFT);
      FramePose3D initialStancePose = new FramePose3D(soleFrame, new Point3D(0.0, 0.0, 0.001), new AxisAngle());
      initialStancePose.changeFrame(ReferenceFrame.getWorldFrame());
      RobotSide initialStanceSide = RobotSide.LEFT;

      YoGraphicsListRegistry graphicsListRegistry = createStartAndGoalGraphics(initialStancePose, goalPose);
      drcSimulationTestHelper.getSimulationConstructionSet().addYoGraphicsListRegistry(graphicsListRegistry);

      FootstepPlanningRequestPacket requestPacket = FootstepPlannerMessageTools.createFootstepPlanningRequestPacket(initialStancePose, initialStanceSide, goalPose,
                                                 plannerType);
      if(planarRegionsList != null)
      {
         PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList);
         requestPacket.getPlanarRegionsListMessage().set(planarRegionsListMessage);
      }

      footstepPlanningRequestPublisher.publish(requestPacket);

      while (outputStatus.get() == null)
      {
         blockingSimulationRunner.simulateAndBlock(1.0);
      }

      FootstepPlanningToolboxOutputStatus outputStatus = this.outputStatus.get();
      if (!FootstepPlanningResult.fromByte(outputStatus.getFootstepPlanningResult()).validForExecution())
      {
         throw new RuntimeException("Footstep plan not valid for execution: " + outputStatus.getFootstepPlanningResult());
      }

      IHMCROS2Publisher<FootstepDataListMessage> publisher = drcSimulationTestHelper.createPublisherForController(FootstepDataListMessage.class);

      planCompleted = false;
      if (outputStatus.getFootstepDataList().getFootstepDataList().size() > 0)
      {
         publisher.publish(outputStatus.getFootstepDataList());

         while (!planCompleted)
         {
            blockingSimulationRunner.simulateAndBlock(1.0);
         }
      }

      FloatingJoint rootJoint = drcSimulationTestHelper.getRobot().getRootJoint();
      Point3D rootJointPosition = new Point3D();
      rootJoint.getPosition(rootJointPosition);

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

      YoFramePoseUsingYawPitchRoll yoInitialStancePose = new YoFramePoseUsingYawPitchRoll("initialStancePose", initialStancePose.getReferenceFrame(),
                                                                                          drcSimulationTestHelper.getYoVariableRegistry());
      yoInitialStancePose.set(initialStancePose);

      YoFramePoseUsingYawPitchRoll yoGoalPose = new YoFramePoseUsingYawPitchRoll("goalStancePose", goalPose.getReferenceFrame(),
                                                                                 drcSimulationTestHelper.getYoVariableRegistry());
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

   public abstract class CollisionCheckerScript implements Script
   {
      final int simTicksPerCollisionCheck;
      int counter = 0;

      public CollisionCheckerScript(int simTicksPerCollisionCheck)
      {
         this.simTicksPerCollisionCheck = simTicksPerCollisionCheck;
      }

      @Override
      public void doScript(double t)
      {
         if(counter++ > simTicksPerCollisionCheck)
         {
            counter = 0;
            if(collisionDetected())
               fail();
         }
      }

      protected abstract boolean collisionDetected();
   }

   protected CollisionCheckerScript getCollisionChecker(int simTicksPerCollisionCheck)
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
