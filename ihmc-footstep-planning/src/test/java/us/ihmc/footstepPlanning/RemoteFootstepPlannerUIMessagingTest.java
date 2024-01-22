package us.ihmc.footstepPlanning;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.IntStream;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import perception_msgs.msg.dds.HeightMapMessage;
import toolbox_msgs.msg.dds.FootstepPlannerParametersPacket;
import toolbox_msgs.msg.dds.FootstepPlanningRequestPacket;
import toolbox_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import toolbox_msgs.msg.dds.VisibilityGraphsParametersPacket;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.ExecutionTiming;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.tools.PlanarRegionToHeightMapConverter;
import us.ihmc.footstepPlanning.ui.FootstepPlannerUI;
import us.ihmc.footstepPlanning.ui.RemoteUIMessageConverter;
import us.ihmc.javaFXToolkit.starter.ApplicationRunner;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.messager.javafx.SharedMemoryJavaFXMessager;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;

public class RemoteFootstepPlannerUIMessagingTest
{
   private static final double epsilon = 1e-5;

   private static final boolean VISUALIZE = false;
   private static final String robotName = "testBot";

   private RealtimeROS2Node localNode = null;
   private RemoteUIMessageConverter messageConverter = null;
   private SharedMemoryMessager messager = null;
   private DomainFactory.PubSubImplementation pubSubImplementation = null;

   private final AtomicReference<FootstepPlanningRequestPacket> planningRequestReference = new AtomicReference<>(null);
   private final AtomicReference<FootstepPlannerParametersPacket> footstepPlannerParametersReference = new AtomicReference<>(null);
   private final AtomicReference<VisibilityGraphsParametersPacket> visibilityGraphsParametersReference = new AtomicReference<>(null);

   @AfterEach
   public void tearDown() throws Exception
   {
      localNode.destroy();
      messager.closeMessager();
      messageConverter.destroy();

      if (ui != null)
         ui.stop();
      ui = null;

      messager = null;
      messageConverter = null;
      localNode = null;
      pubSubImplementation = null;

      planningRequestReference.set(null);
   }

   private FootstepPlannerUI ui;

   public void setup()
   {
      localNode = ROS2Tools.createRealtimeROS2Node(pubSubImplementation, "ihmc_footstep_planner_test");
      if (VISUALIZE)
         messager = new SharedMemoryJavaFXMessager(FootstepPlannerMessagerAPI.API);
      else
         messager = new SharedMemoryMessager(FootstepPlannerMessagerAPI.API);
      messageConverter = RemoteUIMessageConverter.createConverter(messager, robotName, pubSubImplementation);

      try
      {
         messager.startMessager();
      }
      catch (Exception e)
      {
         throw new RuntimeException("Failed to start messager.");
      }

      if (VISUALIZE)
      {
         ApplicationRunner.runApplication(new Application()
         {
            @Override
            public void start(Stage stage) throws Exception
            {
               ui = FootstepPlannerUI.createUI(stage, (SharedMemoryJavaFXMessager) messager);
               ui.show();
            }

            @Override
            public void stop()
            {
               ui.stop();
               Platform.exit();
            }
         });
      }
   }

   @Test
   public void testSendingFootstepPlanningRequestPacketFromUIIntraprocess()
   {
      pubSubImplementation = DomainFactory.PubSubImplementation.INTRAPROCESS;
      setup();
      runPlanningRequestTestFromUI();
   }

   @Test
   public void testSendingFootstepPlanningRequestPacketFromUIFastRTPS()
   {
      pubSubImplementation = DomainFactory.PubSubImplementation.FAST_RTPS;
      setup();
      runPlanningRequestTestFromUI();
   }

   @Test
   public void testSendingFootstepPlannerRequestPacketToUIIntraprocess()
   {
      pubSubImplementation = DomainFactory.PubSubImplementation.INTRAPROCESS;
      setup();
      runPlannerRequestToUI();
   }

   @Test
   public void testSendingFootstepPlannerRequestPacketToUIFastRTPS()
   {
      pubSubImplementation = DomainFactory.PubSubImplementation.FAST_RTPS;
      setup();
      runPlannerRequestToUI();
   }

   @Test
   public void testSendingPlanObjectivePacketIntraprocess()
   {
      pubSubImplementation = DomainFactory.PubSubImplementation.INTRAPROCESS;
      setup();
      runPlanObjectivePackets();
   }

   @Test
   public void testSendingPlanObjectivePacketFastRTPS()
   {
      pubSubImplementation = DomainFactory.PubSubImplementation.FAST_RTPS;
      setup();
      runPlanObjectivePackets();
   }

   @Test
   public void testSendingFootstepPlannerOutputStatusToUIIntraprocess()
   {
      pubSubImplementation = DomainFactory.PubSubImplementation.INTRAPROCESS;
      setup();
      runOutputStatusToUI();
   }

   @Test
   public void testSendingFootstepPlannerOutputStatusToUIFastRTPS()
   {
      pubSubImplementation = DomainFactory.PubSubImplementation.FAST_RTPS;
      setup();
      runOutputStatusToUI();
   }

   private void runPlanningRequestTestFromUI()
   {
      Random random = new Random(1738L);
      ROS2Tools.createCallbackSubscriptionTypeNamed(localNode, FootstepPlanningRequestPacket.class,
                                                    ROS2Tools.FOOTSTEP_PLANNER.withRobot(robotName)
                                                              .withInput(),
                                           s -> processFootstepPlanningRequestPacket(s.takeNextData()));
      localNode.spin();

      double timeout = RandomNumbers.nextDouble(random, 0.1, 100.0);
      int maxIterations = RandomNumbers.nextInt(random, 0, 100);
      double horizonLength = RandomNumbers.nextDouble(random, 0.1, 10.0);

      Pose3D startLeftFootPose = new Pose3D(EuclidCoreRandomTools.nextRigidBodyTransform(random));
      Pose3D startRightFootPose = new Pose3D(EuclidCoreRandomTools.nextRigidBodyTransform(random));
      Pose3D goalLeftFootPose = new Pose3D(EuclidCoreRandomTools.nextRigidBodyTransform(random));
      Pose3D goalRightFootPose = new Pose3D(EuclidCoreRandomTools.nextRigidBodyTransform(random));

      boolean planBodyPath = random.nextBoolean();
      boolean performAStarSearch = random.nextBoolean();

      RobotSide robotSide = RobotSide.generateRandomRobotSide(random);
      PlanarRegionsList planarRegionsList = createRandomPlanarRegionList(random);
      HeightMapMessage heightMapMessage = PlanarRegionToHeightMapConverter.convertFromPlanarRegionsToHeightMap(planarRegionsList);
      HeightMapData heightMapData = HeightMapMessageTools.unpackMessage(heightMapMessage);
      int plannerRequestId = RandomNumbers.nextInt(random, 1, 100);

      messager.submitMessage(FootstepPlannerMessagerAPI.LeftFootPose, startLeftFootPose);
      messager.submitMessage(FootstepPlannerMessagerAPI.RightFootPose, startRightFootPose);
      messager.submitMessage(FootstepPlannerMessagerAPI.LeftFootGoalPose, goalLeftFootPose);
      messager.submitMessage(FootstepPlannerMessagerAPI.RightFootGoalPose, goalRightFootPose);
      messager.submitMessage(FootstepPlannerMessagerAPI.PlanBodyPath, planBodyPath);
      messager.submitMessage(FootstepPlannerMessagerAPI.PerformAStarSearch, performAStarSearch);
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerTimeout, timeout);
      messager.submitMessage(FootstepPlannerMessagerAPI.MaxIterations, maxIterations);
      messager.submitMessage(FootstepPlannerMessagerAPI.HeightMapData, heightMapMessage);
      messager.submitMessage(FootstepPlannerMessagerAPI.InitialSupportSide, robotSide);
      messager.submitMessage(FootstepPlannerMessagerAPI.SnapGoalSteps, random.nextBoolean());
      messager.submitMessage(FootstepPlannerMessagerAPI.AbortIfGoalStepSnapFails, random.nextBoolean());
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerRequestId, plannerRequestId);
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerHorizonLength, horizonLength);

      messager.submitMessage(FootstepPlannerMessagerAPI.ComputePath, true);

      int ticks = 0;
      while (planningRequestReference.get() == null)
      {
         ticks++;
         if (ticks > 100)
            fail();

         ThreadTools.sleep(10);
      }

      FootstepPlanningRequestPacket packet = planningRequestReference.getAndSet(null);

      assertTrue(packet.getStartLeftFootPose().epsilonEquals(startLeftFootPose, epsilon), "Left foot poses aren't equal");
      assertTrue(packet.getStartRightFootPose().epsilonEquals(startRightFootPose, epsilon), "Right foot poses aren't equal");
      assertTrue(packet.getGoalLeftFootPose().epsilonEquals(goalLeftFootPose, epsilon), "Left goal foot poses aren't equal");
      assertTrue(packet.getGoalRightFootPose().epsilonEquals(goalRightFootPose, epsilon), "Right goal foot poses aren't equal");
      assertEquals(timeout, packet.getTimeout(), 1e-5, "Timeouts aren't equal.");
      assertEquals(performAStarSearch, packet.getPerformAStarSearch(), "Perform A* search flags aren't.");
      assertEquals(planBodyPath, packet.getPlanBodyPath(), "Plan body path flags aren't equal.");
      assertEquals(robotSide, RobotSide.fromByte(packet.getRequestedInitialStanceSide()), "Initial support sides aren't equal.");

      assertEquals(plannerRequestId, packet.getPlannerRequestId(), epsilon, "Planner Request Ids aren't equal.");
      assertEquals(horizonLength, packet.getHorizonLength(), epsilon, "Planner horizon lengths aren't equal.");

      checkHeightMapDataAreEqual(heightMapData, HeightMapMessageTools.unpackMessage(packet.getHeightMapMessage()));
   }

   private void runPlannerRequestToUI()
   {
      Random random = new Random(1738L);
      IHMCRealtimeROS2Publisher<FootstepPlanningRequestPacket> footstepPlanningRequestPublisher = ROS2Tools
            .createPublisherTypeNamed(localNode, FootstepPlanningRequestPacket.class,
                                      ROS2Tools.FOOTSTEP_PLANNER.withRobot(robotName).withInput());
      localNode.spin();

      AtomicReference<Pose3DReadOnly> leftFootPoseReference = messager.createInput(FootstepPlannerMessagerAPI.LeftFootPose);
      AtomicReference<Pose3DReadOnly> rightFootPoseReference = messager.createInput(FootstepPlannerMessagerAPI.RightFootPose);
      AtomicReference<Pose3DReadOnly> leftFootGoalPoseReference = messager.createInput(FootstepPlannerMessagerAPI.LeftFootGoalPose);
      AtomicReference<Pose3DReadOnly> rightFootGoalPoseReference = messager.createInput(FootstepPlannerMessagerAPI.RightFootGoalPose);

      AtomicReference<Boolean> planBodyPathReference = messager.createInput(FootstepPlannerMessagerAPI.PlanBodyPath);
      AtomicReference<Boolean> performAStarSearchReference = messager.createInput(FootstepPlannerMessagerAPI.PerformAStarSearch);
      AtomicReference<Double> timeoutReference = messager.createInput(FootstepPlannerMessagerAPI.PlannerTimeout);
      AtomicReference<RobotSide> robotSideReference = messager.createInput(FootstepPlannerMessagerAPI.InitialSupportSide);
      AtomicReference<Integer> plannerRequestIdReference = messager.createInput(FootstepPlannerMessagerAPI.PlannerRequestId);
      AtomicReference<Double> plannerHorizonLengthReference = messager.createInput(FootstepPlannerMessagerAPI.PlannerHorizonLength);

      double timeout = RandomNumbers.nextDouble(random, 0.1, 100.0);
      double horizonLength = RandomNumbers.nextDouble(random, 0.1, 10.0);
      int sequenceId = RandomNumbers.nextInt(random, 1, 100);
      int plannerRequestId = RandomNumbers.nextInt(random, 1, 100);
      boolean planBodyPath = random.nextBoolean();
      boolean performAStarSearch = random.nextBoolean();

      RobotSide robotSide = RobotSide.generateRandomRobotSide(random);
      PlanarRegionsList planarRegionsList = createRandomPlanarRegionList(random);
      HeightMapMessage heightMapMessage = PlanarRegionToHeightMapConverter.convertFromPlanarRegionsToHeightMap(planarRegionsList);
      Pose3D leftFootPose = new Pose3D(EuclidCoreRandomTools.nextRigidBodyTransform(random));
      Pose3D rightFootPose = new Pose3D(EuclidCoreRandomTools.nextRigidBodyTransform(random));
      Pose3D leftFootGoalPose = new Pose3D(EuclidCoreRandomTools.nextRigidBodyTransform(random));
      Pose3D rightFootGoalPose = new Pose3D(EuclidCoreRandomTools.nextRigidBodyTransform(random));

      FootstepPlanningRequestPacket packet = new FootstepPlanningRequestPacket();
      packet.setPerformAStarSearch(performAStarSearch);
      packet.setPlanBodyPath(planBodyPath);
      packet.getStartLeftFootPose().set(leftFootPose);
      packet.getStartRightFootPose().set(rightFootPose);
      packet.getGoalLeftFootPose().set(leftFootGoalPose);
      packet.getGoalRightFootPose().set(rightFootGoalPose);
      packet.setTimeout(timeout);
      packet.setRequestedInitialStanceSide(robotSide.toByte());
      packet.setPlannerRequestId(plannerRequestId);
      packet.setSequenceId(sequenceId);
      packet.setHorizonLength(horizonLength);
      packet.getHeightMapMessage().set(heightMapMessage);

      footstepPlanningRequestPublisher.publish(packet);

      double maxWaitTime = 5.0;
      double currentWaitTime = 0.0;
      long sleepDuration = 10;
      while (leftFootPoseReference.get() == null || rightFootPoseReference.get() == null || leftFootGoalPoseReference.get() == null
             || rightFootGoalPoseReference.get() == null || timeoutReference.get() == null || performAStarSearchReference.get() == null
             || planBodyPathReference.get() == null || robotSideReference.get() == null || plannerRequestIdReference.get() == null
             || plannerHorizonLengthReference.get() == null)
      {
         assertFalse(currentWaitTime > maxWaitTime, "Timed out waiting on the results.");

         ThreadTools.sleep(sleepDuration);
         currentWaitTime += Conversions.millisecondsToSeconds(sleepDuration);
      }

      Assertions.assertTrue(leftFootPose.epsilonEquals(leftFootPoseReference.get(), epsilon), "leftFootPose values aren't equal");
      Assertions.assertTrue(rightFootPose.epsilonEquals(rightFootPoseReference.get(), epsilon), "rightFootPose values aren't equal");
      Assertions.assertTrue(leftFootGoalPose.epsilonEquals(leftFootGoalPoseReference.get(), epsilon), "leftFootGoalPose values aren't equal");
      Assertions.assertTrue(rightFootGoalPose.epsilonEquals(rightFootGoalPoseReference.get(), epsilon), "rightFootGoalPose values aren't equal");
      assertEquals(timeout, timeoutReference.getAndSet(null), epsilon, "Timeouts aren't equal.");
      assertEquals(performAStarSearch, performAStarSearchReference.getAndSet(null), "Perform A* search flags aren't equal.");
      assertEquals(robotSide, robotSideReference.getAndSet(null), "Initial support sides aren't equal.");
      assertEquals(plannerRequestId, plannerRequestIdReference.getAndSet(null), epsilon, "Planner Request Ids aren't equal.");
      assertEquals(horizonLength, plannerHorizonLengthReference.getAndSet(null), epsilon, "Planner horizon lengths aren't equal.");
   }

   private void runPlanObjectivePackets()
   {
      Random random = new Random(1738L);
      ROS2Tools.createCallbackSubscriptionTypeNamed(localNode, FootstepPlannerParametersPacket.class,
                                                    ROS2Tools.FOOTSTEP_PLANNER.withRobot(robotName)
                                                              .withInput(),
                                           s -> processFootstepPlannerParametersPacket(s.takeNextData()));
      ROS2Tools.createCallbackSubscriptionTypeNamed(localNode, VisibilityGraphsParametersPacket.class,
                                                    ROS2Tools.FOOTSTEP_PLANNER.withRobot(robotName)
                                                              .withInput(),
                                           s -> processVisibilityGraphsParametersPacket(s.takeNextData()));
      localNode.spin();

      FootstepPlannerParametersBasics randomParameters = FootstepPlanningTestTools.createRandomParameters(random);
      VisibilityGraphsParametersReadOnly randomVisibilityGraphParameters = createRandomVisibilityGraphsParameters(random);

      double timeout = RandomNumbers.nextDouble(random, 0.1, 100.0);
      int maxIterations = RandomNumbers.nextInt(random, 0, 100);
      double horizonLength = RandomNumbers.nextDouble(random, 0.1, 10);
      boolean planBodyPath = random.nextBoolean();
      boolean performAStarSearch = random.nextBoolean();
      Pose3D leftFootPose = new Pose3D(EuclidCoreRandomTools.nextRigidBodyTransform(random));
      Pose3D rightFootPose = new Pose3D(EuclidCoreRandomTools.nextRigidBodyTransform(random));
      Pose3D leftFootGoalPose = new Pose3D(EuclidCoreRandomTools.nextRigidBodyTransform(random));
      Pose3D rightFootGoalPose = new Pose3D(EuclidCoreRandomTools.nextRigidBodyTransform(random));
      RobotSide robotSide = RobotSide.generateRandomRobotSide(random);
      PlanarRegionsList planarRegionsList = createRandomPlanarRegionList(random);
      int plannerRequestId = RandomNumbers.nextInt(random, 1, 100);

      messager.submitMessage(FootstepPlannerMessagerAPI.LeftFootGoalPose, leftFootGoalPose);
      messager.submitMessage(FootstepPlannerMessagerAPI.RightFootGoalPose, rightFootGoalPose);
      messager.submitMessage(FootstepPlannerMessagerAPI.LeftFootPose, leftFootPose);
      messager.submitMessage(FootstepPlannerMessagerAPI.RightFootPose, rightFootPose);
      messager.submitMessage(FootstepPlannerMessagerAPI.PlanBodyPath, planBodyPath);
      messager.submitMessage(FootstepPlannerMessagerAPI.PerformAStarSearch, performAStarSearch);
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerTimeout, timeout);
      messager.submitMessage(FootstepPlannerMessagerAPI.MaxIterations,maxIterations);
      messager.submitMessage(FootstepPlannerMessagerAPI.SnapGoalSteps, random.nextBoolean());
      messager.submitMessage(FootstepPlannerMessagerAPI.AbortIfGoalStepSnapFails, random.nextBoolean());
      messager.submitMessage(FootstepPlannerMessagerAPI.PlanarRegionData, planarRegionsList);
      messager.submitMessage(FootstepPlannerMessagerAPI.InitialSupportSide, robotSide);
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerRequestId, plannerRequestId);
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerHorizonLength, horizonLength);
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerParameters, randomParameters);
      messager.submitMessage(FootstepPlannerMessagerAPI.VisibilityGraphsParameters, randomVisibilityGraphParameters);

      messager.submitMessage(FootstepPlannerMessagerAPI.ComputePath, true);

      int ticks = 0;
      while (footstepPlannerParametersReference.get() == null)
      {
         ticks++;
         assertTrue(ticks < 100, "Timed out waiting for packet.");

         ThreadTools.sleep(10);
      }

      FootstepPlannerParametersPacket plannerPacket = footstepPlannerParametersReference.getAndSet(null);
      VisibilityGraphsParametersPacket visibilityGraphsPacket = visibilityGraphsParametersReference.getAndSet(null);

      checkFootstepPlannerParameters(randomParameters, plannerPacket);
      checkVisibilityGraphsParameters(randomVisibilityGraphParameters, visibilityGraphsPacket);
   }



   private void runOutputStatusToUI()
   {
      Random random = new Random(1738L);
      IHMCRealtimeROS2Publisher<FootstepPlanningToolboxOutputStatus> footstepOutputStatusPublisher = ROS2Tools
            .createPublisherTypeNamed(localNode, FootstepPlanningToolboxOutputStatus.class,
                                      ROS2Tools.FOOTSTEP_PLANNER.withRobot(robotName).withOutput());

      localNode.spin();
      AtomicReference<PlanarRegionsList> planarRegionsListReference = messager.createInput(FootstepPlannerMessagerAPI.PlanarRegionData);
      AtomicReference<FootstepDataListMessage> footstepPlanReference = messager.createInput(FootstepPlannerMessagerAPI.FootstepPlanResponse);
      AtomicReference<Integer> receivedPlanIdReference = messager.createInput(FootstepPlannerMessagerAPI.ReceivedPlanId);
      AtomicReference<FootstepPlanningResult> plannerResultReference = messager.createInput(FootstepPlannerMessagerAPI.FootstepPlanningResultTopic);
      AtomicReference<Point3D> lowLevelPositionGoalReference = messager.createInput(FootstepPlannerMessagerAPI.LowLevelGoalPosition);
      AtomicReference<Quaternion> lowLevelOrientationGoalReference = messager.createInput(FootstepPlannerMessagerAPI.LowLevelGoalOrientation);

      Pose2D goalPose = new Pose2D();
      goalPose.getPosition().set(EuclidCoreRandomTools.nextPoint2D(random));
      goalPose.getOrientation().set(EuclidCoreRandomTools.nextQuaternion(random));
      PlanarRegionsList planarRegionsList = createRandomPlanarRegionList(random);
      HeightMapMessage heightMapMessage = PlanarRegionToHeightMapConverter.convertFromPlanarRegionsToHeightMap(planarRegionsList);
      FootstepDataListMessage footstepDataListMessage = nextFootstepDataListMessage(random);
      int sequenceId = RandomNumbers.nextInt(random, 0, 100);
      int planId = RandomNumbers.nextInt(random, 0, 100);
      FootstepPlanningResult result = FootstepPlanningResult.generateRandomResult(random);
      List<Pose3DReadOnly> bodyPath = new ArrayList<>();
      for (int i = 2; i < RandomNumbers.nextInt(random, 2, 100); i++)
      {
         Pose3D pose = new Pose3D();
         pose.getPosition().set(EuclidCoreRandomTools.nextPoint3D(random, 100.0));
         pose.getOrientation().set(EuclidCoreRandomTools.nextQuaternion(random, 100.0));
         bodyPath.add(pose);
      }
      Point3D lowLevelGoalPosition = EuclidCoreRandomTools.nextPoint3D(random, 100.0);
      Quaternion lowLevelGoalOrientation = EuclidCoreRandomTools.nextQuaternion(random, 100.0);

      FootstepPlanningToolboxOutputStatus outputPacket = new FootstepPlanningToolboxOutputStatus();
      outputPacket.getGoalPose().set(goalPose);
      outputPacket.getHeightMapMessage().set(heightMapMessage);
      outputPacket.getFootstepDataList().set(footstepDataListMessage);
      outputPacket.setPlanId(planId);
      outputPacket.setSequenceId(sequenceId);
      outputPacket.setFootstepPlanningResult(result.toByte());
      for (int i = 0; i < bodyPath.size(); i++)
         outputPacket.getBodyPath().add().set(bodyPath.get(i));
      outputPacket.getGoalPose().getPosition().set(lowLevelGoalPosition);
      outputPacket.getGoalPose().getOrientation().set(lowLevelGoalOrientation);

      footstepOutputStatusPublisher.publish(outputPacket);

      int ticks = 0;
      while (planarRegionsListReference.get() == null && footstepPlanReference.get() == null)
      {
         ticks++;
         assertTrue(ticks < 100, "Timed out waiting on messages.");
         ThreadTools.sleep(100);
      }

      checkFootstepPlansAreEqual(footstepDataListMessage, footstepPlanReference.getAndSet(null));
      assertEquals(planId, receivedPlanIdReference.getAndSet(null), epsilon, "Planner Ids aren't equal.");
      assertEquals(result, plannerResultReference.getAndSet(null), "Planner results aren't equal.");
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals("Low level goal position results aren't equal.", lowLevelGoalPosition, lowLevelPositionGoalReference.getAndSet(null), epsilon);
      EuclidCoreTestTools.assertOrientation3DGeometricallyEquals("Low level goal orientation results aren't equal.", lowLevelGoalOrientation, lowLevelOrientationGoalReference.getAndSet(null), epsilon);
   }

   private void processFootstepPlanningRequestPacket(FootstepPlanningRequestPacket packet)
   {
      planningRequestReference.set(packet);
   }

   private void processFootstepPlannerParametersPacket(FootstepPlannerParametersPacket packet)
   {
      footstepPlannerParametersReference.set(packet);
   }

   private void processVisibilityGraphsParametersPacket(VisibilityGraphsParametersPacket packet)
   {
      visibilityGraphsParametersReference.set(packet);
   }

   private static PlanarRegionsList createRandomPlanarRegionList(Random random)
   {
      PlanarRegionsList planarRegionsList = new PlanarRegionsList();
      for (int i = 0; i < RandomNumbers.nextInt(random, 1, 50); i++)
         planarRegionsList.addPlanarRegion(createRandomPlanarRegion(random, i));

      return planarRegionsList;
   }

   private static PlanarRegion createRandomPlanarRegion(Random random, int idNumber)
   {
      int numberOfVertices = RandomNumbers.nextInt(random, 3, 50);
      int numberOfPolygons = RandomNumbers.nextInt(random, 1, 5);
      List<ConvexPolygon2D> polygons = new ArrayList<>();
      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      for (int i = 0; i < numberOfPolygons; i++)
      {
         polygons.add(EuclidGeometryRandomTools.nextConvexPolygon2D(random, 10.0, numberOfVertices));
      }
      PlanarRegion planarRegion = new PlanarRegion(transform, polygons);
      planarRegion.setRegionId(idNumber);

      return planarRegion;
   }

   private static FootstepDataListMessage nextFootstepDataListMessage(Random random)
   {
      FootstepDataListMessage next = new FootstepDataListMessage();
      MessageTools.copyData(nextFootstepDataMessages(random), next.getFootstepDataList());
      next.setExecutionTiming(RandomNumbers.nextEnum(random, ExecutionTiming.class).toByte());
      next.setDefaultSwingDuration(RandomNumbers.nextDoubleWithEdgeCases(random, 0.1));
      next.setDefaultTransferDuration(RandomNumbers.nextDoubleWithEdgeCases(random, 0.1));
      next.setFinalTransferDuration(RandomNumbers.nextDoubleWithEdgeCases(random, 0.1));
      next.setTrustHeightOfFootsteps(random.nextBoolean());
      next.setAreFootstepsAdjustable(random.nextBoolean());
      next.setOffsetFootstepsWithExecutionError(random.nextBoolean());
      return next;
   }

   private static ArrayList<FootstepDataMessage> nextFootstepDataMessages(Random random)
   {
      return nextFootstepDataMessages(random, random.nextInt(16) + 1);
   }

   private static ArrayList<FootstepDataMessage> nextFootstepDataMessages(Random random, int length)
   {
      ArrayList<FootstepDataMessage> next = new ArrayList<>();
      for (int i = 0; i < length; i++)
         next.add(nextFootstepDataMessage(random));
      return next;
   }

   private static FootstepDataMessage nextFootstepDataMessage(Random random)
   {
      FootstepDataMessage next = new FootstepDataMessage();
      next.setRobotSide(RobotSide.generateRandomRobotSide(random).toByte());
      next.getLocation().set(EuclidCoreRandomTools.nextPoint3D(random));
      next.getOrientation().set(EuclidCoreRandomTools.nextQuaternion(random));
      IntStream.range(0, random.nextInt(10)).forEach(i -> next.getPredictedContactPoints2d().add().set(EuclidCoreRandomTools.nextPoint2D(random)));
      next.setTrajectoryType(RandomNumbers.nextEnum(random, TrajectoryType.class).toByte());
      next.setSwingHeight(RandomNumbers.nextDoubleWithEdgeCases(random, 0.1));
      next.setSwingTrajectoryBlendDuration(RandomNumbers.nextDoubleWithEdgeCases(random, 0.1));
      next.setSwingDuration(RandomNumbers.nextDoubleWithEdgeCases(random, 0.1));
      next.setTransferDuration(RandomNumbers.nextDoubleWithEdgeCases(random, 0.1));
      next.setTouchdownDuration(RandomNumbers.nextDoubleWithEdgeCases(random, 0.1));
      next.setExecutionDelayTime(RandomNumbers.nextDoubleWithEdgeCases(random, 0.1));
      return next;
   }



   private static VisibilityGraphsParametersReadOnly createRandomVisibilityGraphsParameters(Random random)
   {
      double maxInterRegionConnectionLength = RandomNumbers.nextDouble(random, 0.01, 5.0);
      double normalZThresholdForAccessibleRegions = RandomNumbers.nextDouble(random, 0.01, 2.0);
      double extrusionDistance = RandomNumbers.nextDouble(random, 0.01, 1.0);
      double extrusionDistanceIfNotTooHighToStep = RandomNumbers.nextDouble(random, 0.01, 1.0);
      double tooHighToStepDistance = RandomNumbers.nextDouble(random, 0.01, 0.5);
      double clusterResolution = RandomNumbers.nextDouble(random, 0.01, 1.0);
      double explorationDistance = RandomNumbers.nextDouble(random, 1.0, 100.0);
      double planarRegionMinArea = RandomNumbers.nextDouble(random, 0.01, 0.5);
      int planarRegionMinSize = RandomNumbers.nextInt(random, 0, 100);
      double regionOrthogonalAngle = RandomNumbers.nextDouble(random, 0.0, Math.PI / 2.0);
      double searchHostRegionEpsilon = RandomNumbers.nextDouble(random, 0.0, 0.5);

      VisibilityGraphsParametersBasics parameters = new DefaultVisibilityGraphParameters();
      parameters.setMaxInterRegionConnectionLength(maxInterRegionConnectionLength);
      parameters.setNormalZThresholdForAccessibleRegions(normalZThresholdForAccessibleRegions);
      parameters.setObstacleExtrusionDistance(extrusionDistance);
      parameters.setObstacleExtrusionDistanceIfNotTooHighToStep(extrusionDistanceIfNotTooHighToStep);
      parameters.setTooHighToStepDistance(tooHighToStepDistance);
      parameters.setClusterResolution(clusterResolution);
      parameters.setExplorationDistanceFromStartGoal(explorationDistance);
      parameters.setPlanarRegionMinArea(planarRegionMinArea);
      parameters.setPlanarRegionMinSize(planarRegionMinSize);
      parameters.setRegionOrthogonalAngle(regionOrthogonalAngle);
      parameters.setSearchHostRegionEpsilon(searchHostRegionEpsilon);

      return parameters;
   }

   private static void checkHeightMapDataAreEqual(HeightMapData dataA, HeightMapData dataB)
   {
      assertEquals(dataA.getCenterIndex(), dataB.getCenterIndex());
      assertEquals(dataA.getGridResolutionXY(), dataB.getGridResolutionXY(), epsilon);
      assertEquals(dataA.getGridSizeXY(), dataB.getGridSizeXY(), epsilon);
      EuclidCoreTestTools.assertEquals(dataA.getGridCenter(), dataB.getGridCenter(), epsilon);

      int cellsPerSide = 2 * dataA.getCenterIndex() + 1;
      for (int x = 0; x < cellsPerSide; x++)
      {
         for (int y = 0; y < cellsPerSide; y++)
         {
            assertEquals(dataA.getHeightAt(x, y), dataB.getHeightAt(x, y), epsilon);
         }
      }
   }

   private static void checkFootstepPlansAreEqual(FootstepDataListMessage footstepDataListMessageA, FootstepDataListMessage footstepDataListMessageB)
   {
      footstepDataListMessageA.epsilonEquals(footstepDataListMessageB, epsilon);
   }

   private static void checkFootstepPlannerParameters(FootstepPlannerParametersReadOnly parameters, FootstepPlannerParametersPacket packet)
   {
      assertEquals(parameters.checkForBodyBoxCollisions(), packet.getCheckForBodyBoxCollisions(), "Check for body box collisions flags aren't equal.");
      assertEquals(parameters.checkForPathCollisions(), packet.getCheckForPathCollisions(), "Check for path collisions flags aren't equal.");
      assertEquals(parameters.getIdealFootstepWidth(), packet.getIdealFootstepWidth(), epsilon, "Ideal footstep widths aren't equal.");
      assertEquals(parameters.getIdealFootstepLength(), packet.getIdealFootstepLength(), epsilon, "Ideal footstep lengths aren't equal.");
      assertEquals(parameters.getIdealSideStepWidth(), packet.getIdealSideStepWidth(), epsilon, "Ideal footstep lengths aren't equal.");
      assertEquals(parameters.getIdealBackStepLength(), packet.getIdealBackStepLength(), epsilon, "Ideal footstep lengths aren't equal.");
      assertEquals(parameters.getIdealStepLengthAtMaxStepZ(), packet.getIdealStepLengthAtMaxStepZ(), epsilon, "Ideal footstep lengths aren't equal.");
      assertEquals(parameters.getWiggleInsideDeltaTarget(), packet.getWiggleInsideDeltaTarget(), epsilon, "Wiggle inside delta targets aren't equal.");
      assertEquals(parameters.getWiggleInsideDeltaMinimum(), packet.getWiggleInsideDeltaMinimum(), epsilon, "Wiggle inside delta minimums aren't equal.");
      assertEquals(parameters.getMaximumStepReach(), parameters.getMaximumStepReach(), epsilon, "Maximum step reaches aren't equal.");
      assertEquals(parameters.getMaximumStepYaw(), packet.getMaximumStepYaw(), epsilon, "Maximum step yaws aren't equal.");
      assertEquals(parameters.getUseStepReachabilityMap(), packet.getUseReachabilityMap(), "Use reachability map isn't equal");
      assertEquals(parameters.getSolutionQualityThreshold(), packet.getSolutionQualityThreshold(), epsilon, "Solution quality threshold isn't equal");
      assertEquals(parameters.getMinimumStepWidth(), packet.getMinimumStepWidth(), epsilon, "Minimum step widths aren't equal.");
      assertEquals(parameters.getMinimumStepLength(), packet.getMinimumStepLength(), epsilon, "Minimum step lengths aren't equal.");
      assertEquals(parameters.getMinimumStepYaw(), packet.getMinimumStepYaw(), epsilon, "Minimum step yaws aren't equal.");
      assertEquals(parameters.getMaximumStepReachWhenSteppingUp(), packet.getMaximumStepReachWhenSteppingUp(), epsilon);
      assertEquals(parameters.getMaximumStepZWhenSteppingUp(), packet.getMaximumStepZWhenSteppingUp(), epsilon);
      assertEquals(parameters.getMaximumStepXWhenForwardAndDown(), packet.getMaximumStepXWhenForwardAndDown(), epsilon, "Max X forward and down aren't equal");
      assertEquals(parameters.getMaximumStepZWhenForwardAndDown(), packet.getMaximumStepZWhenForwardAndDown(), epsilon, "Max Z forward and down aren't equal");

      assertEquals(parameters.getMaxStepZ(), packet.getMaximumStepZ(), epsilon, "Max step z isn't equal.");
      assertEquals(parameters.getMaxSwingZ(), packet.getMaximumSwingZ(), epsilon, "Max swing z isn't equal.");
      assertEquals(parameters.getMaxSwingReach(), packet.getMaximumSwingReach(), epsilon, "Max swing reach isn't equal.");
      assertEquals(parameters.getMinimumFootholdPercent(), packet.getMinimumFootholdPercent(), epsilon, "Min foothold percent aren't equal.");
      assertEquals(parameters.getMinimumSurfaceInclineRadians(), packet.getMinimumSurfaceInclineRadians(), epsilon, "Min surface incline aren't equal.");
      assertEquals(parameters.getWiggleWhilePlanning(), packet.getWiggleWhilePlanning());
      assertEquals(parameters.getEnableConcaveHullWiggler(), packet.getEnableConcaveHullWiggler(), "Wiggle while planning isn't equal.");
      assertEquals(parameters.getMaximumXYWiggleDistance(), packet.getMaximumXyWiggleDistance(), epsilon, "Max XY wiggle distance isn't equal.");
      assertEquals(parameters.getMaximumYawWiggle(), packet.getMaximumYawWiggle(), epsilon, "Max yaw wiggle isn't equal.");
      assertEquals(parameters.getMaximumZPenetrationOnValleyRegions(), packet.getMaximumZPenetrationOnValleyRegions(), epsilon, "Max Z penetration isn't equal.");
      assertEquals(parameters.getMaximumStepWidth(), packet.getMaximumStepWidth(), epsilon, "Max step width isn't equal.");
      assertEquals(parameters.getCliffBaseHeightToAvoid(), packet.getCliffBaseHeightToAvoid(), epsilon, "Cliff base height to avoid isn't equal.");
      assertEquals(parameters.getMinimumDistanceFromCliffBottoms(), packet.getMinimumDistanceFromCliffBottoms(), epsilon, "Minimum distance from cliff bottoms isn't equal.");
      assertEquals(parameters.getCliffTopHeightToAvoid(), packet.getCliffTopHeightToAvoid(), epsilon, "Cliff top height to avoid isn't equal.");
      assertEquals(parameters.getMinimumDistanceFromCliffTops(), packet.getMinimumDistanceFromCliffTops(), epsilon, "Minimum distance from cliff tops isn't equal.");
      assertEquals(parameters.getBodyBoxHeight(), packet.getBodyBoxHeight(), epsilon, "Body box heigth isn't equal.");
      assertEquals(parameters.getBodyBoxDepth(), packet.getBodyBoxDepth(), epsilon, "Body box depth isn't equal.");
      assertEquals(parameters.getBodyBoxWidth(), packet.getBodyBoxWidth(), epsilon, "Body box width isn't equal.");
      assertEquals(parameters.getBodyBoxBaseX(), packet.getBodyBoxBaseX(), epsilon, "Body box base X isn't equal.");
      assertEquals(parameters.getBodyBoxBaseY(), packet.getBodyBoxBaseY(), epsilon, "Body box base Y isn't equal.");
      assertEquals(parameters.getBodyBoxBaseZ(), packet.getBodyBoxBaseZ(), epsilon, "Body box base Z isn't equal.");
      assertEquals(parameters.getMaximumSnapHeight(), packet.getMaximumSnapHeight(), epsilon, "Maximum snap height isn't equal");
      assertEquals(parameters.getMinClearanceFromStance(), packet.getMinClearanceFromStance(), epsilon, "Min clearance from stance isn't equal.");

      assertEquals(parameters.getAStarHeuristicsWeight().getValue(), packet.getAStarHeuristicsWeight(), epsilon, "A star heuristics weights aren't equal.");
      assertEquals(parameters.getYawWeight(), packet.getYawWeight(), epsilon, "Yaw weights aren't equal.");
      assertEquals(parameters.getRollWeight(), packet.getRollWeight(), epsilon, "Roll weights aren't equal.");
      assertEquals(parameters.getPitchWeight(), packet.getPitchWeight(), epsilon, "Pitch weights aren't equal.");
      assertEquals(parameters.getForwardWeight(), packet.getForwardWeight(), epsilon, "Forward weights aren't equal.");
      assertEquals(parameters.getLateralWeight(), packet.getLateralWeight(), epsilon, "Lateral weights aren't equal.");
      assertEquals(parameters.getStepUpWeight(), packet.getStepUpWeight(), epsilon, "Step up weights aren't equal.");
      assertEquals(parameters.getStepDownWeight(), packet.getStepDownWeight(), epsilon, "Step down weights aren't equal.");
      assertEquals(parameters.getCostPerStep(), packet.getCostPerStep(), epsilon, "Cost per step isn't equal.");
   }

   private static void checkVisibilityGraphsParameters(VisibilityGraphsParametersReadOnly parameters, VisibilityGraphsParametersPacket packet)
   {
      assertEquals(parameters.getMaxInterRegionConnectionLength(), packet.getMaxInterRegionConnectionLength(), epsilon);
      assertEquals(parameters.getNormalZThresholdForAccessibleRegions(), packet.getNormalZThresholdForAccessibleRegions(), epsilon);
      assertEquals(parameters.getObstacleExtrusionDistance(), packet.getObstacleExtrusionDistance(), epsilon);
      assertEquals(parameters.getObstacleExtrusionDistanceIfNotTooHighToStep(), packet.getObstacleExtrusionDistanceIfNotTooHighToStep(), epsilon);
      assertEquals(parameters.getTooHighToStepDistance(), packet.getTooHighToStepDistance(), epsilon);
      assertEquals(parameters.getClusterResolution(), packet.getClusterResolution(), epsilon);
      assertEquals(parameters.getExplorationDistanceFromStartGoal(), packet.getExplorationDistanceFromStartGoal(), epsilon);
      assertEquals(parameters.getPlanarRegionMinArea(), packet.getPlanarRegionMinArea(), epsilon);
      assertEquals(parameters.getPlanarRegionMinSize(), packet.getPlanarRegionMinSize());
      assertEquals(parameters.getRegionOrthogonalAngle(), packet.getRegionOrthogonalAngle(), epsilon);
      assertEquals(parameters.getSearchHostRegionEpsilon(), packet.getSearchHostRegionEpsilon(), epsilon);
   }
}