package us.ihmc.footstepPlanning;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.IntStream;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepPlannerParametersPacket;
import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.VisibilityGraphsParametersPacket;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
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
import us.ihmc.footstepPlanning.ui.FootstepPlannerUI;
import us.ihmc.footstepPlanning.ui.RemoteUIMessageConverter;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javaFXToolkit.starter.ApplicationRunner;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.ros2.RealtimeROS2Node;

public class RemoteFootstepPlannerUIMessagingTest
{
   private static final int iters = 1;
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
      for (int i = 0; i < 100; i++)
         ThreadTools.sleep(10);

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
               ui = FootstepPlannerUI.createMessagerUI(stage, (SharedMemoryJavaFXMessager) messager);
               ui.show();
            }

            @Override
            public void stop() throws Exception
            {
               ui.stop();
               Platform.exit();
            }
         });

         while (ui == null)
            ThreadTools.sleep(10);
      }
   }

   @Test
   public void testSendingFootstepPlanningRequestPacketFromUIIntraprocess()
   {
      pubSubImplementation = DomainFactory.PubSubImplementation.INTRAPROCESS;
      setup();
      runPlanningRequestTestFromUI();
   }

   @Disabled
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

   @Disabled
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

   @Disabled
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

   @Disabled
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

      for (int iter = 0; iter < iters; iter++)
      {
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
         int plannerRequestId = RandomNumbers.nextInt(random, 1, 100);

         messager.submitMessage(FootstepPlannerMessagerAPI.LeftFootPose, startLeftFootPose);
         messager.submitMessage(FootstepPlannerMessagerAPI.RightFootPose, startRightFootPose);
         messager.submitMessage(FootstepPlannerMessagerAPI.LeftFootGoalPose, goalLeftFootPose);
         messager.submitMessage(FootstepPlannerMessagerAPI.RightFootGoalPose, goalRightFootPose);
         messager.submitMessage(FootstepPlannerMessagerAPI.PlanBodyPath, planBodyPath);
         messager.submitMessage(FootstepPlannerMessagerAPI.PerformAStarSearch, performAStarSearch);
         messager.submitMessage(FootstepPlannerMessagerAPI.PlannerTimeout, timeout);
         messager.submitMessage(FootstepPlannerMessagerAPI.MaxIterations, maxIterations);
         messager.submitMessage(FootstepPlannerMessagerAPI.PlanarRegionData, planarRegionsList);
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
               assertTrue("Timed out waiting for packet.", false);

            ThreadTools.sleep(10);
         }

         FootstepPlanningRequestPacket packet = planningRequestReference.getAndSet(null);

         assertTrue("Left foot poses aren't equal", packet.getStartLeftFootPose().epsilonEquals(startLeftFootPose, epsilon));
         assertTrue("Right foot poses aren't equal", packet.getStartRightFootPose().epsilonEquals(startRightFootPose, epsilon));
         assertTrue("Left goal foot poses aren't equal", packet.getGoalLeftFootPose().epsilonEquals(goalLeftFootPose, epsilon));
         assertTrue("Right goal foot poses aren't equal", packet.getGoalRightFootPose().epsilonEquals(goalRightFootPose, epsilon));
         assertEquals("Timeouts aren't equal.", timeout, packet.getTimeout(), 1e-5);
         assertEquals("Perform A* search flags aren't.", performAStarSearch, packet.getPerformAStarSearch());
         assertEquals("Plan body path flags aren't equal.", planBodyPath, packet.getPlanBodyPath());
         assertEquals("Initial support sides aren't equal.", robotSide, RobotSide.fromByte(packet.getRequestedInitialStanceSide()));

         assertEquals("Planner Request Ids aren't equal.", plannerRequestId, packet.getPlannerRequestId(), epsilon);
         assertEquals("Planner horizon lengths aren't equal.", horizonLength, packet.getHorizonLength(), epsilon);

         checkPlanarRegionListsAreEqual(planarRegionsList, PlanarRegionMessageConverter.convertToPlanarRegionsList(packet.getPlanarRegionsListMessage()));

         for (int i = 0; i < 100; i++)
            ThreadTools.sleep(10);
      }
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

      for (int iter = 0; iter < iters; iter++)
      {
         double timeout = RandomNumbers.nextDouble(random, 0.1, 100.0);
         double horizonLength = RandomNumbers.nextDouble(random, 0.1, 10.0);
         int sequenceId = RandomNumbers.nextInt(random, 1, 100);
         int plannerRequestId = RandomNumbers.nextInt(random, 1, 100);
         boolean planBodyPath = random.nextBoolean();
         boolean performAStarSearch = random.nextBoolean();

         RobotSide robotSide = RobotSide.generateRandomRobotSide(random);
         PlanarRegionsList planarRegionsList = createRandomPlanarRegionList(random);
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
         packet.getPlanarRegionsListMessage().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList));

         footstepPlanningRequestPublisher.publish(packet);

         double maxWaitTime = 5.0;
         double currentWaitTime = 0.0;
         long sleepDuration = 10;
         while (leftFootPoseReference.get() == null || rightFootPoseReference.get() == null || leftFootGoalPoseReference.get() == null
                || rightFootGoalPoseReference.get() == null || timeoutReference.get() == null || performAStarSearchReference.get() == null
                || planBodyPathReference.get() == null || robotSideReference.get() == null || plannerRequestIdReference.get() == null
                || plannerHorizonLengthReference.get() == null)
         {
            assertFalse("Timed out waiting on the results.", currentWaitTime > maxWaitTime);

            ThreadTools.sleep(sleepDuration);
            currentWaitTime += Conversions.millisecondsToSeconds(sleepDuration);
         }

         Assertions.assertTrue(leftFootPose.epsilonEquals(leftFootPoseReference.get(), epsilon), "leftFootPose values aren't equal");
         Assertions.assertTrue(rightFootPose.epsilonEquals(rightFootPoseReference.get(), epsilon), "rightFootPose values aren't equal");
         Assertions.assertTrue(leftFootGoalPose.epsilonEquals(leftFootGoalPoseReference.get(), epsilon), "leftFootGoalPose values aren't equal");
         Assertions.assertTrue(rightFootGoalPose.epsilonEquals(rightFootGoalPoseReference.get(), epsilon), "rightFootGoalPose values aren't equal");
         assertEquals("Timeouts aren't equal.", timeout, timeoutReference.getAndSet(null), epsilon);
         assertEquals("Perform A* search flags aren't equal.", performAStarSearch, performAStarSearchReference.getAndSet(null));
         assertEquals("Plan body path flags aren't equal.", planBodyPath, planBodyPathReference.getAndSet(null));
         assertEquals("Initial support sides aren't equal.", robotSide, robotSideReference.getAndSet(null));
         assertEquals("Planner Request Ids aren't equal.", plannerRequestId, plannerRequestIdReference.getAndSet(null), epsilon);
         assertEquals("Planner horizon lengths aren't equal.", horizonLength, plannerHorizonLengthReference.getAndSet(null), epsilon);

         for (int i = 0; i < 100; i++)
            ThreadTools.sleep(10);
      }
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

      for (int iter = 0; iter < iters; iter++)
      {
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
            assertTrue("Timed out waiting for packet.", ticks < 100);

            ThreadTools.sleep(10);
         }

         FootstepPlannerParametersPacket plannerPacket = footstepPlannerParametersReference.getAndSet(null);
         VisibilityGraphsParametersPacket visibilityGraphsPacket = visibilityGraphsParametersReference.getAndSet(null);

         checkFootstepPlannerParameters(randomParameters, plannerPacket);
         checkVisibilityGraphsParameters(randomVisibilityGraphParameters, visibilityGraphsPacket);

         for (int i = 0; i < 100; i++)
            ThreadTools.sleep(10);
      }
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
      AtomicReference<List<? extends Pose3DReadOnly>> bodyPathReference = messager.createInput(FootstepPlannerMessagerAPI.BodyPathData);
      AtomicReference<Point3D> lowLevelPositionGoalReference = messager.createInput(FootstepPlannerMessagerAPI.LowLevelGoalPosition);
      AtomicReference<Quaternion> lowLevelOrientationGoalReference = messager.createInput(FootstepPlannerMessagerAPI.LowLevelGoalOrientation);

      for (int iter = 0; iter < iters; iter++)
      {

         Pose2D goalPose = new Pose2D();
         goalPose.getPosition().set(EuclidCoreRandomTools.nextPoint2D(random));
         goalPose.getOrientation().set(EuclidCoreRandomTools.nextQuaternion(random));
         PlanarRegionsList planarRegionsList = createRandomPlanarRegionList(random);
         FootstepDataListMessage footstepDataListMessage = nextFootstepDataListMessage(random);
         int sequenceId = RandomNumbers.nextInt(random, 0, 100);
         int planId = RandomNumbers.nextInt(random, 0, 100);
         FootstepPlanningResult result = FootstepPlanningResult.generateRandomResult(random);
         double timeTaken = RandomNumbers.nextDouble(random, 0.0, 1000.0);
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
         outputPacket.getPlanarRegionsList().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList));
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
            assertTrue("Timed out waiting on messages.", ticks < 100);
            ThreadTools.sleep(100);
         }

         checkFootstepPlansAreEqual(footstepDataListMessage, footstepPlanReference.getAndSet(null));
         assertEquals("Planner Ids aren't equal.", planId, receivedPlanIdReference.getAndSet(null), epsilon);
         assertEquals("Planner results aren't equal.", result, plannerResultReference.getAndSet(null));
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals("Low level goal position results aren't equal.", lowLevelGoalPosition, lowLevelPositionGoalReference.getAndSet(null), epsilon);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals("Low level goal orientation results aren't equal.", lowLevelGoalOrientation, lowLevelOrientationGoalReference.getAndSet(null), epsilon);
         List<? extends Pose3DReadOnly> bodyPathResult = bodyPathReference.getAndSet(null);
         assertEquals("Body path size results aren't equal.", bodyPath.size(), bodyPathResult.size());
         for (int i = 0; i < bodyPath.size(); i++)
         {
            EuclidCoreTestTools
                  .assertPoint3DGeometricallyEquals("Body path waypoint " + i + " results aren't equal.", bodyPath.get(i).getPosition(),
                                                    bodyPathResult.get(i).getPosition(), epsilon);
            EuclidCoreTestTools
                  .assertQuaternionGeometricallyEquals("Body path waypoint " + i + " results aren't equal.", bodyPath.get(i).getOrientation(),
                                                    bodyPathResult.get(i).getOrientation(), epsilon);
         }


         for (int i = 0; i < 100; i++)
            ThreadTools.sleep(10);
      }
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
      double preferredExtrusionDistance = RandomNumbers.nextDouble(random, 0.01, 1.0);
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
      parameters.setPreferredObstacleExtrusionDistance(preferredExtrusionDistance);
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

   private static void checkPlanarRegionListsAreEqual(PlanarRegionsList listA, PlanarRegionsList listB)
   {
      assertEquals("Planar region lists are different sizes.", listA.getNumberOfPlanarRegions(), listB.getNumberOfPlanarRegions());

      for (int i = 0; i < listA.getNumberOfPlanarRegions(); i++)
      {
         PlanarRegion planarRegionA = listA.getPlanarRegion(i);
         PlanarRegion planarRegionB = null;
         for (int j = 0; j < listB.getNumberOfPlanarRegions(); j++)
         {
            if (planarRegionA.getRegionId() == listB.getPlanarRegion(j).getRegionId())
            {
               planarRegionB = listB.getPlanarRegion(j);
               break;
            }
         }
         assertFalse("Unable to find equivalent planar region", planarRegionB == null);
         checkPlanarRegionsEqual(i, planarRegionA, planarRegionB);
      }
   }

   private static void checkPlanarRegionsEqual(int regionId, PlanarRegion planarRegionA, PlanarRegion planarRegionB)
   {
      Point3D centerA = new Point3D();
      Point3D centerB = new Point3D();

      planarRegionA.getPointInRegion(centerA);
      planarRegionB.getPointInRegion(centerB);

      Vector3D normalA = new Vector3D();
      Vector3D normalB = new Vector3D();

      planarRegionA.getNormal(normalA);
      planarRegionB.getNormal(normalB);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals("Center of regions " + regionId + " are not equal.", centerA, centerB, epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals("Normal of regions " + regionId + " are not equal.", normalA, normalB, epsilon);

      assertEquals("Number of convex polygons of " + regionId + " not equal. ", planarRegionA.getNumberOfConvexPolygons(),
                   planarRegionB.getNumberOfConvexPolygons());

      for (int i = 0; i < planarRegionA.getNumberOfConvexPolygons(); i++)
      {
         assertTrue("Convex polygon " + i + " of planar region " + regionId + " is not equal",
                    planarRegionA.getConvexPolygon(i).epsilonEquals(planarRegionB.getConvexPolygon(i), epsilon));
      }
   }

   private static void checkFootstepPlansAreEqual(FootstepDataListMessage footstepDataListMessageA, FootstepDataListMessage footstepDataListMessageB)
   {
      footstepDataListMessageA.epsilonEquals(footstepDataListMessageB, epsilon);
   }

   private static void checkFootstepPlannerParameters(FootstepPlannerParametersReadOnly parameters, FootstepPlannerParametersPacket packet)
   {
      assertEquals("Check for body box collisions flags aren't equal.", parameters.checkForBodyBoxCollisions(), packet.getCheckForBodyBoxCollisions());
      assertEquals("Check for path collisions flags aren't equal.", parameters.checkForPathCollisions(), packet.getCheckForPathCollisions());
      assertEquals("Ideal footstep widths aren't equal.", parameters.getIdealFootstepWidth(), packet.getIdealFootstepWidth(), epsilon);
      assertEquals("Ideal footstep lengths aren't equal.", parameters.getIdealFootstepLength(), packet.getIdealFootstepLength(), epsilon);
      assertEquals("Ideal footstep lengths aren't equal.", parameters.getIdealSideStepWidth(), packet.getIdealSideStepWidth(), epsilon);
      assertEquals("Ideal footstep lengths aren't equal.", parameters.getIdealBackStepLength(), packet.getIdealBackStepLength(), epsilon);
      assertEquals("Ideal footstep lengths aren't equal.", parameters.getIdealStepLengthAtMaxStepZ(), packet.getIdealStepLengthAtMaxStepZ(), epsilon);
      assertEquals("Wiggle inside delta targets aren't equal.", parameters.getWiggleInsideDeltaTarget(), packet.getWiggleInsideDeltaTarget(), epsilon);
      assertEquals("Wiggle inside delta minimums aren't equal.", parameters.getWiggleInsideDeltaMinimum(), packet.getWiggleInsideDeltaMinimum(), epsilon);
      assertEquals("Maximum step reaches aren't equal.", parameters.getMaximumStepReach(), parameters.getMaximumStepReach(), epsilon);
      assertEquals("Maximum step yaws aren't equal.", parameters.getMaximumStepYaw(), packet.getMaximumStepYaw(), epsilon);
      assertEquals("Minimum step widths aren't equal.", parameters.getMinimumStepWidth(), packet.getMinimumStepWidth(), epsilon);
      assertEquals("Minimum step lengths aren't equal.", parameters.getMinimumStepLength(), packet.getMinimumStepLength(), epsilon);
      assertEquals("Minimum step yaws aren't equal.", parameters.getMinimumStepYaw(), packet.getMinimumStepYaw(), epsilon);
      assertEquals(parameters.getMaximumStepReachWhenSteppingUp(), packet.getMaximumStepReachWhenSteppingUp(), epsilon);
      assertEquals(parameters.getMaximumStepZWhenSteppingUp(), packet.getMaximumStepZWhenSteppingUp(), epsilon);
      assertEquals("Max X forward and down aren't equal", parameters.getMaximumStepXWhenForwardAndDown(), packet.getMaximumStepXWhenForwardAndDown(), epsilon);
      assertEquals("Max Z forward and down aren't equal", parameters.getMaximumStepZWhenForwardAndDown(), packet.getMaximumStepZWhenForwardAndDown(), epsilon);

      assertEquals("Max step z isn't equal.", parameters.getMaxStepZ(), packet.getMaximumStepZ(), epsilon);
      assertEquals("Max swing z isn't equal.", parameters.getMaxSwingZ(), packet.getMaximumSwingZ(), epsilon);
      assertEquals("Max swing reach isn't equal.", parameters.getMaxSwingReach(), packet.getMaximumSwingReach(), epsilon);
      assertEquals("Min foothold percent aren't equal.", parameters.getMinimumFootholdPercent(), packet.getMinimumFootholdPercent(), epsilon);
      assertEquals("Min surface incline aren't equal.", parameters.getMinimumSurfaceInclineRadians(), packet.getMinimumSurfaceInclineRadians(), epsilon);
      assertEquals("Wiggle while planning isn't equal.", parameters.getWiggleWhilePlanning(), packet.getWiggleWhilePlanning());
      assertEquals("Wiggle into convex hull isn't equal.", parameters.getEnableConcaveHullWiggler(), packet.getEnableConcaveHullWiggler());
      assertEquals("Max XY wiggle distance isn't equal.", parameters.getMaximumXYWiggleDistance(), packet.getMaximumXyWiggleDistance(), epsilon);
      assertEquals("Max yaw wiggle isn't equal.", parameters.getMaximumYawWiggle(), packet.getMaximumYawWiggle(), epsilon);
      assertEquals("Max Z penetration isn't equal.", parameters.getMaximumZPenetrationOnValleyRegions(), packet.getMaximumZPenetrationOnValleyRegions(),
                   epsilon);
      assertEquals("Max step width isn't equal.", parameters.getMaximumStepWidth(), packet.getMaximumStepWidth(), epsilon);
      assertEquals("Cliff base height to avoid isn't equal.", parameters.getCliffBaseHeightToAvoid(), packet.getCliffBaseHeightToAvoid(), epsilon);
      assertEquals("Minimum distance from cliff bottoms isn't equal.", parameters.getMinimumDistanceFromCliffBottoms(),
                   packet.getMinimumDistanceFromCliffBottoms(), epsilon);
      assertEquals("Cliff top height to avoid isn't equal.", parameters.getCliffTopHeightToAvoid(), packet.getCliffTopHeightToAvoid(), epsilon);
      assertEquals("Minimum distance from cliff tops isn't equal.", parameters.getMinimumDistanceFromCliffTops(),
                   packet.getMinimumDistanceFromCliffTops(), epsilon);
      assertEquals("Body box heigth isn't equal.", parameters.getBodyBoxHeight(), packet.getBodyBoxHeight(), epsilon);
      assertEquals("Body box depth isn't equal.", parameters.getBodyBoxDepth(), packet.getBodyBoxDepth(), epsilon);
      assertEquals("Body box width isn't equal.", parameters.getBodyBoxWidth(), packet.getBodyBoxWidth(), epsilon);
      assertEquals("Body box base X isn't equal.", parameters.getBodyBoxBaseX(), packet.getBodyBoxBaseX(), epsilon);
      assertEquals("Body box base Y isn't equal.", parameters.getBodyBoxBaseY(), packet.getBodyBoxBaseY(), epsilon);
      assertEquals("Body box base Z isn't equal.", parameters.getBodyBoxBaseZ(), packet.getBodyBoxBaseZ(), epsilon);
      assertEquals("Maximum snap height isn't equal", parameters.getMaximumSnapHeight(), packet.getMaximumSnapHeight(), epsilon);
      assertEquals("Min clearance from stance isn't equal.", parameters.getMinClearanceFromStance(), packet.getMinClearanceFromStance(), epsilon);

      assertEquals("A star heuristics weights aren't equal.", parameters.getAStarHeuristicsWeight().getValue(), packet.getAStarHeuristicsWeight(), epsilon);
      assertEquals("Yaw weights aren't equal.", parameters.getYawWeight(), packet.getYawWeight(), epsilon);
      assertEquals("Roll weights aren't equal.", parameters.getRollWeight(), packet.getRollWeight(), epsilon);
      assertEquals("Pitch weights aren't equal.", parameters.getPitchWeight(), packet.getPitchWeight(), epsilon);
      assertEquals("Forward weights aren't equal.", parameters.getForwardWeight(), packet.getForwardWeight(), epsilon);
      assertEquals("Lateral weights aren't equal.", parameters.getLateralWeight(), packet.getLateralWeight(), epsilon);
      assertEquals("Step up weights aren't equal.", parameters.getStepUpWeight(), packet.getStepUpWeight(), epsilon);
      assertEquals("Step down weights aren't equal.", parameters.getStepDownWeight(), packet.getStepDownWeight(), epsilon);
      assertEquals("Cost per step isn't equal.", parameters.getCostPerStep(), packet.getCostPerStep(), epsilon);
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

