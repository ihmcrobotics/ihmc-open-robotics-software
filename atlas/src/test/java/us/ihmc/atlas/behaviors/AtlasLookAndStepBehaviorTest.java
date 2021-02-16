package us.ihmc.atlas.behaviors;

import controller_msgs.msg.dds.StoredPropertySetMessage;
import controller_msgs.msg.dds.WalkingControllerFailureStatusMessage;
import javafx.application.Platform;
import org.junit.jupiter.api.*;
import org.junit.jupiter.api.MethodOrderer.OrderAnnotation;
import org.junit.jupiter.api.parallel.Execution;
import org.junit.jupiter.api.parallel.ExecutionMode;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.behaviors.AtlasPerceptionSimulation.Fidelity;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.environments.BehaviorPlanarRegionEnvironments;
import us.ihmc.avatar.environments.RealisticLabTerrainBuilder;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulation;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulationParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearanceTexture;
import us.ihmc.humanoidBehaviors.BehaviorModule;
import us.ihmc.humanoidBehaviors.BehaviorRegistry;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorParameters;
import us.ihmc.humanoidBehaviors.tools.RemoteHumanoidRobotInterface;
import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.humanoidBehaviors.ui.behaviors.LookAndStepRemoteVisualizer;
import us.ihmc.humanoidBehaviors.ui.simulation.EnvironmentInitialSetup;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.tools.thread.PausablePeriodicThread;

import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.LOOK_AND_STEP_PARAMETERS;

// TODO: Add reviewing; Add status logger to visualizer
@Execution(ExecutionMode.SAME_THREAD)
@TestMethodOrder(OrderAnnotation.class)
public class AtlasLookAndStepBehaviorTest
{
   private static final boolean VISUALIZE = Boolean.parseBoolean(System.getProperty("visualize"));
   public static final CommunicationMode COMMUNICATION_MODE = CommunicationMode.INTRAPROCESS;

   static
   {
      System.setProperty("create.scs.gui", Boolean.toString(VISUALIZE));
   }

   private BehaviorModule behaviorModule;
   private ROS2Node ros2Node;
   private Messager behaviorMessager;
   private PausablePeriodicThread monitorThread;
   private LookAndStepRemoteVisualizer lookAndStepVisualizer;
   private HumanoidKinematicsSimulation kinematicsSimulation;
   private AtlasDynamicsSimulation dynamicsSimulation;
   private AtlasPerceptionSimulation perceptionStack;

   private static class TestWaypoint
   {
      String name;
      Pose3D goalPose;
      Function<Pose3DReadOnly, Boolean> reachedCondition;
      Notification reachedNotification = new Notification();
   }

   @Test
   @Order(1)
   public void testLookAndStepOverFlatGround()
   {
      List<TestWaypoint> waypoints = new ArrayList<>();
      waypoints.add(new TestWaypoint());
      waypoints.get(0).name = "HALFWAY";
      waypoints.get(0).goalPose = new Pose3D(1.5, 0.0, 0.0, 0.0, 0.0, 0.0);
      waypoints.get(0).reachedCondition = pelvisPose ->
      {
         double remainingDistance = waypoints.get(0).goalPose.getX() - pelvisPose.getPosition().getX();
         LogTools.info("Remaining distance: {}", remainingDistance);
         return Math.abs(remainingDistance) < 0.7;
      };
      waypoints.add(new TestWaypoint());
      waypoints.get(1).name = "ALL THE WAY";
      waypoints.get(1).goalPose = new Pose3D(3.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      waypoints.get(1).reachedCondition = pelvisPose ->
      {
         double remainingDistance = waypoints.get(0).goalPose.getX() - pelvisPose.getPosition().getX();
         LogTools.info("Remaining distance: {}", remainingDistance);
         return Math.abs(remainingDistance) < 0.8;
      };

      boolean useDynamicsSimulation = false;
      boolean runRealsenseSLAM = true;
      boolean runLidarREA = true;
      EnvironmentInitialSetup environment = new EnvironmentInitialSetup(BehaviorPlanarRegionEnvironments::flatGround, 0.0, 0.0, 0.0, 0.0);
      assertTimeoutPreemptively(Duration.ofMinutes(3),
                                () -> runTheTest(environment,
                                                 useDynamicsSimulation,
                                                 runRealsenseSLAM,
                                                 runLidarREA,
                                                 waypoints));
   }

   @Test
   @Order(2)
   public void testLookAndStepDemoPart1()
   {
      List<TestWaypoint> waypoints = new ArrayList<>();
      waypoints.add(new TestWaypoint());
      waypoints.get(0).name = "ALL THE WAY";
      waypoints.get(0).goalPose = new Pose3D(4.1, 0.0, 0.0, 0.0, 0.0, 0.0);
      waypoints.get(0).reachedCondition = pelvisPose ->
      {
         double remainingDistance = waypoints.get(0).goalPose.getX() - pelvisPose.getPosition().getX();
         LogTools.info("Remaining distance: {}", remainingDistance);
         return Math.abs(remainingDistance) < 0.8;
      };


      boolean useDynamicsSimulation = false;
      boolean runRealsenseSLAM = true;
      boolean runLidarREA = true;
      EnvironmentInitialSetup environment = new EnvironmentInitialSetup(BehaviorPlanarRegionEnvironments::generateRealisticEasierStartingBlockRegions,
                                                                        RealisticLabTerrainBuilder.PALLET_HEIGHT * 2.0,
                                                                        0.0,
                                                                        0.0,
                                                                        0.0);
      assertTimeoutPreemptively(Duration.ofMinutes(3),
                                () -> runTheTest(environment,
                                                 useDynamicsSimulation,
                                                 runRealsenseSLAM,
                                                 runLidarREA,
                                                 waypoints));
   }

   @Test
   @Disabled
   @Order(3)
   public void testLookAndStepOverRoughTerrain()
   {
      List<TestWaypoint> waypoints = new ArrayList<>();
      waypoints.add(new TestWaypoint());
      waypoints.get(0).name = "THE TOP";
      waypoints.get(0).goalPose = new Pose3D(3.0, 0.0, BehaviorPlanarRegionEnvironments.topPlatformHeight, 0.0, 0.0, 0.0);
      waypoints.get(0).reachedCondition = pelvisPose -> pelvisPose.getPosition().getX() > 2.2 && pelvisPose.getPosition().getZ() > 1.3;
      waypoints.add(new TestWaypoint());
      waypoints.get(1).name = "OTHER SIDE";
      waypoints.get(1).goalPose = new Pose3D(6.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      waypoints.get(1).reachedCondition = pelvisPose -> pelvisPose.getPosition().getX() > 5.2;

      boolean useDynamicsSimulation = true;
      boolean runRealsenseSLAM = false;
      boolean runLidarREA = true;
      EnvironmentInitialSetup environment = new EnvironmentInitialSetup(BehaviorPlanarRegionEnvironments::createRoughUpAndDownStepsWithFlatTop,
                                                                        0.0,
                                                                        0.0,
                                                                        0.0,
                                                                        0.0);
      assertTimeoutPreemptively(Duration.ofMinutes(5),
                                () -> runTheTest(environment,
                                                 useDynamicsSimulation,
                                                 runRealsenseSLAM,
                                                 runLidarREA,
                                                 waypoints));
   }

   @Test
   @Disabled
   @Order(4)
   public void testLookAndStepOverStairStepsWRealsenseSLAM()
   {
      List<TestWaypoint> waypoints = new ArrayList<>();
      waypoints.add(new TestWaypoint());
      waypoints.get(0).name = "THE TOP";
      waypoints.get(0).goalPose = new Pose3D(3.0, 0.0, BehaviorPlanarRegionEnvironments.topPlatformHeight, 0.0, 0.0, 0.0);
      waypoints.get(0).reachedCondition = pelvisPose -> pelvisPose.getPosition().getX() > 2.2 && pelvisPose.getPosition().getZ() > 1.3;
      waypoints.add(new TestWaypoint());
      waypoints.get(1).name = "OTHER SIDE";
      waypoints.get(1).goalPose = new Pose3D(6.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      waypoints.get(1).reachedCondition = pelvisPose -> pelvisPose.getPosition().getX() > 5.2;

      boolean useDynamicsSimulation = true;
      boolean runRealsenseSLAM = true;
      boolean runLidarREA = true;
      EnvironmentInitialSetup environment = new EnvironmentInitialSetup(BehaviorPlanarRegionEnvironments::createFlatUpAndDownStepsWithFlatTop,
                                                                        0.0,
                                                                        0.0,
                                                                        0.0,
                                                                        0.0);
      assertTimeoutPreemptively(Duration.ofMinutes(5), () -> runTheTest(environment,
                                                 useDynamicsSimulation,
                                                 runRealsenseSLAM,
                                                 runLidarREA,
                                                 waypoints));
   }

   private void runTheTest(EnvironmentInitialSetup environment,
                           boolean useDynamicsSimulation,
                           boolean runRealsenseSLAM,
                           boolean runLidarREA,
                           List<TestWaypoint> waypoints)
   {
      ThreadTools.startAsDaemon(() -> perceptionStack(environment, runRealsenseSLAM, runLidarREA), "PerceptionStack");
      Notification finishedSimulationSetup = new Notification();
      if (useDynamicsSimulation)
      {
         ThreadTools.startAsDaemon(() -> dynamicsSimulation(environment, finishedSimulationSetup), "DynamicsSimulation");
      }
      else
      {
         ThreadTools.startAsDaemon(() -> kinematicSimulation(environment, finishedSimulationSetup), "KinematicsSimulation");
      }

      ros2Node = ROS2Tools.createROS2Node(COMMUNICATION_MODE.getPubSubImplementation(), "Helper");


      AtlasRobotModel robotModelForBehavior = createRobotModel();
      robotModelForBehavior.getSwingPlannerParameters().setMinimumSwingFootClearance(0.0);
      behaviorModule = new BehaviorModule(BehaviorRegistry.of(LookAndStepBehavior.DEFINITION), robotModelForBehavior, COMMUNICATION_MODE, COMMUNICATION_MODE);
      behaviorMessager = behaviorModule.getMessager();

      if (VISUALIZE)
      {
         lookAndStepVisualizer = new LookAndStepRemoteVisualizer(createRobotModel(), ros2Node, behaviorMessager);
      }

      finishedSimulationSetup.blockingPoll();

      ThreadTools.sleepSeconds(3.0);

      AtlasRobotModel robotModel = createRobotModel();
      RemoteHumanoidRobotInterface robot = new RemoteHumanoidRobotInterface(ros2Node, robotModel);

      AtomicReference<String> currentState = behaviorMessager.createInput(LookAndStepBehaviorAPI.CurrentState);
      behaviorMessager.submitMessage(BehaviorModule.API.BehaviorSelection, LookAndStepBehavior.DEFINITION.getName());

      Notification reachedGoalOrFallen = new Notification();

      new IHMCROS2Callback<>(ros2Node,
                             ControllerAPIDefinition.getTopic(WalkingControllerFailureStatusMessage.class, robotModel.getSimpleRobotName()),
                             message ->
                             {
                                LogTools.error("Controller failure detected! Fall direction: {}", message.getFallingDirection());
                                reachedGoalOrFallen.set();
                             });

      LookAndStepBehaviorParameters lookAndStepBehaviorParameters = new LookAndStepBehaviorParameters();
      StoredPropertySetMessage storedPropertySetMessage = new StoredPropertySetMessage();
      lookAndStepBehaviorParameters.getAllAsStrings().forEach(value -> storedPropertySetMessage.getStrings().add(value));
      IHMCROS2Publisher<StoredPropertySetMessage> parametersPublisher = new IHMCROS2Publisher<>(ros2Node, LOOK_AND_STEP_PARAMETERS);
      parametersPublisher.publish(storedPropertySetMessage);

      IHMCROS2Publisher<Pose3D> goalInputPublisher = IHMCROS2Publisher.newPose3DPublisher(ros2Node, LookAndStepBehaviorAPI.GOAL_INPUT);
      new IHMCROS2Callback<>(ros2Node, LookAndStepBehaviorAPI.REACHED_GOAL, message ->
      {
         LogTools.info("REACHED GOAL");
         reachedGoalOrFallen.set();
      });

      RemoteSyncedRobotModel syncedRobot = robot.newSyncedRobot();
      monitorThread = new PausablePeriodicThread(
            "RobotStatusThread",
            0.5,
            () -> monitorThread(currentState, syncedRobot, waypoints));
      monitorThread.start();

      Notification bodyPathPlanningStateReached = new Notification();
      behaviorMessager.registerTopicListener(LookAndStepBehaviorAPI.CurrentState, state ->
      {
         if (state.equals(LookAndStepBehavior.State.BODY_PATH_PLANNING.name()))
         {
            bodyPathPlanningStateReached.set();
         }
      });
      LogTools.info("Waiting for BODY_PATH_PLANNING state...");
      bodyPathPlanningStateReached.blockingPoll();
      LogTools.info("BODY_PATH_PLANNING state reached");
      behaviorMessager.submitMessage(LookAndStepBehaviorAPI.OperatorReviewEnabled, false);


      for (TestWaypoint waypoint : waypoints)
      {
         LogTools.info("Publishing goal pose: {}", waypoint.goalPose);
         goalInputPublisher.publish(waypoint.goalPose);
         reachedGoalOrFallen.blockingPoll();
         assertTrue(waypoint.reachedNotification.poll(), "NOT " + waypoint.name);
         LogTools.info("REACHED " + waypoint.name);
      }
   }

   private void process()
   {

   }

   private void monitorThread(AtomicReference<String> currentState,
                              RemoteSyncedRobotModel syncedRobot,
                              List<TestWaypoint> waypoints)
   {
      syncedRobot.update();
      FramePose3DReadOnly pelvisPose = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getPelvisZUpFrame);

      LogTools.info("{} pose: {}", currentState.get(), pelvisPose);
      for (int i = 0; i < waypoints.size(); i++)
      {
         TestWaypoint waypoint = waypoints.get(i);
         if (waypoint.reachedCondition.apply(pelvisPose))
         {
            LogTools.info("Waypoint {} reached", i);
            waypoint.reachedNotification.set();
         }
         else
         {
            LogTools.info("Waypoint {} not reached", i);
         }
      }
   }

   private void perceptionStack(EnvironmentInitialSetup environment, boolean runRealsenseSLAM, boolean runLidarREA)
   {
      perceptionStack = new AtlasPerceptionSimulation(COMMUNICATION_MODE,
                                                      environment.getPlanarRegionsSupplier().get(),
                                                      runRealsenseSLAM,
                                                      false,
                                                      runLidarREA,
                                                      createRobotModel(),
                                                      Fidelity.HIGH);
   }

   private void dynamicsSimulation(EnvironmentInitialSetup environment, Notification finishedSettingUp)
   {
      LogTools.info("Creating dynamics simulation");
      int recordFrequencySpeedup = 50; // Increase to 10 when you want the sims to run a little faster and don't need all of the YoVariable data.
      int scsDataBufferSize = 10;
      dynamicsSimulation = AtlasDynamicsSimulation.create(createRobotModel(),
                                                          createCommonAvatarEnvironment(environment),
                                                          environment.getGroundZ(),
                                                          environment.getInitialX(),
                                                          environment.getInitialY(),
                                                          environment.getInitialYaw(),
                                                          COMMUNICATION_MODE.getPubSubImplementation(),
                                                          recordFrequencySpeedup,
                                                          scsDataBufferSize,
                                                          true);
      dynamicsSimulation.simulate();
      LogTools.info("Finished setting up dynamics simulation.");
      finishedSettingUp.set();
   }

   private void kinematicSimulation(EnvironmentInitialSetup environment, Notification finishedSimulationSetup)
   {
      LogTools.info("Creating kinematics  simulation");
      HumanoidKinematicsSimulationParameters kinematicsSimulationParameters = new HumanoidKinematicsSimulationParameters();
      kinematicsSimulationParameters.setPubSubImplementation(COMMUNICATION_MODE.getPubSubImplementation());
      kinematicsSimulationParameters.setLogToFile(true);
      kinematicsSimulationParameters.setCreateYoVariableServer(false);
      kinematicsSimulationParameters.setInitialGroundHeight(environment.getGroundZ());
      kinematicsSimulationParameters.setInitialRobotYaw(environment.getInitialYaw());
      kinematicsSimulationParameters.setInitialRobotX(environment.getInitialX());
      kinematicsSimulationParameters.setInitialRobotY(environment.getInitialY());
      kinematicsSimulation = AtlasKinematicSimulation.create(createRobotModel(), kinematicsSimulationParameters);
      finishedSimulationSetup.set();
   }

   private CommonAvatarEnvironmentInterface createCommonAvatarEnvironment(EnvironmentInitialSetup environment)
   {
      String environmentName = PlanarRegionsListDefinedEnvironment.class.getSimpleName();
      YoAppearanceTexture cinderBlockTexture = new YoAppearanceTexture("sampleMeshes/cinderblock.png");
      return new PlanarRegionsListDefinedEnvironment(environmentName,
                                                     environment.getPlanarRegionsSupplier().get(),
                                                     cinderBlockTexture,
                                                     0.02,
                                                     false);
   }

   private AtlasRobotModel createRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
   }

   @BeforeAll
   public static void beforeAll()
   {
      Platform.setImplicitExit(false);
   }

   @AfterAll
   public static void afterAll()
   {
      Platform.exit();
   }

   @AfterEach
   public void afterEach()
   {
      perceptionStack.destroy();
      behaviorModule.destroy();
      ros2Node.destroy();
      ExceptionTools.handle(() -> behaviorMessager.closeMessager(), DefaultExceptionHandler.PRINT_STACKTRACE);
      monitorThread.stop();
      if (VISUALIZE)
         Platform.runLater(() -> lookAndStepVisualizer.close());
      if (kinematicsSimulation != null)
      {
         kinematicsSimulation.destroy();
         kinematicsSimulation = null;
      }
      if (dynamicsSimulation != null)
      {
         dynamicsSimulation.destroy();
         dynamicsSimulation = null;
      }
   }
}
