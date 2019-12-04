package us.ihmc.atlas.behaviors;

import static org.junit.jupiter.api.Assertions.*;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.footstepPlanning.MultiStageFootstepPlanningModule;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulation;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.ROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FlatGroundFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.heuristics.BodyPathHeuristics;
import us.ihmc.footstepPlanning.graphSearch.heuristics.CostToGoHeuristics;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.AlwaysValidNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.FootstepNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.graphSearch.planners.AStarFootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.stepCost.EuclideanDistanceAndYawBasedCost;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCost;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.humanoidBehaviors.tools.RemoteHumanoidRobotInterface;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedHumanoidRobotState;
import us.ihmc.humanoidBehaviors.tools.SimulatedREAModule;
import us.ihmc.humanoidBehaviors.ui.simulation.RobotAndMapViewer;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.PlannerTestEnvironments;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanHolder;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegionsManager;
import us.ihmc.pathPlanning.visibilityGraphs.OcclusionHandlingPathPlanner;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.ObstacleAvoidanceProcessor;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.PathOrientationCalculator;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.thread.TypedNotification;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.List;

public class AtlasCorridorNavigationTest
{
   private static boolean VISUALIZE = Boolean.parseBoolean(System.getProperty("visualize")); // To visualize, pass -Dvisualize=true

   private RobotAndMapViewer robotAndMapViewer;
   private PubSubImplementation pubSubMode = PubSubImplementation.FAST_RTPS;

   @BeforeAll
   static public void beforeAll()
   {
      if (VISUALIZE) JavaFXApplicationCreator.createAJavaFXApplication();
   }

   @AfterEach
   public void afterEach()
   {
      if (VISUALIZE) ThreadTools.sleepForever();
   }

   @Test
   public void testAtlasMakesItToGoalInTrickyCorridor()
   {
      new Thread(() -> {
         LogTools.info("Creating simulate REA module");
         new SimulatedREAModule(PlannerTestEnvironments.getTrickCorridor(), createRobotModel(), pubSubMode).start();
      }).start();

      new Thread(() -> {
         LogTools.info("Creating simulation");
         boolean createYoVariableServer = false;
         if (pubSubMode == PubSubImplementation.FAST_RTPS)
         {
            HumanoidKinematicsSimulation.createForManualTest(createRobotModel(), createYoVariableServer);
         }
         else
         {
            HumanoidKinematicsSimulation.createForAutomatedTest(createRobotModel(), createYoVariableServer);
         }
      }).start();

      Ros2Node ros2Node = ROS2Tools.createRos2Node(pubSubMode, "test_node");

      if (VISUALIZE)
      {
         new Thread(() ->
         {
            LogTools.info("Creating robot and map viewer");
            robotAndMapViewer = new RobotAndMapViewer(createRobotModel(), ros2Node);
         }).start();
      }

      ThreadTools.sleepSeconds(2.0); // wait a bit for other threads to start

      // create map subscriber
      ROS2Input<PlanarRegionsListMessage> mapRegionsInput = new ROS2Input<>(ros2Node, PlanarRegionsListMessage.class, null, ROS2Tools.REA);

      // subscribe to robot pose
      RemoteSyncedHumanoidRobotState humanoidRobotState = new RemoteSyncedHumanoidRobotState(createRobotModel(), ros2Node);

      RemoteHumanoidRobotInterface robot = new RemoteHumanoidRobotInterface(ros2Node, createRobotModel());

      boolean fullyExpandVisibilityGraph = false;
      Point3D goal = new Point3D(6.0, 0.0, 0.0);
      FramePose3D robotPose = new FramePose3D();
      robotPose.setToZero(humanoidRobotState.pollHumanoidRobotState().getMidFeetZUpFrame());
      robotPose.changeFrame(ReferenceFrame.getWorldFrame());
      List<Point3DReadOnly> pathPoints = null;
      List<? extends Pose3DReadOnly> path = null;


      while (robotPose.getPosition().distance(goal) > 0.5)
      {
         VisibilityGraphsParametersBasics visibilityGraphParameters = new DefaultVisibilityGraphParameters();
         NavigableRegionsManager manager = new NavigableRegionsManager(visibilityGraphParameters, null, new ObstacleAvoidanceProcessor(visibilityGraphParameters));
         OcclusionHandlingPathPlanner occlusionHandlingPathPlanner = new OcclusionHandlingPathPlanner(manager);
         PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(visibilityGraphParameters);
         PlanarRegionsList latestMap = PlanarRegionMessageConverter.convertToPlanarRegionsList(mapRegionsInput.getLatest());
         manager.setPlanarRegions(latestMap.getPlanarRegionsAsList());
         LogTools.info("Planning with occlusions");
         pathPoints = occlusionHandlingPathPlanner.calculateBodyPath(robotPose.getPosition(), goal, fullyExpandVisibilityGraph);
         if (pathPoints == null)
         {
            LogTools.error("Path not found.");
            ThreadTools.sleepSeconds(1.0);
            continue;
         }
         LogTools.info("Found path.");
         try
         {
            LogTools.info("Computing poses from path");
            path = orientationCalculator.computePosesFromPath(pathPoints, manager.getVisibilityMapSolution(), robotPose.getOrientation(), robotPose.getOrientation());
         }
         catch (Exception e)
         {
            e.printStackTrace();
         }

         // request footstep plan
         WaypointDefinedBodyPathPlanHolder bodyPath = new WaypointDefinedBodyPathPlanHolder();

         bodyPath.setPoseWaypoints(path);

         Pose3D startPose = new Pose3D();
         bodyPath.getPointAlongPath(0.0, startPose);
         Pose3D finalPose = new Pose3D();
         bodyPath.getPointAlongPath(1.0, finalPose);

         YoVariableRegistry registry = new YoVariableRegistry("registry");
         FootstepPlannerParametersReadOnly parameters = new DefaultFootstepPlannerParameters();
         double defaultStepWidth = parameters.getIdealFootstepWidth();

         FramePose3D initialMidFootPose = new FramePose3D();
         initialMidFootPose.setX(startPose.getX());
         initialMidFootPose.setY(startPose.getY());
         initialMidFootPose.setOrientationYawPitchRoll(startPose.getYaw(), 0.0, 0.0);
         PoseReferenceFrame midFootFrame = new PoseReferenceFrame("InitialMidFootFrame", initialMidFootPose);

         RobotSide initialStanceFootSide = RobotSide.RIGHT;
         FramePose3D initialStanceFootPose = new FramePose3D(midFootFrame);
         initialStanceFootPose.setY(initialStanceFootSide.negateIfRightSide(defaultStepWidth / 2.0));
         initialStanceFootPose.changeFrame(ReferenceFrame.getWorldFrame());

         FramePose3D goalPose = new FramePose3D();
         goalPose.setX(finalPose.getX());
         goalPose.setY(finalPose.getY());
         goalPose.setOrientationYawPitchRoll(finalPose.getYaw(), 0.0, 0.0); // TODO: use initial yaw?

         FootstepPlannerParametersReadOnly footstepPlannerParameters = new DefaultFootstepPlannerParameters();
         FootstepNodeChecker nodeChecker = new AlwaysValidNodeChecker();
         FootstepNodeExpansion nodeExpansion = new ParameterBasedNodeExpansion(footstepPlannerParameters);
         FootstepCost stepCostCalculator = new EuclideanDistanceAndYawBasedCost(footstepPlannerParameters);
         FlatGroundFootstepNodeSnapper snapper = new FlatGroundFootstepNodeSnapper();
         CostToGoHeuristics heuristics = new BodyPathHeuristics(() -> 10.0, footstepPlannerParameters, snapper, bodyPath);

         AStarFootstepPlanner planner = new AStarFootstepPlanner(footstepPlannerParameters,
                                                                 nodeChecker,
                                                                 heuristics,
                                                                 nodeExpansion,
                                                                 stepCostCalculator,
                                                                 snapper,
                                                                 registry);
         LogTools.info("Running footstep planner");
         FootstepPlan footstepPlan = PlannerTools.runPlanner(planner, initialStanceFootPose, initialStanceFootSide, goalPose, latestMap, true);

         // make robot walk a little of the path
         if (VISUALIZE) robotAndMapViewer.setFootstepsToVisualize(footstepPlan);

         FootstepPlan shortenedFootstepPlan = new FootstepPlan();

         for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
         {
            SimpleFootstep footstep = footstepPlan.getFootstep(i);

            if (footstep.getSoleFramePose().getPosition().distance(robotPose.getPosition()) > 1.5)
            {
               break; // don't go farther than 0.3 meters
            }

            shortenedFootstepPlan.addFootstep(footstepPlan.getFootstep(i));
         }

         LogTools.info("Requesting walk");
         TypedNotification<WalkingStatusMessage> walkingStatusNotification = robot.requestWalk(FootstepDataMessageConverter.createFootstepDataListFromPlan(
               shortenedFootstepPlan,
               1.0,
               0.5,
               ExecutionMode.OVERRIDE));

         // wait for robot to finish walking
         Stopwatch stopwatch = new Stopwatch().start();
         double timeout = 3.0;
         while (!walkingStatusNotification.poll() && stopwatch.lapElapsed() < timeout)
         {
            ThreadTools.sleep(100);
         }
         if (stopwatch.lapElapsed() >= timeout)
         {
            LogTools.info("Walking timed out.");
         }

         robotPose.setToZero(humanoidRobotState.pollHumanoidRobotState().getMidFeetZUpFrame());
         robotPose.changeFrame(ReferenceFrame.getWorldFrame());
         LogTools.info("yaw: {}", robotPose.getYaw());
      }
   }

   private AtlasRobotModel createRobotModel()
   {
      FootContactPoints<RobotSide> simulationContactPoints = new AdditionalSimulationContactPoints<>(RobotSide.values, 8, 3, true, true);
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false, simulationContactPoints);
   }
}
