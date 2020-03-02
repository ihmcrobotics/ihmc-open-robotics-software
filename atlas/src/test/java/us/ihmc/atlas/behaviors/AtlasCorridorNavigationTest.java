package us.ihmc.atlas.behaviors;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import org.junit.jupiter.api.*;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulationParameters;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.ROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.collision.FootstepNodeBodyCollisionDetector;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FlatGroundFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.PlannerNodeData;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.tools.statistics.GraphSearchStatistics;
import us.ihmc.humanoidBehaviors.tools.HumanoidRobotState;
import us.ihmc.humanoidBehaviors.tools.PlanarRegionsMappingModule;
import us.ihmc.humanoidBehaviors.tools.RemoteHumanoidRobotInterface;
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
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.thread.TypedNotification;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

import java.time.Duration;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.pathPlanning.PlannerTestEnvironments.MAZE_CORRIDOR_SQUARE_SIZE;

@Tag("humanoid-behaviors")
public class AtlasCorridorNavigationTest
{
   private static boolean VISUALIZE = Boolean.parseBoolean(System.getProperty("visualize")); // To visualize, pass -Dvisualize=true
   private static boolean KEEP_VISUALIZATION_UP = VISUALIZE && Boolean.parseBoolean(System.getProperty("keep.visualization.up"));
   private static boolean LOG_TO_FILE = Boolean.parseBoolean(System.getProperty("log.to.file"));
   private static boolean CREATE_YOVARIABLE_SERVER = Boolean.parseBoolean(System.getProperty("create.yovariable.server"));

   private RobotAndMapViewer robotAndMapViewer;
   private PubSubImplementation pubSubMode = PubSubImplementation.INTRAPROCESS;
   private Notification slamUpdated;

   @BeforeAll
   static public void beforeAll()
   {
      if (VISUALIZE) JavaFXApplicationCreator.createAJavaFXApplication();
   }

   @AfterEach
   public void afterEach()
   {
      if (KEEP_VISUALIZATION_UP) ThreadTools.sleepForever();
   }

   @Disabled
   @Test
   public void testAtlasMakesItToGoalInTrickyCorridor()
   {
      ArrayDeque<Pose3D> waypointsToHit = new ArrayDeque<>();
      waypointsToHit.addLast(new Pose3D(new Point3D(0.8 * MAZE_CORRIDOR_SQUARE_SIZE, -0.5 * MAZE_CORRIDOR_SQUARE_SIZE, 0.0), new Quaternion()));
      waypointsToHit.addLast(new Pose3D(new Point3D(-0.7 * MAZE_CORRIDOR_SQUARE_SIZE, -1.0 * MAZE_CORRIDOR_SQUARE_SIZE, 0.0), new Quaternion()));
      waypointsToHit.addLast(new Pose3D(new Point3D(0.0 * MAZE_CORRIDOR_SQUARE_SIZE, -1.5 * MAZE_CORRIDOR_SQUARE_SIZE, 0.0), new Quaternion()));
      waypointsToHit.addLast(new Pose3D(new Point3D(2.3 * MAZE_CORRIDOR_SQUARE_SIZE, -1.0 * MAZE_CORRIDOR_SQUARE_SIZE, 0.0), new Quaternion()));
      performTestWithTimeoutAndExceptions(PlannerTestEnvironments.getTrickCorridorWidened(),
                                          new Point3D(2.3 * MAZE_CORRIDOR_SQUARE_SIZE, 0.0, 0.0),
                                          5,
                                          waypointsToHit);
   }

   @Disabled
   @Test
   public void testAtlasMakesItToGoalInMazeCorridor()
   {
      ArrayDeque<Pose3D> waypointsToHit = new ArrayDeque<>();
      waypointsToHit.addLast(new Pose3D(new Point3D(0.5 * MAZE_CORRIDOR_SQUARE_SIZE, 1.0 * MAZE_CORRIDOR_SQUARE_SIZE, 0.0), new Quaternion()));
      waypointsToHit.addLast(new Pose3D(new Point3D(2.0 * MAZE_CORRIDOR_SQUARE_SIZE, 0.0 * MAZE_CORRIDOR_SQUARE_SIZE, 0.0), new Quaternion()));
      waypointsToHit.addLast(new Pose3D(new Point3D(0.5 * MAZE_CORRIDOR_SQUARE_SIZE, 1.0 * MAZE_CORRIDOR_SQUARE_SIZE, 0.0), new Quaternion()));
      waypointsToHit.addLast(new Pose3D(new Point3D(0.0 * MAZE_CORRIDOR_SQUARE_SIZE, 2.0 * MAZE_CORRIDOR_SQUARE_SIZE, 0.0), new Quaternion()));
      waypointsToHit.addLast(new Pose3D(new Point3D(1.5 * MAZE_CORRIDOR_SQUARE_SIZE, 3.0 * MAZE_CORRIDOR_SQUARE_SIZE, 0.0), new Quaternion()));
      waypointsToHit.addLast(new Pose3D(new Point3D(4.0 * MAZE_CORRIDOR_SQUARE_SIZE, 2.5 * MAZE_CORRIDOR_SQUARE_SIZE, 0.0), new Quaternion()));
      waypointsToHit.addLast(new Pose3D(new Point3D(2.0 * MAZE_CORRIDOR_SQUARE_SIZE, 1.5 * MAZE_CORRIDOR_SQUARE_SIZE, 0.0), new Quaternion()));
      performTestWithTimeoutAndExceptions(PlannerTestEnvironments.getMazeCorridor(),
                                          new Point3D(MAZE_CORRIDOR_SQUARE_SIZE * 4.0, MAZE_CORRIDOR_SQUARE_SIZE, 0.0),
                                          10, waypointsToHit);
   }

   private void performTestWithTimeoutAndExceptions(PlanarRegionsList map, Point3D goal, int timeout, ArrayDeque<Pose3D> waypointsToHit)
   {
      assertTimeoutPreemptively(Duration.ofMinutes(timeout), () ->
      {
         try
         {
            runAtlasToGoalUsingBodyPathWithOcclusions(map, goal, waypointsToHit);
         }
         catch (Exception e)
         {
            e.printStackTrace();
         }
      });
   }

   private void runAtlasToGoalUsingBodyPathWithOcclusions(PlanarRegionsList map, Point3D goal, ArrayDeque<Pose3D> waypointsToHit)
   {
      new Thread(() ->
      {
         LogTools.info("Creating simulated REA module");
         SimulatedREAModule simulatedREAModule = new SimulatedREAModule(map, createRobotModel(), pubSubMode);
         simulatedREAModule.start();
      }).start();

      new Thread(() ->
      {
         LogTools.info("Creating planar regions mapping module");
         PlanarRegionsMappingModule planarRegionsMappingModule = new PlanarRegionsMappingModule(pubSubMode);
         slamUpdated = planarRegionsMappingModule.getSlamUpdated();
      }).start();

      new Thread(() ->
      {
         LogTools.info("Creating simulation");
         boolean createYoVariableServer = true;
         HumanoidKinematicsSimulationParameters kinematicsSimulationParameters = new HumanoidKinematicsSimulationParameters();
         kinematicsSimulationParameters.setPubSubImplementation(pubSubMode);
         kinematicsSimulationParameters.setLogToFile(LOG_TO_FILE);
         kinematicsSimulationParameters.setCreateYoVariableServer(CREATE_YOVARIABLE_SERVER);
         AtlasKinematicSimulation.create(createRobotModel(), kinematicsSimulationParameters);
      }).start();

      Ros2Node ros2Node = ROS2Tools.createRos2Node(pubSubMode, "test_node");

      if (VISUALIZE)
      {
         // option to launch SCS 2
//         new Thread(() -> JavaFXMissingTools.runApplication(new SessionV))

         new Thread(() ->
         {
            LogTools.info("Creating robot and map viewer");
            robotAndMapViewer = new RobotAndMapViewer(createRobotModel(), ros2Node);
         }).start();
      }

      ThreadTools.sleepSeconds(5.0); // wait a bit for other threads to start

      // create map subscriber
      ROS2Input<PlanarRegionsListMessage> mapRegionsInput = new ROS2Input<>(ros2Node, PlanarRegionsListMessage.class, null, ROS2Tools.MAPPING_MODULE);

      // subscribe to robot pose
      RemoteHumanoidRobotInterface robot = new RemoteHumanoidRobotInterface(ros2Node, createRobotModel());
      SideDependentList<ConvexPolygon2D> footPolygons = createFootPolygons();
      //      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();

      boolean fullyExpandVisibilityGraph = false;
      FramePose3D robotPose = new FramePose3D();
      HumanoidRobotState latestHumanoidRobotState = robot.pollHumanoidRobotState();
      robotPose.setToZero(latestHumanoidRobotState.getMidFeetZUpFrame());
      robotPose.changeFrame(ReferenceFrame.getWorldFrame());
      List<Point3DReadOnly> pathPoints = null;
      List<? extends Pose3DReadOnly> path = null;
      //         FootstepPlannerParametersBasics footstepPlannerParameters = new AtlasFootstepPlannerParameters();
      FootstepPlannerParametersBasics footstepPlannerParameters = new DefaultFootstepPlannerParameters();
      VisibilityGraphsParametersBasics visibilityGraphParameters = new DefaultVisibilityGraphParameters();

      waypointsToHit.addFirst(new Pose3D());
      waypointsToHit.addLast(new Pose3D(goal, new Quaternion()));
      int waypointOriginalSize = waypointsToHit.size();

      if (VISUALIZE)
      {
         for (Pose3D waypoint : waypointsToHit)
         {
            robotAndMapViewer.addMarker(waypoint.getPosition());
         }
         robotAndMapViewer.setGoalLocation(goal);
      }

      while (!waypointsToHit.isEmpty())
      {
         LogTools.info("Waiting for SLAM update");
         ThreadTools.sleep(300); // try to get a little more perception data TODO wait for a SLAM update
         slamUpdated.poll(); // throw away a poll. Make sure to get a new scan here
         while (!slamUpdated.poll())
         {
            ThreadTools.sleep(100);
         }
         mapRegionsInput.getMessageNotification().blockingPoll();
         ThreadTools.sleep(100); // try to get a little more perception data TODO wait for a SLAM update

         LogTools.info("Planning with occlusions");
         latestHumanoidRobotState = robot.pollHumanoidRobotState();
         robotPose.setToZero(latestHumanoidRobotState.getMidFeetZUpFrame());
         robotPose.changeFrame(ReferenceFrame.getWorldFrame());
         LogTools.info("Distance to goal: {}", robotPose.getPosition().distance(goal));

         visibilityGraphParameters.setNavigableExtrusionDistance(0.3);
         visibilityGraphParameters.setObstacleExtrusionDistance(0.8); // <-- this appears to be all that's necessary
//         visibilityGraphParameters.setPreferredNavigableExtrusionDistance(0.60);
//         visibilityGraphParameters.setPreferredObstacleExtrusionDistance(0.6);
         NavigableRegionsManager manager = new NavigableRegionsManager(visibilityGraphParameters, null, new ObstacleAvoidanceProcessor(visibilityGraphParameters));
         OcclusionHandlingPathPlanner occlusionHandlingPathPlanner = new OcclusionHandlingPathPlanner(manager);
         PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(visibilityGraphParameters);
         PlanarRegionsList latestMap = PlanarRegionMessageConverter.convertToPlanarRegionsList(mapRegionsInput.getLatest());

         if (latestMap.isEmpty())
         {
            LogTools.info("No regions yet.");
            ThreadTools.sleep(100);
            continue;
         }

         manager.setPlanarRegions(latestMap.getPlanarRegionsAsList());
         pathPoints = occlusionHandlingPathPlanner.calculateBodyPath(robotPose.getPosition(), goal, fullyExpandVisibilityGraph);
         if (pathPoints == null || pathPoints.size() < 2)
         {
            LogTools.error("Path not found.");
            ThreadTools.sleepSeconds(1.0);
            continue;
         }
         if (VISUALIZE)
         {
            robotAndMapViewer.setBodyPathPlanToVisualize(pathPoints);
         }

         LogTools.info("Computing poses from path");
         // find last two path points
         Point3DReadOnly lastPoint = pathPoints.get(pathPoints.size() - 1);
         Point3DReadOnly runnerUpPoint = pathPoints.get(pathPoints.size() - 2);
         Vector2D vector = new Vector2D(lastPoint);
         vector.sub(runnerUpPoint.getX(), runnerUpPoint.getY());
         vector.normalize();
         AxisAngle finalOrientation = new AxisAngle(Math.atan2(vector.getY(), vector.getX()), 0.0, 0.0);

         path = orientationCalculator.computePosesFromPath(pathPoints, manager.getVisibilityMapSolution(), robotPose.getOrientation(), finalOrientation);

         // request footstep plan
         WaypointDefinedBodyPathPlanHolder bodyPath = new WaypointDefinedBodyPathPlanHolder();

         bodyPath.setPoseWaypoints(path);

         Pose3D startPose = new Pose3D();
         bodyPath.getPointAlongPath(0.0, startPose);
         Pose3D finalPose = new Pose3D();
         bodyPath.getPointAlongPath(1.0, finalPose);

         RobotSide initialStanceFootSide = null;
         FramePose3D initialStanceFootPose = null;
         FramePose3D leftSolePose = new FramePose3D();
         leftSolePose.setToZero(latestHumanoidRobotState.getSoleZUpFrame(RobotSide.LEFT));
         leftSolePose.changeFrame(ReferenceFrame.getWorldFrame());
         FramePose3D rightSolePose = new FramePose3D();
         rightSolePose.setToZero(latestHumanoidRobotState.getSoleZUpFrame(RobotSide.RIGHT));
         rightSolePose.changeFrame(ReferenceFrame.getWorldFrame());

         if (leftSolePose.getPosition().distance(finalPose.getPosition()) <= rightSolePose.getPosition().distance(finalPose.getPosition()))
         {
            initialStanceFootSide = RobotSide.LEFT;
            initialStanceFootPose = leftSolePose;
         }
         else
         {
            initialStanceFootSide = RobotSide.RIGHT;
            initialStanceFootPose = rightSolePose;
         }

         FramePose3D goalPose = new FramePose3D();
         goalPose.setX(finalPose.getX());
         goalPose.setY(finalPose.getY());
         goalPose.setOrientationYawPitchRoll(finalPose.getYaw(), 0.0, 0.0); // TODO: use initial yaw?

         // TODO: Figure out how to best effort plan with footstep snapping
         // Use BodyPathBasedAStarPlanner instead of manual?

         boolean useFastFlatInvalidFootsteps = true;
         footstepPlannerParameters.setReturnBestEffortPlan(true);
         footstepPlannerParameters.setMaximumStepYaw(1.5);
         FootstepNodeBodyCollisionDetector collisionDetector = new FootstepNodeBodyCollisionDetector(footstepPlannerParameters);
         FootstepNodeSnapper snapper;
         if (useFastFlatInvalidFootsteps)
         {
            snapper = new FlatGroundFootstepNodeSnapper();
         }
         else
         {
            snapper = new SimplePlanarRegionFootstepNodeSnapper(footPolygons);
         }

         LogTools.info("Creating planner");
         LogTools.info("Running footstep planner");

         FootstepPlanningModule planner = new FootstepPlanningModule(getClass().getSimpleName());

         FootstepPlannerRequest request = new FootstepPlannerRequest();
         request.setHorizonLength(100.0);
         request.setGoalPose(goalPose);
         request.setPlanarRegionsList(latestMap);
         request.setInitialStancePose(initialStanceFootPose);
         request.setInitialStanceSide(initialStanceFootSide);
         request.setTimeout(2.0);

         Stopwatch footstepPlannerStopwatch = new Stopwatch().start();
         FootstepPlannerOutput plannerOutput = planner.handleRequest(request);
         LogTools.info("Planning took " + footstepPlannerStopwatch.lapElapsed() + "s");

         if (!plannerOutput.getResult().validForExecution())
         {
            LogTools.error("Footstep plan not valid for execution! {}", plannerOutput.getResult());

            GraphSearchStatistics graphSearchStatistics = new GraphSearchStatistics();
            graphSearchStatistics.set(planner);
            HashMap<BipedalFootstepPlannerNodeRejectionReason, Integer> reasons = new HashMap<>();
            for (PlannerNodeData nodeDatum : graphSearchStatistics.getFullGraph().getNodeData())
            {
               BipedalFootstepPlannerNodeRejectionReason rejectionReason = nodeDatum.getRejectionReason();
               if (!reasons.containsKey(rejectionReason))
               {
                  reasons.put(rejectionReason, 1);
               }
               else
               {
                  reasons.put(rejectionReason, reasons.get(rejectionReason) + 1);
               }
            }
            for (BipedalFootstepPlannerNodeRejectionReason rejectionReason : reasons.keySet())
            {
               System.out.println("Reason: " + rejectionReason + "  " + reasons.get(rejectionReason));
            }

            ThreadTools.sleep(1000);
            continue;
         }

         FootstepPlan footstepPlan = plannerOutput.getFootstepPlan();
         LogTools.info("Got {} footsteps", footstepPlan.getNumberOfSteps());

         // make robot walk a little of the path
         if (VISUALIZE)
         {
            robotAndMapViewer.setFootstepsToVisualize(footstepPlan);
         }

         FootstepPlan shortenedFootstepPlan = new FootstepPlan();

         for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
         {
            SimpleFootstep footstep = footstepPlan.getFootstep(i);

            if (footstep.getSoleFramePose().getPosition().distance(robotPose.getPosition()) > 2.0)
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
            latestHumanoidRobotState = robot.pollHumanoidRobotState();
            robotPose.setToZero(latestHumanoidRobotState.getMidFeetZUpFrame());
            robotPose.changeFrame(ReferenceFrame.getWorldFrame());
            if (!waypointsToHit.isEmpty() && robotPose.getPositionDistance(waypointsToHit.peekFirst()) < 1.0)
            {
               LogTools.info("Robot position: x: {}, y: {}", robotPose.getPosition().getX(), robotPose.getPosition().getY());
               LogTools.info("Waypoint {} reached: x: {}, y: {}",
                             waypointOriginalSize - waypointsToHit.size(),
                             waypointsToHit.peekFirst().getX(),
                             waypointsToHit.peekFirst().getY());
               waypointsToHit.pollFirst();
            }
            ThreadTools.sleep(100);
         }
         if (stopwatch.lapElapsed() >= timeout)
         {
            LogTools.info("Walking timed out.");
         }

         LogTools.info("Robot position: x: {}, y: {}", robotPose.getPosition().getX(), robotPose.getPosition().getY());
      }
   }

   private AtlasRobotModel createRobotModel()
   {
      FootContactPoints<RobotSide> simulationContactPoints = new AdditionalSimulationContactPoints<>(RobotSide.values, 8, 3, true, true);
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false, simulationContactPoints);
   }

   private SideDependentList<ConvexPolygon2D> createFootPolygons()
   {
      AtlasContactPointParameters contactPointParameters = createRobotModel().getContactPointParameters();
      SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
      {
         ArrayList<Point2D> footPoints = contactPointParameters.getFootContactPoints().get(side);
         ConvexPolygon2D scaledFoot = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(footPoints));
         footPolygons.set(side, scaledFoot);
      }

      return footPolygons;
   }
}
