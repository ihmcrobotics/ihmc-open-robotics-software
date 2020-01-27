package us.ihmc.humanoidBehaviors.navigation;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
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
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.collision.FootstepNodeBodyCollisionDetector;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FlatGroundFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.PlannerNodeData;
import us.ihmc.footstepPlanning.graphSearch.heuristics.BodyPathHeuristics;
import us.ihmc.footstepPlanning.graphSearch.heuristics.CostToGoHeuristics;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.*;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.planners.AStarFootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.stepCost.EuclideanDistanceAndYawBasedCost;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCost;
import us.ihmc.humanoidBehaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.BehaviorDefinition;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.humanoidBehaviors.tools.HumanoidRobotState;
import us.ihmc.humanoidBehaviors.tools.RemoteHumanoidRobotInterface;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.log.LogTools;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanHolder;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegionsManager;
import us.ihmc.pathPlanning.visibilityGraphs.OcclusionHandlingPathPlanner;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.ObstacleAvoidanceProcessor;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.PathOrientationCalculator;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.tools.thread.TypedNotification;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.humanoidBehaviors.navigation.NavigationBehavior.NavigationBehaviorAPI.*;
import static us.ihmc.pathPlanning.PlannerTestEnvironments.MAZE_CORRIDOR_SQUARE_SIZE;

public class NavigationBehavior implements BehaviorInterface
{
   public static final BehaviorDefinition DEFINITION = new BehaviorDefinition("Navigation", NavigationBehavior::new, NavigationBehaviorAPI.create());

   private static final Point3D goal = new Point3D(MAZE_CORRIDOR_SQUARE_SIZE * 4.0, MAZE_CORRIDOR_SQUARE_SIZE, 0.0);

   private final BehaviorHelper helper;
   private final ROS2Input<PlanarRegionsListMessage> mapRegionsInput;
   private final RemoteHumanoidRobotInterface robot;
   private final SideDependentList<ConvexPolygon2D> footPolygons;
   private final FootstepPlannerParametersBasics footstepPlannerParameters = new DefaultFootstepPlannerParameters();
   private final VisibilityGraphsParametersBasics visibilityGraphParameters = new DefaultVisibilityGraphParameters();

   private final FramePose3D robotPose = new FramePose3D();
   private final PausablePeriodicThread mainThread;

   private long latestMapSequenceId = 0;
   private HumanoidRobotState latestHumanoidRobotState;
   private List<Point3DReadOnly> pathPoints = null;
   private List<? extends Pose3DReadOnly> path = null;
   private final Notification stepThroughAlgorithm;

   public NavigationBehavior(BehaviorHelper helper)
   {
      this.helper = helper;

      // create map subscriber
      mapRegionsInput = new ROS2Input<>(helper.getManagedROS2Node(), PlanarRegionsListMessage.class, null, ROS2Tools.MAPPING_MODULE);

      robot = helper.getOrCreateRobotInterface();

      footPolygons = createFootPolygons(helper.getRobotModel());

      stepThroughAlgorithm = helper.createUINotification(StepThroughAlgorithm);

      mainThread = helper.createPausablePeriodicThread(getClass(), UnitConversions.hertzToSeconds(250), 5, this::navigate);
   }

   @Override
   public void setEnabled(boolean enabled)
   {
      LogTools.info("Navigation behavior selected = {}", enabled);

      mainThread.setRunning(enabled);
      helper.setCommunicationCallbacksEnabled(enabled);
   }

   private void navigate()
   {
      do
      {
         mapRegionsInput.getMessageNotification().blockingPoll();
      }
      while (mapRegionsInput.getMessageNotification().peek().getSequenceId() <= latestMapSequenceId);
      latestMapSequenceId = mapRegionsInput.getMessageNotification().peek().getSequenceId();
//      ThreadTools.sleep(100); // try to get a little more perception data TODO wait for a SLAM update

      stepThroughAlgorithm.blockingPoll();
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
         return;
      }

      manager.setPlanarRegions(latestMap.getPlanarRegionsAsList());
      boolean fullyExpandVisibilityGraph = false;
      pathPoints = occlusionHandlingPathPlanner.calculateBodyPath(robotPose.getPosition(), goal, fullyExpandVisibilityGraph);
      if (pathPoints == null || pathPoints.size() < 2)
      {
         LogTools.error("Path not found.");
         ThreadTools.sleepSeconds(1.0);
         return;
      }

      helper.getManagedMessager().submitMessage(BodyPathPlanForUI, new ArrayList<>(pathPoints));

      stepThroughAlgorithm.blockingPoll();
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
      FootstepNodeChecker snapBasedNodeChecker = new SnapBasedNodeChecker(footstepPlannerParameters, footPolygons, snapper);
      BodyCollisionNodeChecker bodyCollisionNodeChecker = new BodyCollisionNodeChecker(collisionDetector, footstepPlannerParameters, snapper);
      PlanarRegionBaseOfCliffAvoider cliffAvoider = new PlanarRegionBaseOfCliffAvoider(footstepPlannerParameters, snapper, footPolygons);
      FootstepNodeChecker nodeChecker;
      if (useFastFlatInvalidFootsteps)
      {
         nodeChecker = new AlwaysValidNodeChecker();
      }
      else
      {
         // nodeChecker = new FootstepNodeCheckerOfCheckers(Arrays.asList(snapBasedNodeChecker, bodyCollisionNodeChecker, cliffAvoider));
         nodeChecker = new FootstepNodeCheckerOfCheckers(Arrays.asList(snapBasedNodeChecker));
      }
      FootstepNodeExpansion nodeExpansion = new ParameterBasedNodeExpansion(footstepPlannerParameters);
      FootstepCost stepCostCalculator = new EuclideanDistanceAndYawBasedCost(footstepPlannerParameters);
      CostToGoHeuristics heuristics = new BodyPathHeuristics(() -> 10.0, footstepPlannerParameters, snapper, bodyPath);

      LogTools.info("Creating A* planner");
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      AStarFootstepPlanner planner = new AStarFootstepPlanner(footstepPlannerParameters,
                                                              nodeChecker,
                                                              heuristics,
                                                              nodeExpansion,
                                                              stepCostCalculator,
                                                              snapper,
                                                              registry);
      LogTools.info("Running footstep planner");
      planner.setPlanningHorizonLength(100.0);
      FootstepPlannerGoal footstepPlannerGoal = new FootstepPlannerGoal();
      footstepPlannerGoal.setFootstepPlannerGoalType(FootstepPlannerGoalType.POSE_BETWEEN_FEET);
      footstepPlannerGoal.setGoalPoseBetweenFeet(goalPose);
      planner.setPlanarRegions(latestMap);
      planner.setInitialStanceFoot(initialStanceFootPose, initialStanceFootSide);
      planner.setGoal(footstepPlannerGoal);
      planner.setBestEffortTimeout(2.0);

      Stopwatch footstepPlannerStopwatch = new Stopwatch().start();
      FootstepPlanningResult result = planner.plan();
      LogTools.info("Planning took " + footstepPlannerStopwatch.lapElapsed() + "s");

      if (!result.validForExecution())
      {
         LogTools.error("Footstep plan not valid for execution! {}", result);

         HashMap<BipedalFootstepPlannerNodeRejectionReason, Integer> reasons = new HashMap<>();
         for (PlannerNodeData nodeDatum : planner.getPlannerStatistics().getFullGraph().getNodeData())
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
         return;
      }

      FootstepPlan footstepPlan = planner.getPlan();
      LogTools.info("Got {} footsteps", footstepPlan.getNumberOfSteps());

      // make robot walk a little of the path
      helper.publishToUI(FootstepPlanForUI, FootstepDataMessageConverter.reduceFootstepPlanForUIMessager(footstepPlan));

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

      stepThroughAlgorithm.blockingPoll();
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

         ThreadTools.sleep(100);
      }
      if (stopwatch.lapElapsed() >= timeout)
      {
         LogTools.info("Walking timed out.");
      }

      LogTools.info("Robot position: x: {}, y: {}", robotPose.getPosition().getX(), robotPose.getPosition().getY());
   }

   private SideDependentList<ConvexPolygon2D> createFootPolygons(DRCRobotModel robotModel)
   {
      RobotContactPointParameters<RobotSide> contactPointParameters = robotModel.getContactPointParameters();
      SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
      {
         ArrayList<Point2D> footPoints = contactPointParameters.getFootContactPoints().get(side);
         ConvexPolygon2D scaledFoot = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(footPoints));
         footPolygons.set(side, scaledFoot);
      }

      return footPolygons;
   }

   public static class NavigationBehaviorAPI
   {
      private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
      private static final Category RootCategory = apiFactory.createRootCategory("NavigationBehavior");
      private static final CategoryTheme NavigationTheme = apiFactory.createCategoryTheme("Navigation");

      public static final Topic<Object> StepThroughAlgorithm = topic("StepThroughAlgorithm");
      public static final Topic<ArrayList<Point3DReadOnly>> BodyPathPlanForUI = topic("BodyPathPoints");
      public static final Topic<ArrayList<Pair<RobotSide, Pose3D>>> FootstepPlanForUI = topic("FootstepPlan");

      private static final <T> Topic<T> topic(String name)
      {
         return RootCategory.child(NavigationTheme).topic(apiFactory.createTypedTopicTheme(name));
      }

      public static final MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }
}
