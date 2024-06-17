package us.ihmc.behaviors.navigation;

import perception_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.behaviors.behaviorTree.*;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.PerceptionAPI;
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
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.RemoteHumanoidRobotInterface;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanHolder;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegionsManager;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.ObstacleAvoidanceProcessor;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.PathOrientationCalculator;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.tools.Destroyable;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.pathPlanning.PlannerTestEnvironments.MAZE_CORRIDOR_SQUARE_SIZE;

/**
 * Used visibility graphs to walk through a maze.
 * @deprecated Not supported right now. Being kept for reference or revival.
 */
public class NavigationBehavior extends LocalOnlyBehaviorTreeNodeExecutor implements Destroyable
{
   private static final Point3D goal = new Point3D(MAZE_CORRIDOR_SQUARE_SIZE * 4.0, MAZE_CORRIDOR_SQUARE_SIZE, 0.0);

   private final BehaviorHelper helper;
   private final ROS2Input<PlanarRegionsListMessage> mapRegionsInput;
   private final RemoteHumanoidRobotInterface robotInterface;
   private final ROS2SyncedRobotModel syncedRobot;
   private final SideDependentList<ConvexPolygon2D> footPolygons;
   private final DefaultFootstepPlannerParametersBasics footstepPlannerParameters = new DefaultFootstepPlannerParameters();
   private final VisibilityGraphsParametersBasics visibilityGraphParameters = new DefaultVisibilityGraphParameters();

   private final FramePose3D robotPose = new FramePose3D();
   private final LoopSequenceNode sequence;
   private final Notification stepThroughAlgorithm = null;
   private final PausablePeriodicThread mainThread;

   private long latestMapSequenceId = 0;
   private List<Point3DReadOnly> pathPoints = null;
   private List<? extends Pose3DReadOnly> path = null;
   private NavigableRegionsManager navigableRegionsManager;
   private FootstepPlan latestFootstepPlan;
   private PlanarRegionsList latestMap;

   public NavigationBehavior(BehaviorHelper helper)
   {
      this.helper = helper;

      // create map subscriber
      mapRegionsInput = new ROS2Input<>(helper.getROS2Node(), PerceptionAPI.MAPPING_MODULE.withOutput().withTypeName(PlanarRegionsListMessage.class));

      robotInterface = helper.getOrCreateRobotInterface();
      syncedRobot = robotInterface.newSyncedRobot();

      RobotContactPointParameters<RobotSide> contactPointParameters = helper.getRobotModel().getContactPointParameters();
      footPolygons = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
      {
         ArrayList<Point2D> footPoints = contactPointParameters.getFootContactPoints().get(side);
         ConvexPolygon2D scaledFoot = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(footPoints));
         footPolygons.set(side, scaledFoot);
      }

//      stepThroughAlgorithm = helper.subscribeTypelessViaNotification(StepThroughAlgorithm);

      sequence = new LoopSequenceNode();
      sequence.getChildren().add(new AlwaysSuccessfulAction(() -> stepThroughAlgorithm("aquire map")));
      sequence.getChildren().add(new AlwaysSuccessfulAction(this::aquireMap));
      sequence.getChildren().add(new AlwaysSuccessfulAction(() -> stepThroughAlgorithm("plan body path")));
      sequence.getChildren().add(new AlwaysSuccessfulAction(this::planBodyPath));
      sequence.getChildren().add(new AlwaysSuccessfulAction(() -> stepThroughAlgorithm("plan body orientation trajectory and footsteps")));
      sequence.getChildren().add(new AlwaysSuccessfulAction(this::planBodyOrientationTrajectoryAndFootsteps));
      sequence.getChildren().add(new AlwaysSuccessfulAction(() -> stepThroughAlgorithm("shorten footstep plan and walk it")));
      sequence.getChildren().add(new AlwaysSuccessfulAction(this::shortenFootstepPlanAndWalkIt));

      mainThread = helper.createPausablePeriodicThread(getClass(), UnitConversions.hertzToSeconds(250), 5, sequence::tick);
   }

   @Override
   public BehaviorTreeNodeStatus determineStatus()
   {
      return BehaviorTreeNodeStatus.SUCCESS;
   }

   public void setEnabled(boolean enabled)
   {
      LogTools.info("Navigation behavior selected = {}", enabled);

      mainThread.setRunning(enabled);
   }

   private void stepThroughAlgorithm(String message)
   {
      LogTools.info("Press step to {}", message);
      stepThroughAlgorithm.blockingPoll();
   }

   private void aquireMap()
   {
      LogTools.info("Aquiring map...");
      do
      {
         mapRegionsInput.getMessageNotification().blockingPoll();
      }
      while (mapRegionsInput.getMessageNotification().read().getSequenceId() <= latestMapSequenceId);
      latestMapSequenceId = mapRegionsInput.getLatest().getSequenceId();
      //      ThreadTools.sleep(100); // try to get a little more perception data TODO wait for a SLAM update

//      helper.publish(MapRegionsForUI, PlanarRegionMessageConverter.convertToPlanarRegionsList(mapRegionsInput.getLatest()));
   }

   private void planBodyPath()
   {
      LogTools.info("Planning with occlusions");

      syncedRobot.update();
      robotPose.setToZero(syncedRobot.getReferenceFrames().getMidFeetZUpFrame());
      robotPose.changeFrame(ReferenceFrame.getWorldFrame());
      LogTools.info("Distance to goal: {}", robotPose.getPosition().distance(goal));

      visibilityGraphParameters.setNavigableExtrusionDistance(0.3);
      visibilityGraphParameters.setObstacleExtrusionDistance(0.8); // <-- this appears to be all that's necessary
      //         visibilityGraphParameters.setPreferredNavigableExtrusionDistance(0.60);
      //         visibilityGraphParameters.setPreferredObstacleExtrusionDistance(0.6);
      navigableRegionsManager = new NavigableRegionsManager(visibilityGraphParameters, null, new ObstacleAvoidanceProcessor(visibilityGraphParameters));
      latestMap = PlanarRegionMessageConverter.convertToPlanarRegionsList(mapRegionsInput.getLatest());

      if (latestMap.isEmpty())
      {
         LogTools.info("No regions yet.");
         ThreadTools.sleep(100);
         return;
      }

      navigableRegionsManager.setPlanarRegions(latestMap.getPlanarRegionsAsList());
      boolean fullyExpandVisibilityGraph = false;
      pathPoints = navigableRegionsManager.calculateBodyPathWithOcclusionHandling(robotPose.getPosition(), goal, fullyExpandVisibilityGraph);
      if (pathPoints == null || pathPoints.size() < 2)
      {
         LogTools.error("Path not found.");
         ThreadTools.sleepSeconds(1.0);
         return;
      }

      ArrayList<Pose3DReadOnly> pathPoses = new ArrayList<>();
      for (Point3DReadOnly pathPoint : pathPoints)
      {
         pathPoses.add(new Pose3D(pathPoint, new Quaternion()));
      }

//      helper.getMessager().submitMessage(BodyPathPlanForUI, pathPoses);
   }

   private void planBodyOrientationTrajectoryAndFootsteps()
   {
      LogTools.info("Computing poses from path");
      // find last two path points
      Point3DReadOnly lastPoint = pathPoints.get(pathPoints.size() - 1);
      Point3DReadOnly runnerUpPoint = pathPoints.get(pathPoints.size() - 2);
      Vector2D vector = new Vector2D(lastPoint);
      vector.sub(runnerUpPoint.getX(), runnerUpPoint.getY());
      vector.normalize();
      AxisAngle finalOrientation = new AxisAngle(Math.atan2(vector.getY(), vector.getX()), 0.0, 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(visibilityGraphParameters);
      path = orientationCalculator.computePosesFromPath(pathPoints, navigableRegionsManager.getVisibilityMapSolution(), robotPose.getOrientation(), finalOrientation);

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
      leftSolePose.setToZero(syncedRobot.getReferenceFrames().getSoleZUpFrame(RobotSide.LEFT));
      leftSolePose.changeFrame(ReferenceFrame.getWorldFrame());
      FramePose3D rightSolePose = new FramePose3D();
      rightSolePose.setToZero(syncedRobot.getReferenceFrames().getSoleZUpFrame(RobotSide.RIGHT));
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
      goalPose.getOrientation().setYawPitchRoll(finalPose.getYaw(), 0.0, 0.0); // TODO: use initial yaw?

//      LogTools.info("Creating A* planner");
//      YoRegistry registry = new YoRegistry("registry");
//      AStarFootstepPlanner planner = new AStarFootstepPlanner(footstepPlannerParameters,
//                                                              nodeChecker,
//                                                              heuristics,
//                                                              nodeExpansion,
//                                                              stepCostCalculator,
//                                                              snapper,
//                                                              registry);
//      LogTools.info("Running footstep planner");
//      planner.setPlanningHorizonLength(100.0);
//      FootstepPlannerGoal footstepPlannerGoal = new FootstepPlannerGoal();
//      footstepPlannerGoal.setFootstepPlannerGoalType(FootstepPlannerGoalType.POSE_BETWEEN_FEET);
//      footstepPlannerGoal.setGoalPoseBetweenFeet(goalPose);
//      planner.setPlanarRegions(latestMap);
//      planner.setInitialStanceFoot(initialStanceFootPose, initialStanceFootSide);
//      planner.setGoal(footstepPlannerGoal);
//      planner.setBestEffortTimeout(2.0);
//
//      Stopwatch footstepPlannerStopwatch = new Stopwatch().start();
//      FootstepPlanningResult result = planner.plan();
//      LogTools.info("Planning took " + footstepPlannerStopwatch.lapElapsed() + "s");
//
//      if (!result.validForExecution())
//      {
//         LogTools.error("Footstep plan not valid for execution! {}", result);
//
//         HashMap<BipedalFootstepPlannerNodeRejectionReason, Integer> reasons = new HashMap<>();
//         for (PlannerNodeData nodeDatum : planner.getPlannerStatistics().getFullGraph().getNodeData())
//         {
//            BipedalFootstepPlannerNodeRejectionReason rejectionReason = nodeDatum.getRejectionReason();
//            if (!reasons.containsKey(rejectionReason))
//            {
//               reasons.put(rejectionReason, 1);
//            }
//            else
//            {
//               reasons.put(rejectionReason, reasons.get(rejectionReason) + 1);
//            }
//         }
//         for (BipedalFootstepPlannerNodeRejectionReason rejectionReason : reasons.keySet())
//         {
//            System.out.println("Reason: " + rejectionReason + "  " + reasons.get(rejectionReason));
//         }
//
//         ThreadTools.sleep(1000);
//         return;
//      }
//
//      latestFootstepPlan = planner.getPlan();
//      LogTools.info("Got {} footsteps", latestFootstepPlan.getNumberOfSteps());
//
//      // make robot walk a little of the path
//      helper.publish(FootstepPlanForUI, FootstepDataMessageConverter.reduceFootstepPlanForUIMessager(latestFootstepPlan));
   }

   private void shortenFootstepPlanAndWalkIt()
   {
      FootstepPlan shortenedFootstepPlan = new FootstepPlan();

      for (int i = 0; i < latestFootstepPlan.getNumberOfSteps(); i++)
      {
         PlannedFootstep footstep = latestFootstepPlan.getFootstep(i);

         if (footstep.getFootstepPose().getPosition().distance(robotPose.getPosition()) > 2.0)
         {
            break; // don't go farther than 0.3 meters
         }

         shortenedFootstepPlan.addFootstep(latestFootstepPlan.getFootstep(i));
      }

      LogTools.info("Requesting walk");
      TypedNotification<WalkingStatusMessage> walkingStatusNotification = robotInterface.requestWalk(FootstepDataMessageConverter.createFootstepDataListFromPlan(
            shortenedFootstepPlan,
            1.0,
            0.5));

      // wait for robot to finish walking
      Stopwatch stopwatch = new Stopwatch().start();
      double timeout = 3.0;
      while (!walkingStatusNotification.poll() && stopwatch.lapElapsed() < timeout)
      {
         syncedRobot.update();
         robotPose.setToZero(syncedRobot.getReferenceFrames().getMidFeetZUpFrame());
         robotPose.changeFrame(ReferenceFrame.getWorldFrame());

         ThreadTools.sleep(100);
      }
      if (stopwatch.lapElapsed() >= timeout)
      {
         LogTools.info("Walking timed out.");
      }

      LogTools.info("Robot position: x: {}, y: {}", robotPose.getPosition().getX(), robotPose.getPosition().getY());
   }

   public static class NavigationBehaviorAPI
   {

   }
}
