package us.ihmc.behaviors.behaviorTree.control.buildingExploration;

import static us.ihmc.behaviors.behaviorTree.BehaviorTreeTools.searchDFSFirstMatch;

import controller_msgs.AbortWalkingMessage;
import controller_msgs.ControllerWaypointGoalListMessage;
import controller_msgs.ControllerWaypointGoalMessage;
import geometry_msgs.PoseStamped;
import nav_msgs.Path;
import std_msgs.Bool;
import std_msgs.String_;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeTools;
import us.ihmc.behaviors.behaviorTree.LeafNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.actions.WaitActionExecutor;
import us.ihmc.behaviors.behaviorTree.control.door.DoorTraversalExecutor;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.CSGROS2CommunicationHelper;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Heartbeat;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2QoSProfile;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.detections.yolo.YOLOv8InstantDetection;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.tools.NonWallTimer;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

/**
 * Explores a building with an exploration planner from ihmc-exploration-planner, the SceneComposer
 * planner or GBPlanner, and hands each door it meets to a DoorTraversal child. Both planners have the
 * same topics under their own namespace ({@link #PLANNER_NAMESPACES}); this node talks to both, and
 * whichever one is running answers, so the planner is chosen by which one is started.
 * <p>
 * The tree around it (behaviorTrees/exploration/BuildingExploration.json) is a loop: a wait named
 * {@link #EXPLORE_NODE_NAME}, then DoorTraversal, then a Goto back to the wait. While that wait runs,
 * this node enables the planner and walks the robot along its path with RL waypoints. When YOLO sees a
 * door within {@link #DOOR_AHEAD_DISTANCE} in front of the robot, it stops walking, disables the
 * planner and ends the wait, so the tree continues into DoorTraversal. Only a door near the planned path
 * counts, so doors off to the side are passed by. If a DoorTraversal step fails, the tree stays stopped at
 * that step, as the door behavior does on its own, so the operator can retry it or finish the door by
 * hand. When exploring starts again with the robot still in front of that door, the door is skipped for
 * the rest of the run. Pausing the tree's automatic execution stops exploring and leaves the tree at the
 * wait, so pressing play resumes it.
 */
public class BuildingExplorationExecutor extends BehaviorTreeNodeExecutor<BuildingExplorationState, BuildingExplorationDefinition>
{
   public static final String START_NODE_NAME = "Start exploration";
   public static final String EXPLORE_NODE_NAME = "Explore until a door is ahead";
   /** One line on what this node is doing, for the operator UI's Exploration panel */
   public static final String STATUS_TOPIC = "/ihmc/building_exploration/status";
   /** Each has path, status, enable_command and skip_goal_command topics; run one of them at a time */
   public static final List<String> PLANNER_NAMESPACES = List.of("/ihmc_scenecomposer_planner", "/ihmc_gbplanner");

   /** YOLO classes that mean a door: the panel, or what opens it, as in SceneActionExecutor's door panel setup */
   private static final Set<String> DOOR_CLASSES = Set.of("door_panel", "door_lever", "door_knob", "door_push_bar", "door_pull_handle");
   private static final int DOOR_MINIMUM_HISTORY = 5;
   private static final double DOOR_AHEAD_DISTANCE = 3.0;
   private static final double DOOR_AHEAD_HALF_ANGLE = Math.toRadians(45.0);
   /** How far past the door, along the way the robot came, counts as through it */
   private static final double THROUGH_DOOR_DISTANCE = 0.3;
   /** A door just traversed is ignored until the robot is this far from it, so it doesn't trigger again */
   private static final double TRAVERSED_DOOR_REARM_DISTANCE = 4.0;
   private static final double TRAVERSED_DOOR_RADIUS = 1.5;
   /** While the planner has a path, a door further than this from it is not on the way and is passed by */
   private static final double DOOR_PATH_DISTANCE = 1.0;

   /** The step generator drops waypoint goals until it has heard this heartbeat for this long */
   private static final double HEARTBEAT_LEAD_TIME = 1.0;
   private static final double WAYPOINT_SPACING = 1.0;
   private static final double SKIP_WAYPOINTS_WITHIN = 0.5;
   private static final double FINAL_WAYPOINT_PROXIMITY = 0.4;
   private static final double WAYPOINT_PROXIMITY = 0.5;
   /**
    * The planner republishes its path every second, and Hydra's frontiers drift between updates; only a goal
    * that moved this far is sent again, so the robot isn't restarted on every update
    */
   private static final double NEW_GOAL_DISTANCE = 1.0;
   /** The enable command is repeated, since one message can be lost before the planner is discovered */
   private static final double ENABLE_REPEAT_FREQUENCY = 1.0;
   private static final double ENABLE_REPEAT_AFTER_STOP = 5.0;

   private final List<ROS2Subscription<Path>> pathSubscriptions = new ArrayList<>();
   private final List<ROS2Publisher<Bool>> enablePublishers = new ArrayList<>();
   private final Bool enableMessage = new Bool();
   private final Throttler enableThrottler = new Throttler().setFrequency(ENABLE_REPEAT_FREQUENCY);
   private final ROS2Heartbeat stepGeneratorHeartbeat;
   private final ROS2Heartbeat yoloDemand;
   private final NonWallTimer heartbeatTimer = new NonWallTimer();
   private final NonWallTimer stoppedTimer = new NonWallTimer();

   private final Object pathLock = new Object();
   private final List<Point3D> latestPath = new ArrayList<>();
   private boolean pathReceived = false;
   private String pathPlanner = null;

   private final List<Point3D> traversedDoors = new ArrayList<>();
   /** Doors DoorTraversal failed on, ignored for the rest of the run */
   private final List<Point3D> skippedDoors = new ArrayList<>();
   private Point3D doorHandedOver = null;
   /** Where the robot was when it handed the door over, to tell whether it got through */
   private final Point3D handedOverFrom = new Point3D();
   /** The door whose traversal failed, while the tree waits for the operator */
   private Point3D failedDoor = null;
   private final FramePose3D midFeetPose = new FramePose3D();
   private boolean exploring = false;
   private boolean walking = false;
   private Point3D sentGoal = null;
   private boolean waitingForPlannerLogged = false;

   // Status for the Exploration panel; times are in the robot's (simulation) time
   private final ROS2Publisher<String_> statusPublisher;
   private final String_ statusMessage = new String_();
   private final Throttler statusThrottler = new Throttler().setFrequency(1.0);
   private double now = Double.NaN;
   private String lastGoalSent = null;
   private double lastGoalTime = Double.NaN;
   private String lastStop = null;
   private double lastStopTime = Double.NaN;
   private int doorsHandedOver = 0;
   private String nearestDoorDescription = "";

   public BuildingExplorationExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new BuildingExplorationState(id, rootNode.getState()), rootNode);

      ROS2QoSProfile latched = new ROS2QoSProfile();
      latched.reliability(ROS2QoSProfile.Reliability.RELIABLE);
      latched.durability(ROS2QoSProfile.Durability.TRANSIENT_LOCAL);
      latched.depth(1);
      for (String namespace : PLANNER_NAMESPACES)
      {
         ROS2Topic<Path> pathTopic = new ROS2Topic<>(namespace + "/path", Path.class);
         pathSubscriptions.add(ros2ControllerHelper.getROS2Node().createSubscriptionSampler(pathTopic, path -> onPath(namespace, path), latched));
         enablePublishers.add(ros2ControllerHelper.getROS2Node().createPublisher(new ROS2Topic<>(namespace + "/enable_command", Bool.class)));
      }
      statusPublisher = ros2ControllerHelper.getROS2Node().createPublisher(new ROS2Topic<>(STATUS_TOPIC, String_.class), latched);
      stepGeneratorHeartbeat = new ROS2Heartbeat(ros2ControllerHelper.getROS2Node(), CSGROS2CommunicationHelper.CSG_HEARTBEAT_TOPIC);
      yoloDemand = new ROS2Heartbeat(ros2ControllerHelper.getROS2Node(), PerceptionAPI.REQUEST_YOLO);
   }

   private void onPath(String planner, Path path)
   {
      synchronized (pathLock)
      {
         pathPlanner = planner;
         latestPath.clear();
         for (PoseStamped pose : path.getPoses())
            latestPath.add(new Point3D(pose.getPose().getPosition().getX(), pose.getPose().getPosition().getY(), pose.getPose().getPosition().getZ()));
         pathReceived = true;
      }
   }

   @Override
   public void update()
   {
      super.update();

      now = Conversions.nanosecondsToSeconds(syncedRobot.getTimestamp());
      if (statusThrottler.run())
         publishStatus();
      heartbeatTimer.update(now);
      stoppedTimer.update(now);
      midFeetPose.setFromReferenceFrame(syncedRobot.getReferenceFrames().getMidFeetUnderPelvisFrame());
      traversedDoors.removeIf(door -> door.distanceXY(midFeetPose.getPosition()) > TRAVERSED_DOOR_REARM_DISTANCE);

      if (!(searchDFSFirstMatch(this, EXPLORE_NODE_NAME) instanceof WaitActionExecutor exploreWait))
      {
         if (exploring)
            stopExploring("there is no \"%s\" node".formatted(EXPLORE_NODE_NAME));
         return;
      }

      boolean waitExecuting = exploreWait.getState().getIsExecuting();
      if (waitExecuting)
      {
         if (failedDoor != null)
            resumeAfterDoorFailure();
         doorHandedOver = null;
      }
      else if (doorHandedOver != null && failedDoor == null && !rootNode.getState().getAutomaticExecution())
      {
         List<String> failedSteps = failedDoorSteps();
         if (!failedSteps.isEmpty())
         {
            // Leave the tree stopped at the failed step, as the door behavior does on its own
            failedDoor = doorHandedOver;
            state.getLogger().warn("Door traversal failed at %s. Stopped there: play the tree to retry the door, or press Start exploration to skip it."
                                         .formatted(String.join(", ", failedSteps)));
         }
      }

      if (waitExecuting && !rootNode.getState().getAutomaticExecution())
      {
         // Paused: end the wait and point the tree back at it, so play resumes exploring
         exploreWait.getState().setIsExecuting(false);
         rootNode.getState().setExecutionNextIndex(exploreWait.getState().getLeafIndex());
         stopExploring("the operator paused automatic execution");
         return;
      }

      if (waitExecuting && !exploring)
         startExploring();
      else if (!waitExecuting && exploring)
         stopExploring("\"%s\" ended".formatted(EXPLORE_NODE_NAME));

      publishPlannerEnable();
      if (!exploring)
         return;

      Point3D door = findDoorAhead();
      if (door != null)
      {
         state.getLogger().info("Door ahead at (%.2f, %.2f), %.1f m away. Handing over to door traversal."
                                      .formatted(door.getX(), door.getY(), door.distanceXY(midFeetPose.getPosition())));
         doorsHandedOver++;
         traversedDoors.add(door);
         doorHandedOver = door;
         handedOverFrom.set(midFeetPose.getPosition());
         stopExploring("%s ahead, handed over".formatted(nearestDoorDescription));
         // The tree already points past the wait, so it continues into DoorTraversal
         exploreWait.getState().setIsExecuting(false);
         return;
      }

      followPlannerPath();
   }

   /** The names of the failed leaves under the DoorTraversal child; a failure stops automatic execution. */
   private List<String> failedDoorSteps()
   {
      List<String> failed = new ArrayList<>();
      for (BehaviorTreeNodeExecutor<?, ?> child : getChildren())
         if (child instanceof DoorTraversalExecutor doorTraversal)
            BehaviorTreeTools.runForSubtreeNodes(doorTraversal, node ->
            {
               if (node instanceof LeafNodeExecutor<?, ?> leaf && leaf.getState().getFailed())
                  failed.add("\"" + leaf.getDefinition().getName() + "\"");
            });
      return failed;
   }

   /**
    * Exploring started again after a door step failed. If the operator got the robot through the door,
    * it counts as traversed; if the robot is still in front of it, the door is skipped for the rest of
    * the run, so it doesn't stop exploring again.
    */
   private void resumeAfterDoorFailure()
   {
      double towardDoorX = failedDoor.getX() - handedOverFrom.getX();
      double towardDoorY = failedDoor.getY() - handedOverFrom.getY();
      double length = Math.max(1.0e-6, Math.hypot(towardDoorX, towardDoorY));
      double pastDoor = ((midFeetPose.getX() - failedDoor.getX()) * towardDoorX + (midFeetPose.getY() - failedDoor.getY()) * towardDoorY) / length;
      if (pastDoor > THROUGH_DOOR_DISTANCE)
      {
         state.getLogger().info("Through the door at (%.2f, %.2f) after a failed step. Exploring on.".formatted(failedDoor.getX(), failedDoor.getY()));
      }
      else
      {
         skippedDoors.add(failedDoor);
         state.getLogger().warn("Exploring on in front of the door at (%.2f, %.2f) that failed. Skipping it for the rest of the run."
                                      .formatted(failedDoor.getX(), failedDoor.getY()));
      }
      failedDoor = null;
   }

   private void startExploring()
   {
      exploring = true;
      walking = false;
      sentGoal = null;
      waitingForPlannerLogged = false;
      stepGeneratorHeartbeat.setAlive(true);
      heartbeatTimer.reset();
      yoloDemand.setAlive(true);
      state.getLogger().info("Exploring: enabled the planner on %s".formatted(String.join(" and ", PLANNER_NAMESPACES)));
      sendPlannerEnable();
   }

   private void stopExploring(String reason)
   {
      exploring = false;
      stopWalking();
      stepGeneratorHeartbeat.setAlive(false);
      yoloDemand.setAlive(false);
      stoppedTimer.reset();
      sendPlannerEnable();
      state.getLogger().info("Stopped exploring: %s".formatted(reason));
      lastStop = reason;
      lastStopTime = now;
   }

   private void publishPlannerEnable()
   {
      boolean repeat = exploring || stoppedTimer.isRunning(ENABLE_REPEAT_AFTER_STOP);
      if (repeat && enableThrottler.run())
         sendPlannerEnable();
   }

   private void sendPlannerEnable()
   {
      enableMessage.setData(exploring);
      for (ROS2Publisher<Bool> enablePublisher : enablePublishers)
         enablePublisher.publish(enableMessage);
   }

   private void followPlannerPath()
   {
      List<Point3D> path;
      synchronized (pathLock)
      {
         if (!pathReceived)
         {
            if (!waitingForPlannerLogged)
               state.getLogger().info("Waiting for a path from %s".formatted(String.join(" or ", PLANNER_NAMESPACES)));
            waitingForPlannerLogged = true;
            return;
         }
         path = new ArrayList<>(latestPath);
      }

      if (path.isEmpty())
      {
         if (walking)
            state.getLogger().info("The planner has no goal, stopping");
         stopWalking();
         return;
      }

      Point3D goal = path.get(path.size() - 1);
      if (sentGoal != null && goal.distanceXY(sentGoal) < NEW_GOAL_DISTANCE)
         return;
      if (!heartbeatTimer.isExpired(HEARTBEAT_LEAD_TIME))
         return;

      ControllerWaypointGoalListMessage goalList = new ControllerWaypointGoalListMessage();
      double previousX = midFeetPose.getX();
      double previousY = midFeetPose.getY();
      for (int i = 0; i < path.size(); i++)
      {
         Point3D point = path.get(i);
         boolean last = i == path.size() - 1;
         double fromPrevious = Math.hypot(point.getX() - previousX, point.getY() - previousY);
         // Thin the path to about a waypoint a meter, skipping what the robot is already on
         if (!last && (point.distanceXY(midFeetPose.getPosition()) < SKIP_WAYPOINTS_WITHIN || fromPrevious < WAYPOINT_SPACING))
            continue;

         ControllerWaypointGoalMessage waypoint = goalList.getWaypoints().add();
         waypoint.setXPosition(point.getX());
         waypoint.setYPosition(point.getY());
         waypoint.setYaw(fromPrevious > 1.0e-3 ? Math.atan2(point.getY() - previousY, point.getX() - previousX) : midFeetPose.getYaw());
         waypoint.setPositionProximity(last ? FINAL_WAYPOINT_PROXIMITY : WAYPOINT_PROXIMITY);
         waypoint.setOrientationProximity(Math.toRadians(30.0));
         waypoint.setGoalOrientationMatters(false);
         waypoint.setKeepHeading(false);
         waypoint.setTurnWalkTurn(false);
         previousX = point.getX();
         previousY = point.getY();
      }

      String planner;
      synchronized (pathLock)
      {
         planner = pathPlanner;
      }
      state.getLogger().info("Walking to %s's goal at (%.2f, %.2f) through %d waypoints".formatted(planner, goal.getX(), goal.getY(), goalList.getWaypoints().size()));
      ros2ControllerHelper.publishToController(goalList);
      lastGoalSent = "(%.2f, %.2f), %d waypoints".formatted(goal.getX(), goal.getY(), goalList.getWaypoints().size());
      lastGoalTime = now;
      sentGoal = goal;
      walking = true;
   }

   /**
    * The goal list leaves the step generator walking; aborting clears it. Sent while the heartbeat is
    * still alive, or the step generator drops it.
    */
   private void stopWalking()
   {
      if (walking)
         ros2ControllerHelper.publishToController(new AbortWalkingMessage());
      walking = false;
      sentGoal = null;
   }

   /**
    * The nearest door YOLO has seen for a while, in front of the robot, close enough to traverse, and
    * near the planner's path when there is one.
    */
   private Point3D findDoorAhead()
   {
      List<Point3D> path;
      synchronized (pathLock)
      {
         path = new ArrayList<>(latestPath);
      }

      Point3D nearest = null;
      double nearestDistance = Double.MAX_VALUE;
      for (PersistentDetection detection : scene.getPersistentDetections())
      {
         if (!(detection.getMostRecentDetection() instanceof YOLOv8InstantDetection yoloDetection)
             || !DOOR_CLASSES.contains(yoloDetection.getDetectedObjectClass())
             || detection.getHistorySize() < DOOR_MINIMUM_HISTORY)
            continue;

         Point3D position = new Point3D(detection.getFilteredTransform().getTranslation());
         double dx = position.getX() - midFeetPose.getX();
         double dy = position.getY() - midFeetPose.getY();
         double distance = Math.hypot(dx, dy);
         double bearing = AngleTools.computeAngleDifferenceMinusPiToPi(Math.atan2(dy, dx), midFeetPose.getYaw());
         if (distance > DOOR_AHEAD_DISTANCE || Math.abs(bearing) > DOOR_AHEAD_HALF_ANGLE)
            continue;
         if (traversedDoors.stream().anyMatch(door -> door.distanceXY(position) < TRAVERSED_DOOR_RADIUS)
             || skippedDoors.stream().anyMatch(door -> door.distanceXY(position) < TRAVERSED_DOOR_RADIUS))
            continue;
         if (path.size() >= 2 && distanceToPathXY(position, path) > DOOR_PATH_DISTANCE)
            continue;
         if (distance < nearestDistance)
         {
            nearestDistance = distance;
            nearest = position;
            nearestDoorDescription = "%s (%.0f%%) at (%.2f, %.2f), %.1f m away".formatted(yoloDetection.getDetectedObjectClass(),
                                                                                       100.0 * yoloDetection.getConfidence(),
                                                                                       position.getX(),
                                                                                       position.getY(),
                                                                                       distance);
         }
      }
      return nearest;
   }

   private static double distanceToPathXY(Point3D point, List<Point3D> path)
   {
      double closest = Double.MAX_VALUE;
      for (int i = 1; i < path.size(); i++)
      {
         Point3D a = path.get(i - 1);
         Point3D b = path.get(i);
         double dx = b.getX() - a.getX();
         double dy = b.getY() - a.getY();
         double lengthSquared = dx * dx + dy * dy;
         double t = lengthSquared < 1.0e-9 ? 0.0
               : Math.max(0.0, Math.min(1.0, ((point.getX() - a.getX()) * dx + (point.getY() - a.getY()) * dy) / lengthSquared));
         closest = Math.min(closest, Math.hypot(point.getX() - (a.getX() + t * dx), point.getY() - (a.getY() + t * dy)));
      }
      return closest;
   }

   /**
    * One line for the Exploration panel: what this node is doing, the last walking goal it sent and
    * the last reason it stopped exploring, with their age in robot time.
    */
   private void publishStatus()
   {
      String activity;
      if (exploring)
      {
         if (walking && sentGoal != null)
            activity = "walking to (%.2f, %.2f), %.1f m away".formatted(sentGoal.getX(), sentGoal.getY(), sentGoal.distanceXY(midFeetPose.getPosition()));
         else
            activity = "exploring, no walking goal sent";
      }
      else if (failedDoor != null)
         activity = "stopped on a failed door step";
      else if (doorHandedOver != null)
         activity = "door behavior running";
      else
         activity = "not exploring";
      String goal = lastGoalSent == null ? "none yet" : "%s, %.0f s ago".formatted(lastGoalSent, now - lastGoalTime);
      String stop = lastStop == null ? "none yet" : "%s, %.0f s ago".formatted(lastStop, now - lastStopTime);
      statusMessage.setData("%s | last goal sent: %s | last stop: %s | doors handed over: %d".formatted(activity, goal, stop, doorsHandedOver));
      statusPublisher.publish(statusMessage);
   }

   @Override
   public void destroy()
   {
      if (exploring)
         stopExploring("the tree was destroyed");
      for (ROS2Subscription<Path> pathSubscription : pathSubscriptions)
         ros2ControllerHelper.getROS2Node().destroySubscription(pathSubscription);
      for (ROS2Publisher<Bool> enablePublisher : enablePublishers)
         ros2ControllerHelper.getROS2Node().destroyPublisher(enablePublisher);
      ros2ControllerHelper.getROS2Node().destroyPublisher(statusPublisher);
      stepGeneratorHeartbeat.destroy();
      yoloDemand.destroy();
   }
}
