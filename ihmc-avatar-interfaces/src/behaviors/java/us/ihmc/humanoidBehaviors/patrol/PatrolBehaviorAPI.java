package us.ihmc.humanoidBehaviors.patrol;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.patrol.PatrolBehavior.OperatorPlanReviewResult;
import us.ihmc.humanoidBehaviors.patrol.PatrolBehavior.PatrolBehaviorState;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.TunedFootstepPlannerParameters;
import us.ihmc.humanoidBehaviors.upDownExploration.UpDownResult;
import us.ihmc.humanoidBehaviors.waypoints.WaypointSequence;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;

public class PatrolBehaviorAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final Category Root = apiFactory.createRootCategory("PatrolBehavior");
   private static final CategoryTheme Patrol = apiFactory.createCategoryTheme("Patrol");

   /** Input: Waypoints modified by operator TODO These two require a new feature from messager: remote-receive only */
   public static final Topic<WaypointSequence> WaypointsToModule = topic("WaypointsToModule");
   /** Output: Waypoints modified by behavior */
   public static final Topic<WaypointSequence> WaypointsToUI = topic("WaypointsToUI");
   /** Input: Robot stops and immediately goes to this waypoint. The "start" or "reset" command. */
   public static final Topic<Integer> GoToWaypoint = topic("GoToWaypoint");
   /** Output: to visualize the current waypoint status. TODO clean me up */
   public static final Topic<Integer> CurrentWaypointIndexStatus = topic("CurrentWaypointIndexStatus");
   /** Input: When received, the robot stops walking and waits forever. */
   public static final Topic<Object> Stop = topic("Stop");
   /** Input: Cancel planning for things like live tuning. */
   public static final Topic<Object> CancelPlanning = topic("CancelPlanning");
   /** Input: Option to skip perceive state. */
   public static final Topic<Object> SkipPerceive = topic("SkipPerceive");
   /** Input: Toggle looping through waypoints. */
   public static final Topic<Boolean> Loop = topic("Loop");
   /** Input: Toggle swinging over planar regions. */
   public static final Topic<Boolean> SwingOvers = topic("SwingOvers");
   /** Input: Enable/disable human plan review before walking. */
   public static final Topic<Boolean> PlanReviewEnabled = topic("PlanReviewEnabled");
   /** Input: Enable/disable human plan review before walking. */
   public static final Topic<Boolean> UpDownExplorationEnabled = topic("UpDownExplorationEnabled");
   /** Input: Set the turn amount to find up or down. */
   public static final Topic<Double> PerceiveDuration = topic("PerceiveDuration");
   /** Input: Enable/disable human plan review before walking. */
   public static final Topic<OperatorPlanReviewResult> PlanReviewResult = topic("PlanReviewResult");
   /** Input: For the UI to set the parameters published before walking. */
   public static final Topic<TunedFootstepPlannerParameters> PlannerParameters = topic("PlannerParameters");
   /** Output: to visualize the current robot path plan. */
   public static final Topic<ArrayList<Pair<RobotSide, Pose3D>>> CurrentFootstepPlan = topic("CurrentFootstepPlan");
   /** Output: to visualize the current state. */
   public static final Topic<PatrolBehaviorState> CurrentState = topic("CurrentState");
   /** Output: to visualize up-down goal poses. */
   public static final Topic<UpDownResult> UpDownGoalPoses = topic("UpDownGoalPoses");
   /** Output: to visualize up-down goal poses. */
   public static final Topic<Point3D> UpDownCenter = topic("UpDownCenter");

   private static final <T> Topic<T> topic(String name)
   {
      return Root.child(Patrol).topic(apiFactory.createTypedTopicTheme(name));
   }

   public static final MessagerAPI create()
   {
      return apiFactory.getAPIAndCloseFactory();
   }
}
