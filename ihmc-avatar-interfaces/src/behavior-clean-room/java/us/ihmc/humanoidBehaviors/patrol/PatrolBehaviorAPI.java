package us.ihmc.humanoidBehaviors.patrol;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.humanoidBehaviors.patrol.PatrolBehavior.OperatorPlanReviewResult;
import us.ihmc.humanoidBehaviors.patrol.PatrolBehavior.PatrolBehaviorState;
import us.ihmc.humanoidBehaviors.tools.TunedFootstepPlannerParameters;
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

   /** Input: Waypoints modified by operator */
   public static final Topic<WaypointSequence> WaypointsToModule = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("WaypointsToModule"));

   /** Output: Waypoints modified by behavior */
   public static final Topic<WaypointSequence> WaypointsToUI = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("WaypointsToUI"));

   /** Input: Robot stops and immediately goes to this waypoint. The "start" or "reset" command.  */
   public static final Topic<Integer> GoToWaypoint = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("GoToWaypoint"));

   /** Output: to visualize the current waypoint status. TODO clean me up */
   public static final Topic<Integer> CurrentWaypointIndexStatus
         = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("CurrentWaypointIndexStatus"));

   /** Input: When received, the robot stops walking and waits forever. */
   public static final Topic<Object> Stop = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("Stop"));

   /** Input: Cancel planning for things like live tuning. */
   public static final Topic<Object> CancelPlanning = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("CancelPlanning"));

   /** Input: Toggle looping through waypoints. */
   public static final Topic<Boolean> Loop = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("Loop"));

   /** Input: Toggle swinging over planar regions. */
   public static final Topic<Boolean> SwingOvers = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("SwingOvers"));

   /** Input: Enable/disable human plan review before walking. */
   public static final Topic<Boolean> PlanReviewEnabled = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("PlanReviewEnabled"));

   /** Input: Enable/disable human plan review before walking. */
   public static final Topic<Boolean> UpDownExplorationEnabled
         = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("UpDownExplorationEnabled"));

   /** Input: Set the turn amount to find up or down. */
   public static final Topic<Double> ExplorationTurnAmount
         = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("ExplorationTurnAmount"));

   /** Input: Enable/disable human plan review before walking. */
   public static final Topic<OperatorPlanReviewResult> PlanReviewResult
         = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("PlanReviewResult"));

   /** Input: For the UI to set the parameters published before walking. */
   public static final Topic<TunedFootstepPlannerParameters> PlannerParameters
         = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("PlannerParameters"));

   /** Output: to visualize the current robot path plan. */
   public static final Topic<ArrayList<Pair<RobotSide, Pose3D>>> CurrentFootstepPlan
         = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("CurrentFootstepPlan"));

   /** Output: to visualize the current state. */
   public static final Topic<PatrolBehaviorState> CurrentState = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("CurrentState"));

   public static final MessagerAPI create()
   {
      return apiFactory.getAPIAndCloseFactory();
   }
}
