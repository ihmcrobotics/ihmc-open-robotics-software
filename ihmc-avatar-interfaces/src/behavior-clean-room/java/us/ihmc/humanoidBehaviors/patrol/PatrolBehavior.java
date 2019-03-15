package us.ihmc.humanoidBehaviors.patrol;

import controller_msgs.msg.dds.FootstepDataListMessage;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.humanoidBehaviors.tools.Activator;
import us.ihmc.humanoidBehaviors.tools.RemoteFootstepPlannerInterface;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedHumanoidFrames;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Walk through a list of waypoints in order, looping forever.
 */
public class PatrolBehavior
{
   /** Wait to gather perception data, also serves as stop state as it has no action */
   private final Pair<String, Runnable> STOP_AND_PERCEIVE = Pair.of("STOP_AND_PERCEIVE", this::updateStopAndPerceiveState);
   /** Request and wait for footstep planner result */
   private final Pair<String, Runnable> RETRIEVE_FOOTSTEP_PLAN = Pair.of("RETRIEVE_FOOTSTEP_PLAN", this::updateRetrieveFootstepPlanState);
   /** Walking towards goal waypoint */
   private final Pair<String, Runnable> WALK_TO_GOAL_WAYPOINT = Pair.of("WALK_TO_GOAL_WAYPOINT", this::updateWalkToGoalWaypointState);

   public static final double DEFAULT_PERCEPTION_GATHERING_DURATION = 0.0; // TODO Maybe this is not needed if replanning until valid planner result

   private Pair<String, Runnable> currentState;

   private final Messager messager;
   private final IHMCROS2Publisher<FootstepDataListMessage> footstepDataListPublisher;
   private final RemoteSyncedHumanoidFrames remoteSyncedHumanoidFrames;
   private final RemoteFootstepPlannerInterface remoteFootstepPlannerInterface;

   private final Notification stopNotification = new Notification();

   private final AtomicReference<Double> perceptionGatheringDuration;
   private final Stopwatch gatherPerceptionDataStopwatch = new Stopwatch();


   private final AtomicInteger goalWaypointIndex = new AtomicInteger();

   private final AtomicReference<ArrayList<Pose3D>> waypoints;

   private final Activator walking = new Activator();
   private final int currentGoalWaypointIndex = 0; // Update this
   private final Pose3D currentGoalWaypoint = new Pose3D(); // TODO evaluate if this is a bug

   public PatrolBehavior(Messager messager, Ros2Node ros2Node, DRCRobotModel robotModel)
   {
      this.messager = messager;

      LogTools.debug("Initializing patrol behavior");

      footstepDataListPublisher = ROS2Tools.createPublisher(ros2Node,
                                                            ROS2Tools.newMessageInstance(FootstepDataListCommand.class).getMessageClass(),
                                                            ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotModel.getSimpleRobotName()));

      remoteSyncedHumanoidFrames = new RemoteSyncedHumanoidFrames(robotModel, ros2Node);

      remoteFootstepPlannerInterface = new RemoteFootstepPlannerInterface(ros2Node, robotModel);

      perceptionGatheringDuration = messager.createInput(API.PerceptionGatheringDuration, DEFAULT_PERCEPTION_GATHERING_DURATION);
      messager.submitMessage(API.PerceptionGatheringDuration, DEFAULT_PERCEPTION_GATHERING_DURATION);

      messager.registerTopicListener(API.Stop, object -> stopNotification.set());
      messager.registerTopicListener(API.GoToWaypoint, goToWaypointIndex ->
      {
         goalWaypointIndex.set(goToWaypointIndex);
         stopNotification.set();
      });

      waypoints = messager.createInput(API.Waypoints);

      gatherPerceptionDataStopwatch.start();
      transitionTo(STOP_AND_PERCEIVE);

      PeriodicNonRealtimeThreadScheduler patrolThread = new PeriodicNonRealtimeThreadScheduler(getClass().getSimpleName());
      patrolThread.schedule(this::patrolThread, 1, TimeUnit.MILLISECONDS);
   }

   private void patrolThread()
   {
      currentState.getValue().run();
   }

   private void updateStopAndPerceiveState()
   {
      if (checkStopped()) return;

      if (gatherPerceptionDataStopwatch.lapElapsed() >= perceptionGatheringDuration.get())
      {
         transitionTo(RETRIEVE_FOOTSTEP_PLAN);
      }
   }

   private void updateRetrieveFootstepPlanState()
   {
      if (checkStopped()) return;

      // else if footstep plan not yet sent
       // if got a footstep plan

      if ()
      {
         ArrayList<Pose3D> latestWaypoints = waypoints.get();
         if (goToWaypoint.get() >= 0 && !latestWaypoints.isEmpty())
         {

         }

         ArrayList<Pose3D> waypoints = this.waypoints.accumulateAndGet() get();

         remoteFootstepPlannerInterface.requestPlanBlocking()
      }
   }

   private void updateWalkToGoalWaypointState()
   {
      if (checkStopped()) return;
      // if walking is done
      {
         //
      }
   }

   private boolean checkStopped()
   {
      if (stopNotification.poll())
      {
         gatherPerceptionDataStopwatch.reset();
         transitionTo(STOP_AND_PERCEIVE);
      }

      return stopNotification.read();
   }

   private void transitionTo(Pair<String, Runnable> stateToTransitionTo)
   {
      currentState = stateToTransitionTo;
      messager.submitMessage(API.CurrentState, currentState.getKey());
   }

   public static class API
   {
      private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
      private static final Category Root = apiFactory.createRootCategory("Behavior");
      private static final CategoryTheme Patrol = apiFactory.createCategoryTheme("Patrol");

      /** Input: Update the waypoints */
      public static final Topic<ArrayList<Pose3D>> Waypoints = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("Waypoints"));

      /** Input: Robot stops and immediately goes to this waypoint. The "start" or "reset" command.  */
      public static final Topic<Integer> GoToWaypoint = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("GoToWaypoint"));

      /** Input: When received, the robot stops walking and waits forever. */
      public static final Topic<Object> Stop = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("Stop"));

      /** Input: Amount of time in seconds to gather perception data before sending out for a footstep plan. */
      public static final Topic<Double> PerceptionGatheringDuration =
            Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("PerceptionGatheringDuration"));

      /** Output: to visualize the current robot path plan. */
      public static final Topic<WaypointFootstepPlan> FootstepPlan = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("FootstepPlan"));

      /** Output: to visualize the current state. */
      public static final Topic<String> CurrentState = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("CurrentState"));

      public static final MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }
}
