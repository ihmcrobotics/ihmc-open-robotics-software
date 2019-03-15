package us.ihmc.humanoidBehaviors.patrol;

import controller_msgs.msg.dds.FootstepDataListMessage;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidBehaviors.tools.Activator;
import us.ihmc.humanoidBehaviors.tools.RemoteFootstepPlannerInterface;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedHumanoidFrames;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
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
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Walk through a list of waypoints in order, looping forever.
 */
public class PatrolBehavior
{
   /** Stop state that waits for or is triggered by a GoToWaypoint message */
   private final Pair<String, Runnable> STOP = Pair.of("STOP", this::updateStopState);
   /** Request and wait for footstep planner result */
   private final Pair<String, Runnable> RETRIEVE_FOOTSTEP_PLAN = Pair.of("RETRIEVE_FOOTSTEP_PLAN", this::updateRetrieveFootstepPlanState);
   /** Walking towards goal waypoint */
   private final Pair<String, Runnable> WALK_TO_GOAL_WAYPOINT = Pair.of("WALK_TO_GOAL_WAYPOINT", this::updateWalkToGoalWaypointState);

   private Pair<String, Runnable> currentState;

   private final Messager messager;
   private final IHMCROS2Publisher<FootstepDataListMessage> footstepDataListPublisher;
   private final RemoteSyncedHumanoidFrames remoteSyncedHumanoidFrames;
   private final RemoteFootstepPlannerInterface remoteFootstepPlannerInterface;

   private final Notification stopNotification = new Notification();
   private final Notification goToWaypointNotification = new Notification();

   private final AtomicInteger goalWaypointIndex = new AtomicInteger();


   private final Notification readyToPlanNotification = new Notification();
   private final AtomicBoolean isPlanning = new AtomicBoolean();
   private final AtomicReference<ArrayList<Pose3D>> waypoints;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final Activator walking = new Activator();
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

      messager.registerTopicListener(API.Stop, object -> stopNotification.set());
      messager.registerTopicListener(API.GoToWaypoint, goToWaypointIndex ->
      {
         goalWaypointIndex.set(goToWaypointIndex);
         goToWaypointNotification.set();
      });

      waypoints = messager.createInput(API.Waypoints);

      transitionTo(STOP);

      PeriodicNonRealtimeThreadScheduler patrolThread = new PeriodicNonRealtimeThreadScheduler(getClass().getSimpleName());
      patrolThread.schedule(this::patrolThread, 1, TimeUnit.MILLISECONDS);
   }

   private void patrolThread()
   {
      currentState.getValue().run();
   }

   private void updateStopState()
   {
      checkForInterruptions();
   }

   private void updateRetrieveFootstepPlanState()
   {
      if (checkForInterruptions()) return;

      if (readyToPlanNotification.poll())
      {
         MovingReferenceFrame midFeetZUpFrame = remoteSyncedHumanoidFrames.pollHumanoidReferenceFrames().getMidFeetZUpFrame();
         FramePose3D midFeetZUpPose = new FramePose3D(midFeetZUpFrame);
         midFeetZUpPose.changeFrame(worldFrame);

         FramePose3D currentGoalWaypoint = new FramePose3D(waypoints.get().get(goalWaypointIndex.get()));

         new Thread(() -> {
            remoteFootstepPlannerInterface.requestPlanBlocking(midFeetZUpPose, currentGoalWaypoint);
            isPlanning.set(false);
         }).start();
      }
      else if (!isPlanning.get())
      {

      }
   }

   private void updateWalkToGoalWaypointState()
   {
      if (checkForInterruptions()) return;
      // if walking is done
      {
         //
      }
   }

   private boolean checkForInterruptions()
   {
      stopNotification.poll();
      goToWaypointNotification.poll();

      if (stopNotification.read())  // favor stop if race condition
      {
         sendStopWalking();
         transitionTo(STOP);
      }

      if (goToWaypointNotification.read())
      {
         sendStopWalking();

         ArrayList<Pose3D> latestWaypoints = waypoints.get();
         int currentGoalWaypointIndex = goalWaypointIndex.get();
         if (currentGoalWaypointIndex >= 0 && currentGoalWaypointIndex < latestWaypoints.size())
         {
            readyToPlanNotification.set();
            transitionTo(RETRIEVE_FOOTSTEP_PLAN);
         }
         else
         {
            transitionTo(STOP);
         }
      }

      return stopNotification.read() || goToWaypointNotification.read();
   }

   private void sendStopWalking()
   {
      footstepDataListPublisher.publish(new FootstepDataListMessage()); // TODO I guess this works?
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
