package us.ihmc.humanoidBehaviors.patrol;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.Pose3D;
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
import java.util.concurrent.atomic.AtomicReference;

/**
 * Walk through a list of waypoints in order, looping, forever.
 */
public class PatrolBehavior
{
   private final IHMCROS2Publisher<FootstepDataListMessage> footstepDataListPublisher;
   private final RemoteSyncedHumanoidFrames remoteSyncedHumanoidFrames;
   private final RemoteFootstepPlannerInterface remoteFootstepPlannerInterface;

   private final Notification newWaypointsAvailable = new Notification();
   private final AtomicReference<Integer> nextWaypointIndex;
   private final AtomicReference<ArrayList<Pose3D>> waypoints;

   private final Activator
   private final int currentGoalWaypointIndex = 0; // Update this
   private final Pose3D currentGoalWaypoint = new Pose3D(); // TODO evaluate if this is a bug

   public PatrolBehavior(Messager messager, Ros2Node ros2Node, DRCRobotModel robotModel)
   {
      LogTools.debug("Initializing patrol behavior");

      footstepDataListPublisher = ROS2Tools.createPublisher(ros2Node,
                                                            ROS2Tools.newMessageInstance(FootstepDataListCommand.class).getMessageClass(),
                                                            ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotModel.getSimpleRobotName()));

      remoteSyncedHumanoidFrames = new RemoteSyncedHumanoidFrames(robotModel, ros2Node);

      remoteFootstepPlannerInterface = new RemoteFootstepPlannerInterface(ros2Node, robotModel);

      nextWaypointIndex = messager.createInput(API.GoToWaypoint, 0);

      waypoints = messager.createInput(API.Waypoints);
      messager.registerTopicListener(API.Waypoints, waypoints -> newWaypointsAvailable.set());

      PeriodicNonRealtimeThreadScheduler patrolThread = new PeriodicNonRealtimeThreadScheduler(getClass().getSimpleName());
      patrolThread.schedule(this::patrolThread, 250, TimeUnit.MILLISECONDS);

      PeriodicNonRealtimeThreadScheduler waypointUpdateThread = new PeriodicNonRealtimeThreadScheduler(getClass().getSimpleName());
      waypointUpdateThread.schedule(this::waypointUpdateThread, 250, TimeUnit.MILLISECONDS);
   }

   private void patrolThread()
   {

      // if not stopped

      // while not interrupted by GoToWaypoint or stopped
         // walk to next waypoint

      // planNext
   }

   private void waypointUpdateThread()
   {
      if (newWaypointsAvailable.poll())
      {
         // if update

         // if new set
      }
   }

   private void planNext()
   {


      remoteFootstepPlannerInterface.requestPlanBlocking()

   }

   private void planRest()
   {

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

      /** When received, the robot stops walking and waits forever. */
      public static final Topic<Object> Stop = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("Stop"));

      /** Output to visualize the current robot path plan. */
      public static final Topic<WaypointFootstepPlan> FootstepPlan = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("FootstepPlan"));

      public static final MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }
}
