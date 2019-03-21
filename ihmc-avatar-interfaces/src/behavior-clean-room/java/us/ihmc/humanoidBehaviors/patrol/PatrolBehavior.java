package us.ihmc.humanoidBehaviors.patrol;

import controller_msgs.msg.dds.*;
import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.humanoidBehaviors.tools.RemoteFootstepPlannerInterface;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedHumanoidFrames;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PauseWalkingCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotics.robotSide.RobotSide;
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
   /** Stop state that waits for or is triggered by a GoToWaypoint message */
   private final Pair<String, Runnable> STOP = Pair.of("STOP", this::updateStopState);
   /** Request and wait for footstep planner result */
   private final Pair<String, Runnable> RETRIEVE_FOOTSTEP_PLAN = Pair.of("RETRIEVE_FOOTSTEP_PLAN", this::updateRetrieveFootstepPlanState);
   /** Walking towards goal waypoint */
   private final Pair<String, Runnable> WALK_TO_GOAL_WAYPOINT = Pair.of("WALK_TO_GOAL_WAYPOINT", this::updateWalkToGoalWaypointState);

   private Pair<String, Runnable> currentState;

   private final Messager messager;
   private final IHMCROS2Publisher<FootstepDataListMessage> footstepDataListPublisher;
   private final IHMCROS2Publisher<PauseWalkingMessage> pausePublisher;
   private final RemoteSyncedHumanoidFrames remoteSyncedHumanoidFrames;
   private final RemoteFootstepPlannerInterface remoteFootstepPlannerInterface;

   private final Notification stopNotification = new Notification();
   private final Notification overrideGoToWaypointNotification = new Notification();

   private final AtomicInteger goalWaypointIndex = new AtomicInteger();

   private final Notification readyToPlanNotification = new Notification();
   private final Notification footstepPlanCompleted = new Notification();
   private final Notification walkingCompleted = new Notification();

   private FootstepPlanningToolboxOutputStatus footstepPlanningOutput;

   private final AtomicReference<ArrayList<Pose3D>> waypoints;

   private final AtomicReference<PlanarRegionsListMessage> latestPlanarRegions = new AtomicReference<>();

   public PatrolBehavior(Messager messager, Ros2Node ros2Node, DRCRobotModel robotModel)
   {
      this.messager = messager;

      LogTools.debug("Initializing patrol behavior");

      footstepDataListPublisher = ROS2Tools.createPublisher(ros2Node,
                                                            ROS2Tools.newMessageInstance(FootstepDataListCommand.class).getMessageClass(),
                                                            ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotModel.getSimpleRobotName()));
      pausePublisher = ROS2Tools.createPublisher(ros2Node,
                                                 ROS2Tools.newMessageInstance(PauseWalkingCommand.class).getMessageClass(),
                                                 ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotModel.getSimpleRobotName()));
      ROS2Tools.createCallbackSubscription(ros2Node,
                                           WalkingStatusMessage.class,
                                           ControllerAPIDefinition.getPublisherTopicNameGenerator(robotModel.getSimpleRobotName()),
                                           this::updateWalkingCompletedStatus);

      ROS2Tools.createCallbackSubscription(ros2Node, PlanarRegionsListMessage.class,
                                           ROS2Tools.getTopicNameGenerator(robotModel.getSimpleRobotName(),
                                                                           ROS2Tools.REA_MODULE,
                                                                           ROS2Tools.ROS2TopicQualifier.OUTPUT),
                                           this::receivePlanarRegions);

      remoteSyncedHumanoidFrames = new RemoteSyncedHumanoidFrames(robotModel, ros2Node);

      remoteFootstepPlannerInterface = new RemoteFootstepPlannerInterface(ros2Node, robotModel);

      messager.registerTopicListener(API.Stop, object -> stopNotification.set());
      messager.registerTopicListener(API.GoToWaypoint, goToWaypointIndex ->
      {
         goalWaypointIndex.set(goToWaypointIndex);
         overrideGoToWaypointNotification.set();
      });

      waypoints = messager.createInput(API.Waypoints);

      transitionTo(STOP);

      PeriodicNonRealtimeThreadScheduler patrolThread = new PeriodicNonRealtimeThreadScheduler(getClass().getSimpleName());
      patrolThread.schedule(this::patrolThread, 2, TimeUnit.MILLISECONDS); // TODO tune this up, 500Hz is probably too much
   }

   private void patrolThread()   // pretty much just updating whichever state is active
   {
      currentState.getValue().run();
   }

   private void updateStopState()
   {
      checkForInterruptions();
   }

   private void updateRetrieveFootstepPlanState()
   {
      if (checkForInterruptions()) return;  // need to return in case of stop condition

      if (readyToPlanNotification.poll())
      {
         FramePose3D midFeetZUpPose = new FramePose3D();
         // prevent frame from continuing to change
         midFeetZUpPose.setFromReferenceFrame(remoteSyncedHumanoidFrames.pollHumanoidReferenceFrames().getMidFeetZUpFrame());
         int index = goalWaypointIndex.get();
         messager.submitMessage(API.CurrentWaypointIndexStatus, index);
         FramePose3D currentGoalWaypoint = new FramePose3D(waypoints.get().get(index));

         new Thread(() -> {  // plan in a thread to catch interruptions during planning
            footstepPlanningOutput = remoteFootstepPlannerInterface.requestPlanBlocking(midFeetZUpPose, currentGoalWaypoint, latestPlanarRegions.get());
            FootstepPlan footstepPlan = FootstepDataMessageConverter.convertToFootstepPlan(footstepPlanningOutput.getFootstepDataList());
            ArrayList<Pair<RobotSide, Pose3D>> footstepLocations = new ArrayList<>();
            for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)  // this code makes the message smaller to send over the network, TODO investigate
            {
               FramePose3D soleFramePoseToPack = new FramePose3D();
               footstepPlan.getFootstep(i).getSoleFramePose(soleFramePoseToPack);
               footstepLocations.add(new MutablePair<>(footstepPlan.getFootstep(i).getRobotSide(), new Pose3D(soleFramePoseToPack)));
            }
            messager.submitMessage(API.CurrentFootstepPlan, footstepLocations);
            footstepPlanCompleted.set();
         }).start();
      }
      else if (footstepPlanCompleted.poll())
      {
         boolean walkable =
               footstepPlanningOutput.getFootstepPlanningResult() == FootstepPlanningToolboxOutputStatus.FOOTSTEP_PLANNING_RESULT_OPTIMAL_SOLUTION ||
               footstepPlanningOutput.getFootstepPlanningResult() == FootstepPlanningToolboxOutputStatus.FOOTSTEP_PLANNING_RESULT_SUB_OPTIMAL_SOLUTION;

         if (walkable)
         {
            walkingCompleted.poll(); // acting to clear the notification
            LogTools.debug("Tasking {} footstep(s) to the robot", footstepPlanningOutput.getFootstepDataList().getFootstepDataList().size());
            footstepDataListPublisher.publish(footstepPlanningOutput.getFootstepDataList());
            transitionTo(WALK_TO_GOAL_WAYPOINT);
         }
         else
         {
            LogTools.warn("Footstep plan result: {}", FootstepPlanningResult.fromByte(footstepPlanningOutput.getFootstepPlanningResult()).name());
            readyToPlanNotification.set();  // plan again until walkable plan, also functions to wait for perception data and user modifications
         }
      }
   }

   private void updateWalkToGoalWaypointState()
   {
      if (checkForInterruptions()) return;

      if (walkingCompleted.poll())  // in the future, may want to count steps and assert robot got to the waypoint bounding box
      {
         ArrayList<Pose3D> latestWaypoints = waypoints.get();      // access and store these early
         int nextGoalWaypointIndex = goalWaypointIndex.get() + 1;  // to make thread-safe
         if (nextGoalWaypointIndex >= latestWaypoints.size())
            nextGoalWaypointIndex = 0;
         goalWaypointIndex.set(nextGoalWaypointIndex);
         readyToPlanNotification.set();
         transitionTo(RETRIEVE_FOOTSTEP_PLAN);
      }
   }

   private boolean checkForInterruptions()
   {
      stopNotification.poll();         // poll both at the same time to handle race condition
      overrideGoToWaypointNotification.poll();

      if (stopNotification.read())  // favor stop if race condition
      {
         LogTools.info("Interrupted with STOP");
         sendPauseWalking();
         transitionTo(STOP);
      }

      if (overrideGoToWaypointNotification.read())
      {
         LogTools.info("Interrupted with GO_TO_WAYPOINT {}", goalWaypointIndex.get());
         sendPauseWalking();

         ArrayList<Pose3D> latestWaypoints = waypoints.get();     // access and store these early
         int currentGoalWaypointIndex = goalWaypointIndex.get();  // to make thread-safe
         if (currentGoalWaypointIndex >= 0 && currentGoalWaypointIndex < latestWaypoints.size())
         {
            readyToPlanNotification.set();
            transitionTo(RETRIEVE_FOOTSTEP_PLAN);
         }
         else
         {
            transitionTo(STOP); // TODO submit message about incorrect index
         }
      }

      return stopNotification.read() || overrideGoToWaypointNotification.read();
   }

   private void sendPauseWalking()
   {
      LogTools.debug("Sending pause walking to robot");
      PauseWalkingMessage pause = new PauseWalkingMessage();
      pause.setPause(true);
      pausePublisher.publish(pause);
   }

   private void transitionTo(Pair<String, Runnable> stateToTransitionTo)
   {
      LogTools.debug("Transitioning to {}", stateToTransitionTo.getKey());
      currentState = stateToTransitionTo;
      messager.submitMessage(API.CurrentState, currentState.getKey());
   }

   private void updateWalkingCompletedStatus(Subscriber<WalkingStatusMessage> status)
   {
      WalkingStatusMessage walkingStatusMessage;
      while ((walkingStatusMessage = status.takeNextData()) != null)
      {
         LogTools.debug("Walking status: {}", WalkingStatus.fromByte(walkingStatusMessage.getWalkingStatus()).name());
         if (walkingStatusMessage.getWalkingStatus() == WalkingStatusMessage.COMPLETED)
         {
            walkingCompleted.set();
         }
      }
   }

   private void receivePlanarRegions(Subscriber<PlanarRegionsListMessage> subscriber)
   {
      PlanarRegionsListMessage incomingData = subscriber.takeNextData(); // may be 1 or 2 ticks behind, is this okay?
      if (incomingData != null)
      {
         latestPlanarRegions.set(incomingData);
      }
   }

   public static class API
   {
      private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
      private static final Category Root = apiFactory.createRootCategory("PatrolBehavior");
      private static final CategoryTheme Patrol = apiFactory.createCategoryTheme("Patrol");

      /** Input: Update the waypoints */
      public static final Topic<ArrayList<Pose3D>> Waypoints = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("Waypoints"));

      /** Input: Robot stops and immediately goes to this waypoint. The "start" or "reset" command.  */
      public static final Topic<Integer> GoToWaypoint = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("GoToWaypoint"));

      /** Input: When received, the robot stops walking and waits forever. */
      public static final Topic<Object> Stop = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("Stop"));

      /** Output: to visualize the current robot path plan. */
      public static final Topic<ArrayList<Pair<RobotSide, Pose3D>>> CurrentFootstepPlan
            = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("CurrentFootstepPlan"));

      /** Output: to visualize the current state. */
      public static final Topic<String> CurrentState = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("CurrentState"));

      /** Output: to visualize the current waypoint status. TODO clean me up */
      public static final Topic<Integer> CurrentWaypointIndexStatus
            = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("CurrentWaypointIndexStatus"));

      public static final MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }
}
