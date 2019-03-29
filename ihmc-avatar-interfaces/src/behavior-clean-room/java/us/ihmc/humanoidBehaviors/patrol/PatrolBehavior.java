package us.ihmc.humanoidBehaviors.patrol;

import boofcv.core.image.FactoryGImageMultiBand.PL;
import com.google.common.collect.Lists;
import controller_msgs.msg.dds.*;
import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Callback;
import us.ihmc.communication.ROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.humanoidBehaviors.tools.*;
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
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.humanoidBehaviors.patrol.PatrolBehavior.PatrolBehaviorState.*;

/**
 * Walk through a list of waypoints in order, looping forever.
 */
public class PatrolBehavior
{
   /** Stop state that waits for or is triggered by a GoToWaypoint message */
   private final Pair<String, Runnable> STOP_PAIR = Pair.of("STOP_PAIR", this::updateStopState);
   /** Request and wait for footstep planner result */
   private final Pair<String, Runnable> RETRIEVE_FOOTSTEP_PLAN = Pair.of("RETRIEVE_FOOTSTEP_PLAN", this::updateRetrieveFootstepPlanState);
   /** Walking towards goal waypoint */
   private final Pair<String, Runnable> WALK_TO_GOAL_WAYPOINT = Pair.of("WALK_TO_GOAL_WAYPOINT", this::updateWalkToGoalWaypointState);

   private Pair<String, Runnable> currentState;

   enum PatrolBehaviorState
   {
      STOP, PLAN, WALK
   }

   private final Messager messager;
   private final IHMCROS2Publisher<FootstepDataListMessage> footstepDataListPublisher;
   private final IHMCROS2Publisher<PauseWalkingMessage> pausePublisher;
   private final RemoteSyncedHumanoidFrames remoteSyncedHumanoidFrames;
   private final RemoteFootstepPlannerInterface remoteFootstepPlannerInterface;
   private final ROS2Input<PlanarRegionsListMessage> planarRegionsList;

   private final Notification stopNotification = new Notification();
   private final Notification overrideGoToWaypointNotification = new Notification();

   private final AtomicInteger goalWaypointIndex = new AtomicInteger();

   private final Notification readyToPlanNotification = new Notification();
   private TypedNotification<FootstepPlanningToolboxOutputStatus> footstepPlanResultNotification;
   private final Notification walkingCompleted = new Notification();

   private final AtomicReference<ArrayList<Pose3D>> waypoints;

   public PatrolBehavior(Messager messager, Ros2Node ros2Node, DRCRobotModel robotModel)
   {
      this.messager = messager;

      LogTools.debug("Initializing patrol behavior");

      CustomBehaviorStateMachineFactory<PatrolBehaviorState> factory = new CustomBehaviorStateMachineFactory<>(PatrolBehaviorState.class);
      factory.getStateMap().get(STOP).setOnEntry(this::onStopStateEntry);
      factory.getStateMap().get(STOP).setDoAction(this::doStopStateAction);
      factory.getFactory().addTransition(STOP, PLAN, this::transitionFromStopToPlan);
      factory.getStateMap().get(PLAN).setOnEntry(this::onPlanStateEntry);
      factory.getStateMap().get(PLAN).setDoAction(this::doPlanStateAction);
      factory.addTransition(PLAN, Lists.newArrayList(STOP, PLAN, WALK), this::transitionFromPlan);
      factory.getStateMap().get(WALK).setOnEntry(this::onWalkStateEntry);
      factory.getStateMap().get(WALK).setDoAction(this::doWalkStateAction);
      factory.addTransition(WALK, Lists.newArrayList(STOP, PLAN), this::transitionFromWalk);

      footstepDataListPublisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.newMessageInstance(FootstepDataListCommand.class).getMessageClass(),
                                                            ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotModel.getSimpleRobotName()));
      pausePublisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.newMessageInstance(PauseWalkingCommand.class).getMessageClass(),
                                                 ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotModel.getSimpleRobotName()));

      new ROS2Callback<>(ros2Node, WalkingStatusMessage.class, robotModel.getSimpleRobotName(), ROS2Tools.HUMANOID_CONTROL_MODULE, this::acceptWalkingStatus);

      planarRegionsList = new ROS2Input<>(ros2Node, PlanarRegionsListMessage.class, robotModel.getSimpleRobotName(), ROS2Tools.REA_MODULE);

      remoteSyncedHumanoidFrames = new RemoteSyncedHumanoidFrames(robotModel, ros2Node);

      remoteFootstepPlannerInterface = new RemoteFootstepPlannerInterface(ros2Node, robotModel);

      messager.registerTopicListener(API.Stop, object -> stopNotification.set());
      messager.registerTopicListener(API.GoToWaypoint, goToWaypointIndex -> {
         goalWaypointIndex.set(goToWaypointIndex);
         LogTools.info("Interrupted with GO_TO_WAYPOINT {}", goalWaypointIndex.get());
         overrideGoToWaypointNotification.set();
      });

      waypoints = messager.createInput(API.Waypoints);

      transitionTo(STOP_PAIR);

      PeriodicNonRealtimeThreadScheduler patrolThread = new PeriodicNonRealtimeThreadScheduler(getClass().getSimpleName());
      patrolThread.schedule(this::patrolThread, 2, TimeUnit.MILLISECONDS); // TODO tune this up, 500Hz is probably too much
   }

   private void patrolThread()   // pretty much just updating whichever state is active
   {
      currentState.getValue().run();
   }

   private void onStopStateEntry()
   {
      sendPauseWalking(); // TODO make into tool
   }

   private void doStopStateAction(double timeInState)
   {
      pollInterrupts();
   }

   private boolean transitionFromStopToPlan(double timeInState)
   {
      return overrideGoToWaypointNotification.read() && goalWaypointInBounds();
   }

   private void onPlanStateEntry()
   {
      remoteFootstepPlannerInterface.abortPlanning();

      FramePose3D midFeetZUpPose = new FramePose3D();
      // prevent frame from continuing to change
      midFeetZUpPose.setFromReferenceFrame(remoteSyncedHumanoidFrames.pollHumanoidReferenceFrames().getMidFeetZUpFrame());
      int index = goalWaypointIndex.get();
      messager.submitMessage(API.CurrentWaypointIndexStatus, index);
      FramePose3D currentGoalWaypoint = new FramePose3D(waypoints.get().get(index));

      footstepPlanResultNotification = remoteFootstepPlannerInterface.requestPlan(midFeetZUpPose, currentGoalWaypoint, planarRegionsList.getLatest());
   }

   private void doPlanStateAction(double timeInState)
   {
      pollInterrupts();
      footstepPlanResultNotification.poll();
   }

   private PatrolBehaviorState transitionFromPlan(double timeInState)
   {
      if (stopNotification.read())
      {
         return STOP;
      }
      else if (overrideGoToWaypointNotification.read())
      {
         if (goalWaypointInBounds())
         {
            return PLAN;
         }
         else
         {
            return STOP;
         }
      }
      else if (footstepPlanResultNotification.hasNext())
      {
         if (FootstepPlanningResult.fromByte(footstepPlanResultNotification.read().getFootstepPlanningResult()).validForExecution())
         {
            return WALK;
         }
         else
         {
            return PLAN;
         }
      }

      return null;
   }

   private void onWalkStateEntry()
   {
      FootstepPlanningToolboxOutputStatus footstepPlanningOutput = footstepPlanResultNotification.read();
      FootstepPlan footstepPlan = FootstepDataMessageConverter.convertToFootstepPlan(footstepPlanningOutput.getFootstepDataList());
      ArrayList<Pair<RobotSide, Pose3D>> footstepLocations = new ArrayList<>();
      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)  // this code makes the message smaller to send over the network, TODO investigate
      {
         FramePose3D soleFramePoseToPack = new FramePose3D();
         footstepPlan.getFootstep(i).getSoleFramePose(soleFramePoseToPack);
         footstepLocations.add(new MutablePair<>(footstepPlan.getFootstep(i).getRobotSide(), new Pose3D(soleFramePoseToPack)));
      }
      messager.submitMessage(API.CurrentFootstepPlan, footstepLocations);

      walkingCompleted.poll(); // acting to clear the notification TODO return walk result same as footstep plan
      LogTools.debug("Tasking {} footstep(s) to the robot", footstepPlanningOutput.getFootstepDataList().getFootstepDataList().size());
      footstepDataListPublisher.publish(footstepPlanningOutput.getFootstepDataList());
   }

   private void doWalkStateAction(double timeInState)
   {
      pollInterrupts();
      walkingCompleted.poll();
   }

   private PatrolBehaviorState transitionFromWalk(double timeInState)
   {
      if (stopNotification.read())
      {
         return STOP;
      }
      else if (overrideGoToWaypointNotification.read())
      {
         if (goalWaypointInBounds())
         {
            return PLAN;
         }
         else
         {
            return STOP;
         }
      }
      else if (walkingCompleted.read())
      {
         return PLAN;
      }

      return null;
   }

   private void onWalkStateExit()
   {
      if (!stopNotification.read() && !overrideGoToWaypointNotification.read()) // only increment if WALK -> PLAN
      {
         ArrayList<Pose3D> latestWaypoints = waypoints.get();      // access and store these early
         int nextGoalWaypointIndex = goalWaypointIndex.get() + 1;  // to make thread-safe
         if (nextGoalWaypointIndex >= latestWaypoints.size())
            nextGoalWaypointIndex = 0;
         goalWaypointIndex.set(nextGoalWaypointIndex);
      }
   }

   private void updateRetrieveFootstepPlanState()
   {
      if (checkForInterruptions())
         return;  // need to return in case of stop condition

      if (readyToPlanNotification.poll())
      {
         FramePose3D midFeetZUpPose = new FramePose3D();
         // prevent frame from continuing to change
         midFeetZUpPose.setFromReferenceFrame(remoteSyncedHumanoidFrames.pollHumanoidReferenceFrames().getMidFeetZUpFrame());
         int index = goalWaypointIndex.get();
         messager.submitMessage(API.CurrentWaypointIndexStatus, index);
         FramePose3D currentGoalWaypoint = new FramePose3D(waypoints.get().get(index));

         footstepPlanResultNotification = remoteFootstepPlannerInterface.requestPlan(midFeetZUpPose, currentGoalWaypoint, planarRegionsList.getLatest());
      }
      else if (footstepPlanResultNotification.poll())
      {
         FootstepPlanningToolboxOutputStatus footstepPlanningOutput = footstepPlanResultNotification.read();
         FootstepPlan footstepPlan = FootstepDataMessageConverter.convertToFootstepPlan(footstepPlanningOutput.getFootstepDataList());
         ArrayList<Pair<RobotSide, Pose3D>> footstepLocations = new ArrayList<>();
         for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)  // this code makes the message smaller to send over the network, TODO investigate
         {
            FramePose3D soleFramePoseToPack = new FramePose3D();
            footstepPlan.getFootstep(i).getSoleFramePose(soleFramePoseToPack);
            footstepLocations.add(new MutablePair<>(footstepPlan.getFootstep(i).getRobotSide(), new Pose3D(soleFramePoseToPack)));
         }
         messager.submitMessage(API.CurrentFootstepPlan, footstepLocations);

         if (FootstepPlanningResult.fromByte(footstepPlanningOutput.getFootstepPlanningResult()).validForExecution())
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
      if (checkForInterruptions())
         return;

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

   private void pollInterrupts()
   {
      stopNotification.poll();         // poll both at the same time to handle race condition
      overrideGoToWaypointNotification.poll();
   }

   private boolean checkForInterruptions()
   {
      stopNotification.poll();         // poll both at the same time to handle race condition
      overrideGoToWaypointNotification.poll();

      boolean interrupted = stopNotification.read() || overrideGoToWaypointNotification.read();

      if (interrupted)
      {
         sendPauseWalking();
         remoteFootstepPlannerInterface.abortPlanning();
      }

      if (stopNotification.read())  // favor stop if race condition
      {
         LogTools.info("Interrupted with STOP");
         transitionTo(STOP_PAIR);
      }

      if (overrideGoToWaypointNotification.read())
      {
         LogTools.info("Interrupted with GO_TO_WAYPOINT {}", goalWaypointIndex.get());

         ArrayList<Pose3D> latestWaypoints = waypoints.get();     // access and store these early
         int currentGoalWaypointIndex = goalWaypointIndex.get();  // to make thread-safe
         if (currentGoalWaypointIndex >= 0 && currentGoalWaypointIndex < latestWaypoints.size())
         {
            readyToPlanNotification.set();
            transitionTo(RETRIEVE_FOOTSTEP_PLAN);
         }
         else
         {
            transitionTo(STOP_PAIR); // TODO submit message about incorrect index
         }
      }

      return interrupted;
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

   private boolean goalWaypointInBounds()
   {
      ArrayList<Pose3D> latestWaypoints = waypoints.get();     // access and store these early
      int currentGoalWaypointIndex = goalWaypointIndex.get();  // to make thread-safe
      boolean indexInBounds = currentGoalWaypointIndex >= 0 && currentGoalWaypointIndex < latestWaypoints.size();
      return indexInBounds;
   }

   private void acceptWalkingStatus(WalkingStatusMessage message)
   {
      LogTools.debug("Walking status: {}", WalkingStatus.fromByte(message.getWalkingStatus()).name());
      if (message.getWalkingStatus() == WalkingStatusMessage.COMPLETED)
      {
         walkingCompleted.set();
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
      public static final Topic<ArrayList<Pair<RobotSide, Pose3D>>> CurrentFootstepPlan = Root.child(Patrol)
                                                                                              .topic(apiFactory.createTypedTopicTheme("CurrentFootstepPlan"));

      /** Output: to visualize the current state. */
      public static final Topic<String> CurrentState = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("CurrentState"));

      /** Output: to visualize the current waypoint status. TODO clean me up */
      public static final Topic<Integer> CurrentWaypointIndexStatus = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("CurrentWaypointIndexStatus"));

      public static final MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }
}
