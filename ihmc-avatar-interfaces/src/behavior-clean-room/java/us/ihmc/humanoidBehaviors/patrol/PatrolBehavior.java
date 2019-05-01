package us.ihmc.humanoidBehaviors.patrol;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.REAStateRequestMessage;
import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;

import com.google.common.collect.Lists;

import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Input;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.humanoidBehaviors.tools.RemoteFootstepPlannerInterface;
import us.ihmc.humanoidBehaviors.tools.RemoteFootstepPlannerInterface.PlanType;
import us.ihmc.humanoidBehaviors.tools.RemoteRobotControllerInterface;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedHumanoidFrames;
import us.ihmc.humanoidBehaviors.upDownExploration.PlanarRegionUpDownNavigation;
import us.ihmc.humanoidBehaviors.upDownExploration.PlanarRegionUpDownNavigation.NavigationResult;
import us.ihmc.humanoidBehaviors.waypoints.WaypointSequence;
import us.ihmc.humanoidBehaviors.waypoints.WaypointManager;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.extra.EnumBasedStateMachineFactory;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;
import us.ihmc.tools.thread.TypedNotification;

import static us.ihmc.humanoidBehaviors.patrol.PatrolBehavior.PatrolBehaviorState.*;

/**
 * Walk through a list of waypoints in order, looping forever.
 */
public class PatrolBehavior
{
   public static final double TIME_TO_PERCEIVE = 20.0;

   public enum PatrolBehaviorState
   {
      /** Stop state that waits for or is triggered by a GoToWaypoint message */
      STOP,
      /** Decide on waypoints. */
//      NAVIGATE,
//      /** Request and wait for footstep planner result */
      PLAN,
      /** Optional user review of footstep plan. */
      REVIEW,
      /** Walking towards goal waypoint */
      WALK,
      /** Wait to gather perception data */
      PERCEIVE
   }

   public enum OperatorPlanReviewResult
   {
      REPLAN, WALK
   }

   private final Messager messager;
   private final StateMachine<PatrolBehaviorState, State> stateMachine;

   private final ROS2Input<PlanarRegionsListMessage> planarRegionsList;
   private final IHMCROS2Publisher<REAStateRequestMessage> reaStateRequestPublisher;
   private final RemoteRobotControllerInterface remoteRobotControllerInterface;
   private final RemoteSyncedHumanoidFrames remoteSyncedHumanoidFrames;
   private final RemoteFootstepPlannerInterface remoteFootstepPlannerInterface;

   private final Notification stopNotification = new Notification();
   private final Notification goNotification = new Notification();
   private final TypedNotification<OperatorPlanReviewResult> planReviewResult = new TypedNotification<>();

   private TypedNotification<FootstepPlanningToolboxOutputStatus> footstepPlanResultNotification;
   private TypedNotification<WalkingStatusMessage> walkingCompleted;

   private final WaypointManager waypointManager;
   private final AtomicReference<Boolean> loop;
   private final AtomicReference<Boolean> swingOvers;
   private final AtomicReference<Boolean> planReview;
   private final AtomicReference<Boolean> upDownExploration;

   public PatrolBehavior(Messager messager, Ros2Node ros2Node, DRCRobotModel robotModel)
   {
      this.messager = messager;

      LogTools.debug("Initializing patrol behavior");

      EnumBasedStateMachineFactory<PatrolBehaviorState> factory = new EnumBasedStateMachineFactory<>(PatrolBehaviorState.class);
      factory.getStateMap().get(STOP).setOnEntry(this::onStopStateEntry);
      factory.getStateMap().get(STOP).setDoAction(this::doStopStateAction);
      factory.getFactory().addTransition(STOP, PLAN, this::transitionFromStop);
      factory.getStateMap().get(PLAN).setOnEntry(this::onPlanStateEntry);
      factory.getStateMap().get(PLAN).setDoAction(this::doPlanStateAction);
      factory.addTransition(PLAN, Lists.newArrayList(REVIEW, PLAN, STOP), this::transitionFromPlan);
      factory.getStateMap().get(REVIEW).setOnEntry(this::onReviewStateEntry);
      factory.getStateMap().get(REVIEW).setDoAction(this::onReviewStateAction);
      factory.addTransition(REVIEW, Lists.newArrayList(WALK, PLAN, STOP), this::transitionFromReview);
      factory.getStateMap().get(WALK).setOnEntry(this::onWalkStateEntry);
      factory.getStateMap().get(WALK).setDoAction(this::doWalkStateAction);
      factory.getStateMap().get(WALK).setOnExit(this::onWalkStateExit);
      factory.addTransition(WALK, Lists.newArrayList(PERCEIVE, PLAN, STOP), this::transitionFromWalk);
      factory.getStateMap().get(PERCEIVE).setOnEntry(this::onPerceiveStateEntry);
      factory.addTransition(PERCEIVE, Lists.newArrayList(PLAN, STOP), this::transitionFromPerceive);
      factory.getFactory().addStateChangedListener((from, to) ->
                                                   {
                                                      messager.submitMessage(API.CurrentState, to);
                                                      LogTools.debug("{} -> {}", from == null ? null : from.name(), to == null ? null : to.name());
                                                   });
      factory.getFactory().buildClock(() -> Conversions.nanosecondsToSeconds(System.nanoTime()));
      stateMachine = factory.getFactory().build(STOP);

      planarRegionsList = new ROS2Input<>(ros2Node, PlanarRegionsListMessage.class, null, LIDARBasedREAModule.ROS2_ID);
      reaStateRequestPublisher = new IHMCROS2Publisher<>(ros2Node, REAStateRequestMessage.class, null, LIDARBasedREAModule.ROS2_ID);
      remoteRobotControllerInterface = new RemoteRobotControllerInterface(ros2Node, robotModel);
      remoteSyncedHumanoidFrames = new RemoteSyncedHumanoidFrames(robotModel, ros2Node);
      remoteFootstepPlannerInterface = new RemoteFootstepPlannerInterface(ros2Node, robotModel);

      waypointManager = WaypointManager.createForModule(messager,
                                                        API.WaypointsToModule,
                                                        API.WaypointsToUI,
                                                        API.GoToWaypoint,
                                                        API.CurrentWaypointIndexStatus,
                                                        goNotification);

      messager.registerTopicListener(API.Stop, object -> stopNotification.set());
      messager.registerTopicListener(API.PlanReviewResult, message ->
      {
         planReviewResult.add(message);
      });

      loop = messager.createInput(API.Loop, false);
      swingOvers = messager.createInput(API.SwingOvers, false);
      planReview = messager.createInput(API.PlanReviewEnabled, false);
      upDownExploration = messager.createInput(API.UpDownExplorationEnabled, false);

      ExceptionHandlingThreadScheduler patrolThread = new ExceptionHandlingThreadScheduler(getClass().getSimpleName(),
                                                                                           DefaultExceptionHandler.PRINT_STACKTRACE,
                                                                                           5);
      patrolThread.schedule(this::patrolThread, 2, TimeUnit.MILLISECONDS); // TODO tune this up, 500Hz is probably too much
   }

   private void patrolThread()   // pretty much just updating whichever state is active
   {
      stateMachine.doActionAndTransition();
   }

   private void onStopStateEntry()
   {
      remoteRobotControllerInterface.pauseWalking();
   }

   private void doStopStateAction(double timeInState)
   {
      pollInterrupts();
      goNotification.poll();
      if (goNotification.read())
      {
         LogTools.debug("Go notified. Number of waypoints: {}", waypointManager.size());
      }
   }

   private boolean transitionFromStop(double timeInState)
   {
      boolean transition = goNotification.read() && waypointManager.hasWaypoints();
      if (transition)
      {
         LogTools.debug("STOP -> PLAN");
      }
      return transition;
   }

   private void onPlanStateEntry()
   {
      // update waypoints if UI modified them
      waypointManager.updateToMostRecentData();

      remoteFootstepPlannerInterface.abortPlanning();

      FramePose3DReadOnly midFeetZUpPose = remoteSyncedHumanoidFrames.quickPollPoseReadOnly(HumanoidReferenceFrames::getMidFeetZUpFrame);

      if (upDownExploration.get()) // find up-down or spin. setup the waypoint
      {
         Pair<NavigationResult, FramePose3D> upOrDownResult = PlanarRegionUpDownNavigation
                        .upOrDown(remoteSyncedHumanoidFrames.pollHumanoidReferenceFrames().getMidFeetZUpFrame(),
                                  PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsList.getLatest()));
                  if (upOrDownResult.getLeft() == NavigationResult.WAYPOINT_FOUND)
                  {
                     // success
                  }
                  else
                  {
                     // turn
                  }
      }

//      int index = goalWaypointIndex.get();
//      messager.submitMessage(API.CurrentWaypointIndexStatus, index);
      FramePose3D currentGoalWaypoint = new FramePose3D(waypointManager.peekNextPose());

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
      else if (footstepPlanResultNotification.hasNext())
      {
         if (FootstepPlanningResult.fromByte(footstepPlanResultNotification.peek().getFootstepPlanningResult()).validForExecution())
         {
            return REVIEW;
         }
         else
         {
            return PLAN;
         }
      }

      return null;
   }

   private void onReviewStateEntry()
   {
      reduceAndSendFootstepsForVisualization(footstepPlanResultNotification.peek());
   }

   private void onReviewStateAction(double timeInState)
   {
      pollInterrupts();
      planReviewResult.poll();
   }

   private PatrolBehaviorState transitionFromReview(double timeInState)
   {
      if (stopNotification.read())
      {
         return STOP;
      }
      else if (!planReview.get()) // if review is disabled, proceed
      {
         return WALK;
      }
      else if (planReviewResult.hasNext())
      {
         if (planReviewResult.peek() == OperatorPlanReviewResult.REPLAN)
         {
            return PLAN;
         }
         else if (planReviewResult.peek() == OperatorPlanReviewResult.WALK)
         {
            return WALK;
         }
      }

      return null;
   }

   private void onWalkStateEntry()
   {
      walkingCompleted = remoteRobotControllerInterface
            .requestWalk(footstepPlanResultNotification.peek(), remoteSyncedHumanoidFrames.pollHumanoidReferenceFrames(), swingOvers.get());
   }

   private void doWalkStateAction(double timeInState)
   {
      pollInterrupts();
      walkingCompleted.poll();
      waypointManager.updateToMostRecentData();
   }

   private PatrolBehaviorState transitionFromWalk(double timeInState)
   {
      if (stopNotification.read())
      {
         return STOP;
      }
      else if (walkingCompleted.hasNext()) // TODO handle robot fell and more
      {
//         int currentGoalWaypoint = goalWaypointIndex.get();
//         int nextWaypointIndex = currentGoalWaypoint + 1;
//         ArrayList<Pose3D> currentWaypointList = waypoints.get();
         if (!loop.get() && waypointManager.incrementingWillLoop()) // stop if looping disabled
         {
            return STOP;
         }
         else
         {
            // next waypoint is far, gather more data to increase robustness
            if (remoteFootstepPlannerInterface.decidePlanType(remoteSyncedHumanoidFrames.quickPollPoseReadOnly(HumanoidReferenceFrames::getMidFeetZUpFrame),
                                                              waypointManager.peekAfterNextPose())
                  == PlanType.FAR)
            {
               return PERCEIVE;
            }
            else // next waypoint is close
            {
               return PLAN;
            }
         }
      }

      return null;
   }

   private void onWalkStateExit()
   {
      if (!stopNotification.read()) // only increment in normal operation (WALK -> PLAN or end with loop disabled)
      {
         waypointManager.increment();
      }
   }

   private void onPerceiveStateEntry()
   {
      REAStateRequestMessage clearMessage = new REAStateRequestMessage();
      clearMessage.setRequestClear(true);
      reaStateRequestPublisher.publish(clearMessage);
   }

   private PatrolBehaviorState transitionFromPerceive(double timeInState)
   {
      if (stopNotification.read())
      {
         return STOP;
      }
      else if (timeInState > TIME_TO_PERCEIVE)
      {
         return PLAN;
      }

      return null;
   }

   private void pollInterrupts()
   {
      HighLevelControllerName controllerState = remoteRobotControllerInterface.latestControllerState();
      if (!stateMachine.getCurrentStateKey().equals(STOP)
            && controllerState != HighLevelControllerName.WALKING) // STOP if robot falls
      {
         LogTools.debug("Stopping from robot state: {}", controllerState.name());
         stopNotification.set();
      }

      stopNotification.poll();         // poll both at the same time to handle race condition
   }

   private void reduceAndSendFootstepsForVisualization(FootstepPlanningToolboxOutputStatus footstepPlanningOutput)
   {
      FootstepPlan footstepPlan = FootstepDataMessageConverter.convertToFootstepPlan(footstepPlanningOutput.getFootstepDataList());
      ArrayList<Pair<RobotSide, Pose3D>> footstepLocations = new ArrayList<>();
      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)  // this code makes the message smaller to send over the network, TODO investigate
      {
         FramePose3D soleFramePoseToPack = new FramePose3D();
         footstepPlan.getFootstep(i).getSoleFramePose(soleFramePoseToPack);
         footstepLocations.add(new MutablePair<>(footstepPlan.getFootstep(i).getRobotSide(), new Pose3D(soleFramePoseToPack)));
      }
      messager.submitMessage(API.CurrentFootstepPlan, footstepLocations);
   }

   public static class API
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

      /** Input: Toggle looping through waypoints. */
      public static final Topic<Boolean> Loop = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("Loop"));

      /** Input: Toggle swinging over planar regions. */
      public static final Topic<Boolean> SwingOvers = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("SwingOvers"));

      /** Input: Enable/disable human plan review before walking. */
      public static final Topic<Boolean> PlanReviewEnabled = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("PlanReviewEnabled"));

      /** Input: Enable/disable human plan review before walking. */
      public static final Topic<Boolean> UpDownExplorationEnabled
            = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("UpDownExplorationEnabled"));

      /** Input: Enable/disable human plan review before walking. */
      public static final Topic<OperatorPlanReviewResult> PlanReviewResult
            = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("PlanReviewResult"));

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
}
