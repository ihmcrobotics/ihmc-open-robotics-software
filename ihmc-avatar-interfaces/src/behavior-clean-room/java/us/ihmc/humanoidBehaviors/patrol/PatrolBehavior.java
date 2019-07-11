package us.ihmc.humanoidBehaviors.patrol;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.REAStateRequestMessage;
import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;

import com.google.common.collect.Lists;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Input;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.RemoteFootstepPlannerInterface;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.PlanTravelDistance;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.humanoidBehaviors.tools.RemoteRobotControllerInterface;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedHumanoidFrames;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.RemoteFootstepPlannerResult;
import us.ihmc.humanoidBehaviors.upDownExploration.UpDownExplorer;
import us.ihmc.humanoidBehaviors.waypoints.WaypointManager;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.extra.EnumBasedStateMachineFactory;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;
import us.ihmc.tools.thread.TypedNotification;

import static us.ihmc.humanoidBehaviors.patrol.PatrolBehavior.PatrolBehaviorState.*;
import static us.ihmc.humanoidBehaviors.patrol.PatrolBehaviorAPI.*;

/**
 * Walk through a list of waypoints in order, looping forever.
 */
public class PatrolBehavior
{
   public enum PatrolBehaviorState
   {
      /** Stop state that waits for or is triggered by a GoToWaypoint message */
      STOP,
      /** Decide on waypoints. */
      NAVIGATE,
      /** Request and wait for footstep planner result */
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

   private final BehaviorHelper behaviorHelper;

   private final Messager messager;
   private final StateMachine<PatrolBehaviorState, State> stateMachine;

   private final IHMCROS2Publisher<REAStateRequestMessage> reaStateRequestPublisher;
   private final RemoteRobotControllerInterface remoteRobotControllerInterface;
   private final RemoteFootstepPlannerInterface remoteFootstepPlannerInterface;
   private final UpDownExplorer upDownExplorer;

   private final Notification stopNotification = new Notification();
   private final Notification goNotification = new Notification();
   private final TypedNotification<OperatorPlanReviewResult> planReviewResult = new TypedNotification<>();
   private final Notification cancelPlanning = new Notification();
   private final Notification skipPerceive = new Notification();

   private TypedNotification<RemoteFootstepPlannerResult> footstepPlanResultNotification;
   private TypedNotification<WalkingStatusMessage> walkingCompleted;

   private final WaypointManager waypointManager;
   private final AtomicReference<Boolean> loop;
   private final AtomicReference<Boolean> swingOvers;
   private final AtomicReference<Boolean> planReviewEnabled;
   private final AtomicReference<Boolean> upDownExplorationEnabled;
   private final AtomicReference<Double> perceiveDuration;

   public PatrolBehavior(Messager messager, Ros2Node ros2Node, DRCRobotModel robotModel)
   {
      behaviorHelper = new BehaviorHelper(messager, robotModel, ros2Node);

      this.messager = messager;

      LogTools.debug("Initializing patrol behavior");

      EnumBasedStateMachineFactory<PatrolBehaviorState> factory = new EnumBasedStateMachineFactory<>(PatrolBehaviorState.class);
      factory.setOnEntry(STOP, this::onStopStateEntry);
      factory.setDoAction(STOP, this::doStopStateAction);
      factory.getFactory().addTransition(STOP, NAVIGATE, this::transitionFromStop);
      factory.setOnEntry(NAVIGATE, this::onNavigateStateEntry);
      factory.setDoAction(NAVIGATE, this::doNavigateStateAction);
      factory.addTransition(NAVIGATE, Lists.newArrayList(PLAN, NAVIGATE, STOP), this::transitionFromNavigate);
      factory.setOnEntry(PLAN, this::onPlanStateEntry);
      factory.setDoAction(PLAN, this::doPlanStateAction);
      factory.addTransition(PLAN, Lists.newArrayList(REVIEW, NAVIGATE, STOP), this::transitionFromPlan);
      factory.setOnEntry(REVIEW, this::onReviewStateEntry);
      factory.setDoAction(REVIEW, this::onReviewStateAction);
      factory.addTransition(REVIEW, Lists.newArrayList(WALK, PLAN, STOP), this::transitionFromReview);
      factory.setOnEntry(WALK, this::onWalkStateEntry);
      factory.setDoAction(WALK, this::doWalkStateAction);
      factory.setOnExit(WALK, this::onWalkStateExit);
      factory.addTransition(WALK, Lists.newArrayList(PERCEIVE, NAVIGATE, STOP), this::transitionFromWalk);
      factory.setOnEntry(PERCEIVE, this::onPerceiveStateEntry);
      factory.setDoAction(PERCEIVE, this::doPerceiveStateAction);
      factory.addTransition(PERCEIVE, Lists.newArrayList(NAVIGATE, STOP), this::transitionFromPerceive);

      factory.getFactory().addStateChangedListener((from, to) ->
      {
         messager.submitMessage(CurrentState, to);
         LogTools.debug("{} -> {}", from == null ? null : from.name(), to == null ? null : to.name());
      });
      factory.getFactory().buildClock(() -> Conversions.nanosecondsToSeconds(System.nanoTime()));
      stateMachine = factory.getFactory().build(STOP);

      reaStateRequestPublisher = new IHMCROS2Publisher<>(ros2Node, REAStateRequestMessage.class, null, LIDARBasedREAModule.ROS2_ID);
      remoteRobotControllerInterface = new RemoteRobotControllerInterface(ros2Node, robotModel);
      remoteFootstepPlannerInterface = new RemoteFootstepPlannerInterface(ros2Node, robotModel, messager);

      waypointManager = WaypointManager.createForModule(messager,
                                                        WaypointsToModule,
                                                        WaypointsToUI,
                                                        GoToWaypoint,
                                                        CurrentWaypointIndexStatus,
                                                        goNotification);

      messager.registerTopicListener(Stop, object -> stopNotification.set());
      messager.registerTopicListener(PlanReviewResult, message ->
      {
         planReviewResult.add(message);
      });
      messager.registerTopicListener(SkipPerceive, object -> skipPerceive.set());

      loop = messager.createInput(Loop, false);
      swingOvers = messager.createInput(SwingOvers, false);
      planReviewEnabled = messager.createInput(PlanReviewEnabled, false);
      upDownExplorationEnabled = messager.createInput(UpDownExplorationEnabled, false);
      perceiveDuration = messager.createInput(PerceiveDuration, RemoteFootstepPlannerInterface.DEFAULT_PERCEIVE_TIME_REQUIRED);
      messager.registerTopicListener(UpDownExplorationEnabled, enabled -> { if (enabled) goNotification.set(); });

      upDownExplorer = new UpDownExplorer(messager, behaviorHelper);
      messager.registerTopicListener(CancelPlanning, object ->
      {
         cancelPlanning.set();
         upDownExplorer.abortPlanning();
      });

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
   }

   private boolean transitionFromStop(double timeInState)
   {
      boolean transition = goNotification.read() && (waypointManager.hasWaypoints() || upDownExplorationEnabled.get());
      if (transition)
      {
         LogTools.debug("STOP -> PLAN");
      }
      return transition;
   }

   private void onNavigateStateEntry()
   {
      if (upDownExplorationEnabled.get()) // find up-down if. setup the waypoint
      {
         upDownExplorer.onNavigateEntry(behaviorHelper.pollHumanoidReferenceFrames());
      }
   }

   private void doNavigateStateAction(double timeInState)
   {
      pollInterrupts();
      upDownExplorer.poll();
   }

   private PatrolBehaviorState transitionFromNavigate(double timeInState)
   {
      if (stopNotification.read())
      {
         return STOP;
      }
      else if (cancelPlanning.read())
      {
         return NAVIGATE;
      }
      else if (!upDownExplorationEnabled.get())
      {
         return PLAN;
      }
      else if (upDownExplorer.shouldTransitionToPlan())
      {
         return PLAN;
      }

      return null;
   }

   private void onPlanStateEntry()
   {
      // update waypoints if UI modified them
      waypointManager.updateToMostRecentData();

      remoteFootstepPlannerInterface.abortPlanning();

      FramePose3DReadOnly midFeetZUpPose = behaviorHelper.quickPollPoseReadOnly(HumanoidReferenceFrames::getMidFeetZUpFrame);

      if (upDownExplorationEnabled.get()) // TODO need this?? && upDownExplorer.getUpDownSearchNotification().hasNext())
      {
         upDownExplorer.onPlanEntry(midFeetZUpPose, waypointManager);
      }

      PlanarRegionsListMessage latestPlanarRegionList = behaviorHelper.getLatestPlanarRegionListMessage();

      footstepPlanResultNotification
            = remoteFootstepPlannerInterface.requestPlan(midFeetZUpPose,
                                                         new FramePose3D(waypointManager.peekNextPose()),
                                                         latestPlanarRegionList);
   }

   private void doPlanStateAction(double timeInState)
   {
      pollInterrupts();
      footstepPlanResultNotification.poll();

      if (footstepPlanResultNotification.hasNext())
      {
         upDownExplorer.onPlanFinished(footstepPlanResultNotification.peek());
      }
   }

   private PatrolBehaviorState transitionFromPlan(double timeInState)
   {
      if (stopNotification.read())
      {
         return STOP;
      }
      else if (cancelPlanning.read())
      {
         return NAVIGATE;
      }
      else if (footstepPlanResultNotification.hasNext())
      {
         if (footstepPlanResultNotification.peek().isValidForExecution())
         {
            return REVIEW;
         }
         else
         {
            return NAVIGATE;
         }
      }

      return null;
   }

   private void onReviewStateEntry()
   {
      reduceAndSendFootstepsForVisualization(footstepPlanResultNotification.peek().getFootstepPlan());
   }

   private void onReviewStateAction(double timeInState)
   {
      pollInterrupts();
   }

   private PatrolBehaviorState transitionFromReview(double timeInState)
   {
      if (stopNotification.read())
      {
         return STOP;
      }
      else if (!planReviewEnabled.get()) // if review is disabled, proceed
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
      walkingCompleted = remoteRobotControllerInterface.requestWalk(footstepPlanResultNotification.peek().getFootstepDataListMessage(),
                                                                    behaviorHelper.pollHumanoidReferenceFrames(),
                                                                    swingOvers.get(),
                                                                    footstepPlanResultNotification.peek().getPlanarRegionsList());
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
         if (!upDownExplorationEnabled.get() && !loop.get() && waypointManager.incrementingWillLoop()) // stop in the acute case of not exploring and looping disabled
         {                                                                                      // and it's going to loop
            return STOP;
         }
         else
         {
            // next waypoint is far, gather more data to increase robustness
            if (upDownExplorationEnabled.get() // perceive everytime when updownenabled
             || remoteFootstepPlannerInterface.decidePlanType(
                   behaviorHelper.quickPollPoseReadOnly(HumanoidReferenceFrames::getMidFeetZUpFrame),
                   waypointManager.peekAfterNextPose())
                == PlanTravelDistance.FAR)
            {
               return PERCEIVE;
            }
            else // next waypoint is close
            {
               return NAVIGATE;
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

   private void doPerceiveStateAction(double timeInState)
   {
      pollInterrupts();
      upDownExplorer.setMidFeetZUpPose(behaviorHelper.quickPollPoseReadOnly(HumanoidReferenceFrames::getMidFeetZUpFrame));
   }

   private PatrolBehaviorState transitionFromPerceive(double timeInState)
   {
      if (stopNotification.read())
      {
         return STOP;
      }
      else if (skipPerceive.read())
      {
         return NAVIGATE;
      }
      else if (upDownExplorationEnabled.get() && upDownExplorer.shouldTransitionFromPerceive())
      {
         return NAVIGATE;
      }
      else if (timeInState > perceiveDuration.get())
      {
         return NAVIGATE;
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

      // poll these everytime to throw away ill-timed inputs
      goNotification.poll();
      planReviewResult.poll();
      cancelPlanning.poll();
      skipPerceive.poll();
   }

   private void reduceAndSendFootstepsForVisualization(FootstepPlan footstepPlan)
   {
      ArrayList<Pair<RobotSide, Pose3D>> footstepLocations = new ArrayList<>();
      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)  // this code makes the message smaller to send over the network, TODO investigate
      {
         FramePose3D soleFramePoseToPack = new FramePose3D();
         footstepPlan.getFootstep(i).getSoleFramePose(soleFramePoseToPack);
         footstepLocations.add(new MutablePair<>(footstepPlan.getFootstep(i).getRobotSide(), new Pose3D(soleFramePoseToPack)));
      }
      messager.submitMessage(CurrentFootstepPlan, footstepLocations);
   }
}
