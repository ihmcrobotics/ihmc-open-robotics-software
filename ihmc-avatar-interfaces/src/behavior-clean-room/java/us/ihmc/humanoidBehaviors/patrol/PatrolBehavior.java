package us.ihmc.humanoidBehaviors.patrol;

import java.util.ArrayList;
import java.util.Optional;
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
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.RemoteFootstepPlannerInterface;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.RemoteFootstepPlannerInterface.PlanType;
import us.ihmc.humanoidBehaviors.tools.RemoteRobotControllerInterface;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedHumanoidFrames;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.RemoteFootstepPlannerResult;
import us.ihmc.humanoidBehaviors.upDownExploration.UpDownExplorer;
import us.ihmc.humanoidBehaviors.upDownExploration.UpDownFlatAreaFinder;
import us.ihmc.humanoidBehaviors.waypoints.Waypoint;
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
   public static final double TIME_TO_PERCEIVE = 20.0;

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

   private final Messager messager;
   private final StateMachine<PatrolBehaviorState, State> stateMachine;

   private final ROS2Input<PlanarRegionsListMessage> planarRegionsList;
   private final IHMCROS2Publisher<REAStateRequestMessage> reaStateRequestPublisher;
   private final RemoteRobotControllerInterface remoteRobotControllerInterface;
   private final RemoteSyncedHumanoidFrames remoteSyncedHumanoidFrames;
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
   private final AtomicReference<Boolean> planReview;
   private final AtomicReference<Boolean> upDownExploration;
   private final AtomicReference<Double> exploreTurnAmount;

   public PatrolBehavior(Messager messager, Ros2Node ros2Node, DRCRobotModel robotModel)
   {
      this.messager = messager;

      LogTools.debug("Initializing patrol behavior");

      EnumBasedStateMachineFactory<PatrolBehaviorState> factory = new EnumBasedStateMachineFactory<>(PatrolBehaviorState.class);
      factory.getStateMap().get(STOP).setOnEntry(this::onStopStateEntry);
      factory.getStateMap().get(STOP).setDoAction(this::doStopStateAction);
      factory.getFactory().addTransition(STOP, NAVIGATE, this::transitionFromStop);
      factory.getStateMap().get(NAVIGATE).setOnEntry(this::onNavigateStateEntry);
      factory.getStateMap().get(NAVIGATE).setDoAction(this::doNavigateStateAction);
      factory.addTransition(NAVIGATE, Lists.newArrayList(PLAN, NAVIGATE, STOP), this::transitionFromNavigate);
      factory.getStateMap().get(PLAN).setOnEntry(this::onPlanStateEntry);
      factory.getStateMap().get(PLAN).setDoAction(this::doPlanStateAction);
      factory.addTransition(PLAN, Lists.newArrayList(REVIEW, NAVIGATE, STOP), this::transitionFromPlan);
      factory.getStateMap().get(REVIEW).setOnEntry(this::onReviewStateEntry);
      factory.getStateMap().get(REVIEW).setDoAction(this::onReviewStateAction);
      factory.addTransition(REVIEW, Lists.newArrayList(WALK, PLAN, STOP), this::transitionFromReview);
      factory.getStateMap().get(WALK).setOnEntry(this::onWalkStateEntry);
      factory.getStateMap().get(WALK).setDoAction(this::doWalkStateAction);
      factory.getStateMap().get(WALK).setOnExit(this::onWalkStateExit);
      factory.addTransition(WALK, Lists.newArrayList(PERCEIVE, NAVIGATE, STOP), this::transitionFromWalk);
      factory.getStateMap().get(PERCEIVE).setOnEntry(this::onPerceiveStateEntry);
      factory.getStateMap().get(PERCEIVE).setDoAction(this::doPerceiveStateAction);
      factory.addTransition(PERCEIVE, Lists.newArrayList(NAVIGATE, STOP), this::transitionFromPerceive);
      factory.getFactory().addStateChangedListener((from, to) ->
                                                   {
                                                      messager.submitMessage(CurrentState, to);
                                                      LogTools.debug("{} -> {}", from == null ? null : from.name(), to == null ? null : to.name());
                                                   });
      factory.getFactory().buildClock(() -> Conversions.nanosecondsToSeconds(System.nanoTime()));
      stateMachine = factory.getFactory().build(STOP);

      planarRegionsList = new ROS2Input<>(ros2Node, PlanarRegionsListMessage.class, null, LIDARBasedREAModule.ROS2_ID);
      reaStateRequestPublisher = new IHMCROS2Publisher<>(ros2Node, REAStateRequestMessage.class, null, LIDARBasedREAModule.ROS2_ID);
      remoteRobotControllerInterface = new RemoteRobotControllerInterface(ros2Node, robotModel);
      remoteSyncedHumanoidFrames = new RemoteSyncedHumanoidFrames(robotModel, ros2Node);
      remoteFootstepPlannerInterface = new RemoteFootstepPlannerInterface(ros2Node, robotModel, messager);
      upDownExplorer = new UpDownExplorer(messager);

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
      messager.registerTopicListener(CancelPlanning, object ->
      {
         cancelPlanning.set();
         upDownExplorer.abortPlanning();
      });
      messager.registerTopicListener(SkipPerceive, object -> skipPerceive.set());

      loop = messager.createInput(Loop, false);
      swingOvers = messager.createInput(SwingOvers, false);
      planReview = messager.createInput(PlanReviewEnabled, false);
      upDownExploration = messager.createInput(UpDownExplorationEnabled, false);
      exploreTurnAmount = messager.createInput(ExplorationTurnAmount, 180.0);
      messager.registerTopicListener(UpDownExplorationEnabled, enabled -> { if (enabled) goNotification.set(); });

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
      boolean transition = goNotification.read() && (waypointManager.hasWaypoints() || upDownExploration.get());
      if (transition)
      {
         LogTools.debug("STOP -> PLAN");
      }
      return transition;
   }

   private void onNavigateStateEntry()
   {
      if (upDownExploration.get()) // find up-down if. setup the waypoint
      {
         upOrDownNotification = upDownFlatAreaFinder.upOrDownOnAThread(remoteSyncedHumanoidFrames.pollHumanoidReferenceFrames().getMidFeetZUpFrame(),
                                                                       PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsList.getLatest()));
      }
   }

   private void doNavigateStateAction(double timeInState)
   {
      pollInterrupts();
      upOrDownNotification.poll();
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
      else if (!upDownExploration.get() || upOrDownNotification.hasNext())
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

      FramePose3DReadOnly midFeetZUpPose = remoteSyncedHumanoidFrames.quickPollPoseReadOnly(HumanoidReferenceFrames::getMidFeetZUpFrame);

      if (upDownExploration.get() && upOrDownNotification.hasNext())
      {
         waypointManager.clearWaypoints();
         Waypoint newWaypoint = waypointManager.appendNewWaypoint();

         if (upOrDownNotification.peek().isPresent()) // success
         {
            newWaypoint.getPose().set(upOrDownNotification.peek().get());
         }
         else // turn counter-clockwise to try and find an up-down
         {
            newWaypoint.getPose().set(midFeetZUpPose);
            newWaypoint.getPose().appendYawRotation(Math.toRadians(exploreTurnAmount.get())); // TODO figure out another algorithm? state machine?
         }

         waypointManager.publish();
         waypointManager.setNextFromIndex(0);
      }

      footstepPlanResultNotification
            = remoteFootstepPlannerInterface.requestPlan(midFeetZUpPose,
                                                         new FramePose3D(waypointManager.peekNextPose()),
                                                         planarRegionsList.getLatest());
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
      walkingCompleted = remoteRobotControllerInterface.requestWalk(footstepPlanResultNotification.peek().getFootstepDataListMessage(),
                                                                    remoteSyncedHumanoidFrames.pollHumanoidReferenceFrames(),
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
         if (!upDownExploration.get() && !loop.get() && waypointManager.incrementingWillLoop()) // stop in the acute case of not exploring and looping disabled
         {                                                                                      // and it's going to loop
            return STOP;
         }
         else
         {
            // next waypoint is far, gather more data to increase robustness
            if (upDownExploration.get() // perceive everytime when updownenabled
             || remoteFootstepPlannerInterface.decidePlanType(
                   remoteSyncedHumanoidFrames.quickPollPoseReadOnly(HumanoidReferenceFrames::getMidFeetZUpFrame),
                   waypointManager.peekAfterNextPose())
                == PlanType.FAR)
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
      else if (timeInState > TIME_TO_PERCEIVE)
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
