package us.ihmc.behaviors.stairs;

import controller_msgs.msg.dds.BipedalSupportPlanarRegionParametersMessage;
import perception_msgs.msg.dds.DetectedFiducialPacket;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.fiducialDetectorToolBox.FiducialDetectorToolboxModule;
import us.ihmc.behaviors.tools.BehaviorTools;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.behaviors.behaviorTree.ResettingNode;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.RemoteHumanoidRobotInterface;
import us.ihmc.behaviors.tools.interfaces.StatusLogger;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.tools.Timer;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.behaviors.stairs.TraverseStairsBehaviorAPI.*;

/**
 * This was for Atlas to deal with all the state machine stuff involves in doing
 * stairs autonomously, I think we might be able to simplify it on Nadia.
 * @deprecated Not supported right now. Being kept for reference or revival.
 */
public class TraverseStairsBehavior extends ResettingNode
{
   private static final int UPDATE_RATE_MILLIS = 100;
   public static final int STAIRS_FIDUCIAL_ID = 350;

   private final BehaviorHelper helper;
   private ROS2SyncedRobotModel syncedRobot;
   private final StatusLogger statusLogger;
   private final RemoteHumanoidRobotInterface robotInterface;
   private final TraverseStairsBehaviorParameters parameters = new TraverseStairsBehaviorParameters();

   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor();
   private ScheduledFuture<?> behaviorTask = null;
   private final AtomicBoolean isRunning = new AtomicBoolean(false);

   private final AtomicBoolean hasPublishedCompleted = new AtomicBoolean();
   private final AtomicBoolean behaviorHasCrashed = new AtomicBoolean();
   private final AtomicBoolean operatorReviewEnabled = new AtomicBoolean(true);
   private final AtomicReference<DetectedFiducialPacket> detectedFiducial = new AtomicReference<>();

   private final StateMachine<TraverseStairsStateName, TraverseStairsState> stateMachine;
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final TraverseStairsSquareUpState squareUpState;
   private final TraverseStairsPauseState pauseState;
   private final TraverseStairsPlanStepsState planStepsState;
   private final TraverseStairsExecuteStepsState executeStepsState;
   private TraverseStairsStateName currentState = TraverseStairsStateName.SQUARE_UP;
   private final Timer stairsDetectedTimer = new Timer();
   private TraverseStairsLifecycleStateName currentLifeCycleState;
   private final Pose3D stairsPose = new Pose3D(BehaviorTools.createNaNPose());
   private double distanceToStairs = 0.0;

   public enum TraverseStairsStateName
   {
      /** Initial state, walks toward squared up position at top or bottom */
      SQUARE_UP,
      /** Collects lidar scan */
      PAUSE,
      /** Plans footsteps */
      PLAN_STEPS,
      /** Executes steps, blocks on this state while steps are being taken */
      EXECUTE_STEPS
   }

   public enum TraverseStairsLifecycleStateName
   {
      CRASHED,
      RUNNING,
      NOT_RUNNING,
      COMPLETED,
      AWAITING_APPROVAL,
   }

   public TraverseStairsBehavior(BehaviorHelper helper)
   {
      this.helper = helper;

      robotInterface = helper.getOrCreateRobotInterface();
      statusLogger = helper.getOrCreateStatusLogger();

      squareUpState = new TraverseStairsSquareUpState(helper, parameters);
      pauseState = new TraverseStairsPauseState(helper, parameters);
      planStepsState = new TraverseStairsPlanStepsState(helper, parameters, operatorReviewEnabled);
      executeStepsState = new TraverseStairsExecuteStepsState(helper, parameters, planStepsState::getOutput);

      stateMachine = buildStateMachine();

      helper.subscribeViaCallback(START, this::start);
      helper.subscribeViaCallback(STOP, this::stop);
//      helper.subscribeViaCallback(OperatorReviewEnabled, enabled ->
//      {
//         statusLogger.info("Operator review {}", enabled ? "enabled" : "disabled");
//         operatorReviewEnabled.set(enabled);
//      });
      helper.subscribeViaCallback(FiducialDetectorToolboxModule::getDetectedFiducialOutputTopic, detectedFiducialMessage ->
      {
         if (detectedFiducialMessage.getFiducialId() == STAIRS_FIDUCIAL_ID)
         {
            stairsDetectedTimer.reset();
            detectedFiducial.set(detectedFiducialMessage);
            stairsPose.set(detectedFiducialMessage.getFiducialTransformToWorld());
//            helper.publish(DetectedStairsPose, new Pose3D(detectedFiducialMessage.getFiducialTransformToWorld()));
         }
      });
   }

   private StateMachine<TraverseStairsStateName, TraverseStairsState> buildStateMachine()
   {
      StateMachineFactory<TraverseStairsStateName, TraverseStairsState> factory = new StateMachineFactory<>(TraverseStairsStateName.class);

      factory.setNamePrefix("traverseStairs");
      factory.setRegistry(registry);
      factory.buildClock(() -> Conversions.millisecondsToSeconds(System.currentTimeMillis()));

      factory.addState(TraverseStairsStateName.SQUARE_UP, squareUpState);
      factory.addState(TraverseStairsStateName.PAUSE, pauseState);
      factory.addState(TraverseStairsStateName.PLAN_STEPS, planStepsState);
      factory.addState(TraverseStairsStateName.EXECUTE_STEPS, executeStepsState);

      factory.addDoneTransition(TraverseStairsStateName.SQUARE_UP, TraverseStairsStateName.PAUSE);
      factory.addDoneTransition(TraverseStairsStateName.PAUSE, TraverseStairsStateName.PLAN_STEPS);
      factory.addTransition(TraverseStairsStateName.PLAN_STEPS, TraverseStairsStateName.EXECUTE_STEPS, planStepsState::shouldTransitionToExecute);
      factory.addTransition(TraverseStairsStateName.PLAN_STEPS, TraverseStairsStateName.PAUSE, planStepsState::shouldTransitionBackToPause);
      factory.addDoneTransition(TraverseStairsStateName.EXECUTE_STEPS, TraverseStairsStateName.PAUSE);
      factory.addStateChangedListener((from, to) ->
      {
         currentState = to;
//         helper.publish(TraverseStairsBehaviorAPI.State, to.name());
      });

      factory.getRegisteredStates().forEach(state -> factory.addStateChangedListener((from, to) -> state.setPreviousStateName(from)));

      return factory.build(TraverseStairsStateName.SQUARE_UP);
   }

   public void setSyncedRobot(ROS2SyncedRobotModel syncedRobot)
   {
      this.syncedRobot = syncedRobot;
   }

   @Override
   public void clock()
   {
      FramePose3DReadOnly robotPose = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getMidFeetUnderPelvisFrame);
      distanceToStairs = stairsPose.getPosition().distance(robotPose.getPosition());
//      helper.publish(DistanceToStairs, distanceToStairs);

      super.clock();
   }

   @Override
   public BehaviorTreeNodeStatus determineStatus()
   {
      start();

      if (behaviorHasCrashed.get())
      {
         currentLifeCycleState = TraverseStairsLifecycleStateName.CRASHED;
      }
      else if (executeStepsState.planEndsAtGoal() && executeStepsState.walkingIsComplete())
      {
         currentLifeCycleState = TraverseStairsLifecycleStateName.COMPLETED;
      }
      else if (currentState == TraverseStairsStateName.PLAN_STEPS && !planStepsState.isStillPlanning() && planStepsState.searchWasSuccessful())
      {
         currentLifeCycleState = TraverseStairsLifecycleStateName.AWAITING_APPROVAL;
      }
      else if (isRunning.get())
      {
         currentLifeCycleState = TraverseStairsLifecycleStateName.RUNNING;
      }
      else
      {
         currentLifeCycleState = TraverseStairsLifecycleStateName.NOT_RUNNING;
      }
//      helper.publish(LifecycleState, currentLifeCycleState.name());

      return BehaviorTreeNodeStatus.SUCCESS;
   }

   @Override
   public void reset()
   {
      stop();
   }

   private void stop()
   {
      if (!isRunning.getAndSet(false))
      {
         return;
      }
//      helper.publish(LifecycleState, TraverseStairsLifecycleStateName.NOT_RUNNING.name());

      if (behaviorTask != null)
      {
         // TODO send pause walking command
         behaviorTask.cancel(true);
         behaviorTask = null;
      }

//      BipedalSupportPlanarRegionParametersMessage supportRegionParametersMessage = new BipedalSupportPlanarRegionParametersMessage();
//      supportRegionParametersMessage.setEnable(true);
//      supportRegionParametersMessage.setSupportRegionScaleFactor(2.0);
//      supportRegionParametersPublisher.publish(supportRegionParametersMessage);
   }

   private void start()
   {
      if (isRunning.getAndSet(true))
      {
         return;
      }

//      helper.publish(LifecycleState, TraverseStairsLifecycleStateName.RUNNING.name());

      planStepsState.reset();
      pauseState.reset();
      executeStepsState.clearWalkingCompleteFlag();

      hasPublishedCompleted.set(false);
      behaviorHasCrashed.set(false);
      stateMachine.resetToInitialState();
      behaviorTask = executorService.scheduleAtFixedRate(this::update, 0, UPDATE_RATE_MILLIS, TimeUnit.MILLISECONDS);

      BipedalSupportPlanarRegionParametersMessage supportRegionParametersMessage = new BipedalSupportPlanarRegionParametersMessage();
      supportRegionParametersMessage.setEnable(false);
      helper.publish(PerceptionAPI::getBipedalSupportRegionParametersTopic, supportRegionParametersMessage);
   }

   public void update()
   {
      if (behaviorHasCrashed.get())
      {
//         helper.publish(LifecycleState, TraverseStairsLifecycleStateName.CRASHED.name());
         return;
      }

      try
      {
         stateMachine.doActionAndTransition();

         if (executeStepsState.planEndsAtGoal() && executeStepsState.walkingIsComplete() && !hasPublishedCompleted.get())
         {
            helper.publish(TraverseStairsBehaviorAPI.COMPLETED);
            hasPublishedCompleted.set(true);
         }
      }
      catch (Exception e)
      {
         e.printStackTrace();
         behaviorHasCrashed.set(true);
//         helper.publish(LifecycleState, TraverseStairsLifecycleStateName.CRASHED.name());
      }
   }

   public boolean hasSeenStairsecently()
   {
      return stairsDetectedTimer.isRunning(5.0);
   }

   public double getDistanceToStairs()
   {
      return distanceToStairs;
   }

   public boolean isGoing()
   {
      return currentLifeCycleState == TraverseStairsLifecycleStateName.RUNNING || currentLifeCycleState == TraverseStairsLifecycleStateName.AWAITING_APPROVAL;
   }

   public String getName()
   {
      return "Traverse Stairs";
   }

   public void destroy()
   {
      planStepsState.destroy();
   }
}
