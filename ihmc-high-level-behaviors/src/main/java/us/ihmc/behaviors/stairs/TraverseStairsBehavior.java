package us.ihmc.behaviors.stairs;

import controller_msgs.msg.dds.BipedalSupportPlanarRegionParametersMessage;
import std_msgs.msg.dds.Empty;
import us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeControlFlowNode;
import us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.behaviors.tools.behaviorTree.ResettingNode;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.behaviors.BehaviorDefinition;
import us.ihmc.behaviors.BehaviorInterface;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.RemoteHumanoidRobotInterface;
import us.ihmc.behaviors.tools.interfaces.StatusLogger;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

import static us.ihmc.behaviors.stairs.TraverseStairsBehaviorAPI.*;

public class TraverseStairsBehavior extends ResettingNode implements BehaviorInterface
{
   public static final BehaviorDefinition DEFINITION = new BehaviorDefinition("Traverse Stairs", TraverseStairsBehavior::new, create());
   private static final int UPDATE_RATE_MILLIS = 100;

   private final BehaviorHelper helper;
   private final StatusLogger statusLogger;
   private final RemoteHumanoidRobotInterface robotInterface;
   private final TraverseStairsBehaviorParameters parameters = new TraverseStairsBehaviorParameters();

   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor();
   private ScheduledFuture<?> behaviorTask = null;
   private final AtomicBoolean isRunning = new AtomicBoolean(false);

   private final IHMCROS2Publisher<Empty> completedPublisher;
   private final IHMCROS2Publisher<BipedalSupportPlanarRegionParametersMessage> supportRegionParametersPublisher;

   private final AtomicBoolean hasPublishedCompleted = new AtomicBoolean();
   private final AtomicBoolean behaviorHasCrashed = new AtomicBoolean();
   private final AtomicBoolean operatorReviewEnabled = new AtomicBoolean(true);

   private final StateMachine<TraverseStairsStateName, State> stateMachine;
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final TraverseStairsSquareUpState squareUpState;
   private final TraverseStairsPauseState pauseState;
   private final TraverseStairsPlanStepsState planStepsState;
   private final TraverseStairsExecuteStepsState executeStepsState;
   private TraverseStairsStateName currentState = TraverseStairsStateName.SQUARE_UP;

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

      completedPublisher = ROS2Tools.createPublisher(helper.getROS2Node(), TraverseStairsBehaviorAPI.COMPLETED);
      supportRegionParametersPublisher = ROS2Tools.createPublisherTypeNamed(helper.getROS2Node(), BipedalSupportPlanarRegionParametersMessage.class,
                                                                            ROS2Tools.BIPED_SUPPORT_REGION_PUBLISHER
                                                                                  .withRobot(helper.getRobotModel().getSimpleRobotName()).withInput());

      squareUpState = new TraverseStairsSquareUpState(helper, parameters);
      pauseState = new TraverseStairsPauseState(helper, parameters);
      planStepsState = new TraverseStairsPlanStepsState(helper, parameters, operatorReviewEnabled);
      executeStepsState = new TraverseStairsExecuteStepsState(helper, parameters, planStepsState::getOutput);

      stateMachine = buildStateMachine();

      helper.subscribeViaCallback(START, this::start);
      helper.subscribeViaCallback(STOP, this::stop);
      helper.subscribeViaCallback(OperatorReviewEnabled, enabled ->
      {
         statusLogger.info("Operator review {}", enabled ? "enabled" : "disabled");
         operatorReviewEnabled.set(enabled);
      });
   }

   private StateMachine<TraverseStairsStateName, State> buildStateMachine()
   {
      StateMachineFactory<TraverseStairsStateName, State> factory = new StateMachineFactory<>(TraverseStairsStateName.class);

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
         helper.publish(TraverseStairsBehaviorAPI.State, to.name());
      });

      return factory.build(TraverseStairsStateName.SQUARE_UP);
   }

   @Override
   public BehaviorTreeNodeStatus tickInternal()
   {
      start();

      if (behaviorHasCrashed.get())
         helper.publish(LifecycleState, TraverseStairsLifecycleStateName.CRASHED.name());
      else if (executeStepsState.planEndsAtGoal() && executeStepsState.walkingIsComplete())
         helper.publish(LifecycleState, TraverseStairsLifecycleStateName.COMPLETED.name());
      else if (currentState == TraverseStairsStateName.PLAN_STEPS && !planStepsState.isStillPlanning() && planStepsState.searchWasSuccessful())
         helper.publish(LifecycleState, TraverseStairsLifecycleStateName.AWAITING_APPROVAL.name());
      else if (isRunning.get())
         helper.publish(LifecycleState, TraverseStairsLifecycleStateName.RUNNING.name());
      else
         helper.publish(LifecycleState, TraverseStairsLifecycleStateName.NOT_RUNNING.name());

      return BehaviorTreeNodeStatus.SUCCESS;
   }

   @Override
   public void reset()
   {
      stop();
   }

   @Override
   public void setEnabled(boolean enable)
   {
      LogTools.info((enable ? "Enable" : "Disable") + " requested");

      if (enable)
      {
         start();
      }
      else
      {
         stop();
      }
   }

   private void stop()
   {
      if (!isRunning.getAndSet(false))
      {
         return;
      }
      helper.publish(LifecycleState, TraverseStairsLifecycleStateName.NOT_RUNNING.name());

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

      helper.publish(LifecycleState, TraverseStairsLifecycleStateName.RUNNING.name());

      planStepsState.reset();
      executeStepsState.clearWalkingCompleteFlag();

      hasPublishedCompleted.set(false);
      behaviorHasCrashed.set(false);
      stateMachine.resetToInitialState();
      behaviorTask = executorService.scheduleAtFixedRate(this::update, 0, UPDATE_RATE_MILLIS, TimeUnit.MILLISECONDS);

      BipedalSupportPlanarRegionParametersMessage supportRegionParametersMessage = new BipedalSupportPlanarRegionParametersMessage();
      supportRegionParametersMessage.setEnable(false);
      supportRegionParametersPublisher.publish(supportRegionParametersMessage);
   }

   private void update()
   {
      if (behaviorHasCrashed.get())
      {
         helper.publish(LifecycleState, TraverseStairsLifecycleStateName.CRASHED.name());
         return;
      }

      try
      {
         stateMachine.doActionAndTransition();

         if (executeStepsState.planEndsAtGoal() && executeStepsState.walkingIsComplete() && !hasPublishedCompleted.get())
         {
            completedPublisher.publish(new Empty());
            hasPublishedCompleted.set(true);
         }
      }
      catch (Exception e)
      {
         e.printStackTrace();
         behaviorHasCrashed.set(true);
         helper.publish(LifecycleState, TraverseStairsLifecycleStateName.CRASHED.name());
      }
   }

   @Override
   public String getName()
   {
      return DEFINITION.getName();
   }
}
