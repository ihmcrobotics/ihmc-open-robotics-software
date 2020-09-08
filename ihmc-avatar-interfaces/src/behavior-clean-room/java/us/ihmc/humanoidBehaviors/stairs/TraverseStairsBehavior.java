package us.ihmc.humanoidBehaviors.stairs;

import us.ihmc.commons.Conversions;
import us.ihmc.humanoidBehaviors.BehaviorDefinition;
import us.ihmc.humanoidBehaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.humanoidBehaviors.tools.RemoteHumanoidRobotInterface;
import us.ihmc.humanoidBehaviors.tools.interfaces.StatusLogger;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

import static us.ihmc.humanoidBehaviors.stairs.TraverseStairsBehaviorAPI.create;

public class TraverseStairsBehavior implements BehaviorInterface
{
   public static final BehaviorDefinition DEFINITION = new BehaviorDefinition("Traverse Stairs", TraverseStairsBehavior::new, create());
   private static final int UPDATE_RATE_MILLIS = 50;

   private final BehaviorHelper helper;
   private final StatusLogger statusLogger;
   private final RemoteHumanoidRobotInterface robotInterface;
   private final TraverseStairsBehaviorParameters parameters = new TraverseStairsBehaviorParameters();

   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor();
   private ScheduledFuture<?> behaviorTask = null;
   private final AtomicBoolean isRunning = new AtomicBoolean(false);

   private final StateMachine<TraverseStairsStateName, State> stateMachine;
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final TraverseStairsSquareUpState squareUpState;
   private final TraverseStairsPauseState pauseState;
   private final TraverseStairsPlanStepsState planStepsState;
   private final TraverseStairsExecuteStepsState executeStepsState;

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

   public TraverseStairsBehavior(BehaviorHelper helper)
   {
      this.helper = helper;

      robotInterface = helper.getOrCreateRobotInterface();
      statusLogger = helper.getOrCreateStatusLogger();

      squareUpState = new TraverseStairsSquareUpState(helper, parameters);
      pauseState = new TraverseStairsPauseState(helper, parameters);
      planStepsState = new TraverseStairsPlanStepsState(helper, parameters);
      executeStepsState = new TraverseStairsExecuteStepsState(helper, parameters);

      stateMachine = buildStateMachine();

      helper.createROS2Callback(TraverseStairsBehaviorAPI.START, s -> setEnabled(true));
      helper.createROS2Callback(TraverseStairsBehaviorAPI.STOP, s -> setEnabled(true));
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

      factory.addDoneTransition(TraverseStairsStateName.EXECUTE_STEPS, TraverseStairsStateName.PAUSE);
      factory.addDoneTransition(TraverseStairsStateName.PAUSE, TraverseStairsStateName.PLAN_STEPS);
      factory.addTransition(TraverseStairsStateName.PLAN_STEPS, TraverseStairsStateName.EXECUTE_STEPS, planStepsState::shouldTransitionToExecute);
      factory.addTransition(TraverseStairsStateName.PLAN_STEPS, TraverseStairsStateName.PAUSE, planStepsState::shouldTransitionBackToPause);
      factory.addDoneTransition(TraverseStairsStateName.EXECUTE_STEPS, TraverseStairsStateName.PAUSE);

      return factory.build(TraverseStairsStateName.SQUARE_UP);
   }

   @Override
   public void setEnabled(boolean enable)
   {
      if (enable)
      {
         if (isRunning.getAndSet(true))
         {
            return;
         }

         stateMachine.resetToInitialState();
         behaviorTask = executorService.scheduleAtFixedRate(stateMachine::doActionAndTransition, 0, UPDATE_RATE_MILLIS, TimeUnit.MILLISECONDS);
      }
      else
      {
         if (!isRunning.getAndSet(false))
         {
            return;
         }

         if (behaviorTask != null)
         {
            // TODO send pause walking command
            behaviorTask.cancel(true);
            behaviorTask = null;
         }
      }
   }
}
