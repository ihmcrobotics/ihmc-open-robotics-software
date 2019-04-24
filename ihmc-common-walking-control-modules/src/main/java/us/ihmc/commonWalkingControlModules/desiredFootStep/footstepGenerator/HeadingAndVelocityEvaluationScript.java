package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import java.util.Arrays;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.robotics.math.filters.RateLimitedYoFrameVector2d;
import us.ihmc.robotics.math.filters.RateLimitedYoVariable;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachineClock;
import us.ihmc.robotics.taskExecutor.TaskExecutor;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFrameVector2D;

/**
 * Time-based script to generate desired forward, lateral, and turning velocities for the
 * {@link ContinuousStepGenerator}.
 * <p>
 * This class was updated when re-implementing the continuous step generator. There is still quite
 * some work to do here.
 * </p>
 */
public class HeadingAndVelocityEvaluationScript implements Updatable
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoFrameVector2D desiredVelocity = new YoFrameVector2D("scriptedDesiredVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final RateLimitedYoFrameVector2d desiredVelocityRateLimited;
   private final YoDouble desiredTurningVelocity = new YoDouble("scriptedDesiredTurningVelocity", registry);
   private final RateLimitedYoVariable desiredTurningVelocityRateLimited;
   private final YoDouble acceleration = new YoDouble("acceleration", registry);
   private final YoDouble maxVelocity = new YoDouble("maxVelocity", registry);
   private final YoDouble maxTurningVelocity = new YoDouble("maxTurningVelocity", registry);
   private final YoDouble turningAcceleration = new YoDouble("turningAcceleration", registry);
   private final YoDouble cruiseVelocity = new YoDouble("cruiseVelocity", registry);
   private final YoDouble sidestepVelocity = new YoDouble("sidestepVelocity", registry);

   private final YoEnum<EvaluationEvent> currentScriptEvent = new YoEnum<>("currentScriptEvent", registry, EvaluationEvent.class);

   private final Vector2D desiredVelocityDirection = new Vector2D(1.0, 0.0);

   public enum EvaluationEvent
   {
      STEP_IN_PLACE(5.0),
      GO_TO_CRUISE_STRAIGHT(6.0),
      TURN_180_CRUISE(8.0),
      SPEED_UP_TO_MAX_STRAIGHT(4.0),
      SLOW_DOWN_TO_ZERO(4.0),
      SIDE_STEP_LEFT(5.0),
      SIDE_STEP_RIGHT(5.0),
      TURN_IN_PLACE180(8.0),
      DIAGONALLY_RIGHT_45(6.0),
      DIAGONALLY_LEFT_45(6.0),
      WAVE_CRUISE(12.0),
      CHANGE_HEADING_WALKING_STRAIGHT(12.0);

      private final double minTime;

      private EvaluationEvent(double minTime)
      {
         this.minTime = minTime;
      }

      public double getMinEventDuration()
      {
         return minTime;
      }
   }

   private final StateMachineClock clock;
   private final TaskExecutor taskExecutor = new TaskExecutor();
   private final List<EventTask> eventList;

   /**
    * Creates a new script.
    * 
    * @param controlDT the tick duration.
    * @param timeProvider provider to obtain the current time.
    * @param parameters script parameters.
    * @param parentRegistry registry to attach this script {@code YoVariable}s
    */
   public HeadingAndVelocityEvaluationScript(double controlDT, DoubleProvider timeProvider, HeadingAndVelocityEvaluationScriptParameters parameters,
                                             YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);

      if (parameters == null)
         parameters = new HeadingAndVelocityEvaluationScriptParameters();

      clock = StateMachineClock.yoClock(timeProvider, "headingAndVelocity", registry);

      acceleration.set(parameters.getAcceleration());
      maxVelocity.set(parameters.getMaxVelocity());
      maxTurningVelocity.set(10.0 * parameters.getMaxHeadingDot());
      turningAcceleration.set(0.75);
      cruiseVelocity.set(parameters.getCruiseVelocity());
      sidestepVelocity.set(parameters.getSideStepVelocity());

      desiredVelocityRateLimited = RateLimitedYoFrameVector2d.createRateLimitedYoFrameVector2d("scriptDesiredVelocityRateLimited", "", registry, acceleration,
                                                                                               controlDT, desiredVelocity);
      desiredTurningVelocityRateLimited = new RateLimitedYoVariable("scriptDesiredTurningVelocityRateLimited", registry, turningAcceleration,
                                                                    desiredTurningVelocity, controlDT);

      eventList = createCompleteEventList();
   }

   private List<EventTask> createCompleteEventList()
   {
      return Arrays.asList(createStepInPlace(), createGoToCruiseVelocity(), createTurn180Cruise(), createSpeedUpToMaxStraight(), createSlowDownToZero(),
                           createSidestepLeft(), createSlowDownToZero(), createSidestepRight(), createSlowDownToZero(), createTurnInPlace180(),
                           createDiagonallyRight45(), createSlowDownToZero(), createDiagonallyLeft45(), createSlowDownToZero(), createWaveCruise(),
                           createSlowDownToZero(), createTurnInPlace180(), createChangeHeadingWalkingStraight(), createSlowDownToZero());
   }

   public DesiredVelocityProvider getDesiredVelocityProvider()
   {
      return () -> desiredVelocityRateLimited;
   }

   public DesiredTurningVelocityProvider getDesiredTurningVelocityProvider()
   {
      return () -> desiredTurningVelocityRateLimited.getValue();
   }

   @Override
   public void update(double time)
   {
      if (taskExecutor.isDone())
      {
         eventList.forEach(taskExecutor::submit);
      }

      taskExecutor.doControl();
      desiredVelocityRateLimited.update();
      desiredTurningVelocityRateLimited.update();
   }

   private abstract class EventTask implements State
   {
      private final EvaluationEvent evaluationEvent;
      protected final double minEventDuration;

      public EventTask(EvaluationEvent evaluationEvent)
      {
         this.evaluationEvent = evaluationEvent;
         this.minEventDuration = evaluationEvent.getMinEventDuration();
      }

      @Override
      public void onEntry()
      {
         currentScriptEvent.set(evaluationEvent);
         clock.notifyStateChanged();
      }

      public abstract void doAction(double timeInState);

      @Override
      public void onExit()
      {

      }

      @Override
      public boolean isDone(double timeInState)
      {
         return timeInState + 1.0e-7 > minEventDuration;
      }
   }

   private EventTask createConstantVelocityEvent(EvaluationEvent evaluationEvent, Vector2DReadOnly velocityDirection, double velocityMagnitude,
                                                 double turningVelocity)
   {

      return new EventTask(evaluationEvent)
      {
         @Override
         public void onEntry()
         {
            super.onEntry();
            desiredVelocityDirection.setAndNormalize(velocityDirection);
            desiredVelocity.setAndScale(velocityMagnitude, desiredVelocityDirection);
            desiredTurningVelocity.set(turningVelocity);
         }

         @Override
         public void doAction(double timeInState)
         {
         }
      };
   }

   private EventTask createStepInPlace()
   {
      return createConstantVelocityEvent(EvaluationEvent.STEP_IN_PLACE, new Vector2D(1.0, 0.0), 0.0, 0.0);
   }

   private EventTask createSpeedUpToMaxStraight()
   {
      return createConstantVelocityEvent(EvaluationEvent.SPEED_UP_TO_MAX_STRAIGHT, new Vector2D(1.0, 0.0), maxVelocity.getValue(), 0.0);
   }

   private EventTask createGoToCruiseVelocity()
   {
      return createConstantVelocityEvent(EvaluationEvent.GO_TO_CRUISE_STRAIGHT, new Vector2D(1.0, 0.0), cruiseVelocity.getValue(), 0.0);
   }

   private EventTask createTurn180Cruise()
   {
      return createConstantVelocityEvent(EvaluationEvent.TURN_180_CRUISE, new Vector2D(1.0, 0.0), cruiseVelocity.getValue(),
                                         0.4 * maxTurningVelocity.getValue());
   }

   private EventTask createSlowDownToZero()
   {
      return new EventTask(EvaluationEvent.SLOW_DOWN_TO_ZERO)
      {
         private double initialTurningVelocity;

         @Override
         public void onEntry()
         {
            super.onEntry();
            initialTurningVelocity = desiredTurningVelocity.getValue();
         }

         @Override
         public void doAction(double timeInState)
         {
            desiredVelocity.setToZero();
            double alpha = MathTools.clamp(timeInState / minEventDuration, 0.0, 1.0);
            desiredTurningVelocity.set(EuclidCoreTools.interpolate(initialTurningVelocity, 0.0, alpha));
         }
      };
   }

   private EventTask createSidestepLeft()
   {
      return createConstantVelocityEvent(EvaluationEvent.SIDE_STEP_LEFT, new Vector2D(0.0, 1.0), cruiseVelocity.getValue(), 0.0);
   }

   private EventTask createSidestepRight()
   {
      return createConstantVelocityEvent(EvaluationEvent.SIDE_STEP_RIGHT, new Vector2D(0.0, -1.0), cruiseVelocity.getValue(), 0.0);
   }

   private EventTask createTurnInPlace180()
   {
      return createConstantVelocityEvent(EvaluationEvent.TURN_IN_PLACE180, new Vector2D(1.0, 0.0), 0.0, maxTurningVelocity.getValue());
   }

   private EventTask createDiagonallyLeft45()
   {
      return createConstantVelocityEvent(EvaluationEvent.DIAGONALLY_LEFT_45, new Vector2D(1.0, 1.0), sidestepVelocity.getValue(), 0.0);
   }

   private EventTask createDiagonallyRight45()
   {
      return createConstantVelocityEvent(EvaluationEvent.DIAGONALLY_RIGHT_45, new Vector2D(1.0, -1.0), sidestepVelocity.getValue(), 0.0);
   }

   private EventTask createWaveCruise()
   {
      return new EventTask(EvaluationEvent.WAVE_CRUISE)
      {
         double freq = 0.2; // Hz
         double amplitude = Math.PI / 4.0;

         @Override
         public void doAction(double timeInState)
         {
            desiredVelocity.setAndScale(cruiseVelocity.getValue(), new Vector2D(1.0, 0.0));
            desiredTurningVelocity.set(2.0 * Math.PI * freq * amplitude * Math.cos(2.0 * Math.PI * freq * timeInState));
         }
      };
   }

   private EventTask createChangeHeadingWalkingStraight()
   {
      return new EventTask(EvaluationEvent.CHANGE_HEADING_WALKING_STRAIGHT)
      {
         double freq = 0.1; // Hz
         double amplitude = Math.PI / 4.0;

         @Override
         public void doAction(double timeInState)
         {
            desiredVelocity.setAndScale(cruiseVelocity.getValue(), new Vector2D(1.0, 0.0));
            desiredTurningVelocity.set(2.0 * Math.PI * freq * amplitude * Math.cos(2.0 * Math.PI * freq * timeInState));
         }
      };
   }
}
