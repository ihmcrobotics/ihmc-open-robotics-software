package us.ihmc.aware.controller.common;

import us.ihmc.aware.controller.force.taskSpaceController.*;
import us.ihmc.aware.params.ParameterMap;
import us.ihmc.aware.params.ParameterMapRepository;
import us.ihmc.aware.planning.ThreeDoFSwingFootTrajectory;
import us.ihmc.aware.state.StateMachine;
import us.ihmc.aware.state.StateMachineBuilder;
import us.ihmc.aware.state.StateMachineState;
import us.ihmc.aware.util.ContactState;
import us.ihmc.aware.util.PreallocatedQueue;
import us.ihmc.aware.util.QuadrupedTimedStep;
import us.ihmc.aware.util.TimeInterval;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedTimedStepController
{
   private static int STEP_QUEUE_CAPACITY = 60;
   private final static String SWING_POSITION_PROPORTIONAL_GAINS = "swingPositionProportionalGains";
   private final static String SWING_POSITION_DERIVATIVE_GAINS = "swingPositionDerivativeGains";
   private final static String SWING_POSITION_INTEGRAL_GAINS = "swingPositionIntegralGains";
   private final static String SWING_POSITION_MAX_INTEGRAL_ERROR = "swingPositionMaxIntegralError";
   private final static String SWING_TRAJECTORY_GROUND_CLEARANCE = "swingTrajectoryGroundClearance";
   private final static String SWING_PRESSURE_LIMIT = "swingPressureLimit";
   private final static String SUPPORT_PRESSURE_LIMIT = "supportPressureLimit";

   public enum StepState
   {
      SUPPORT, SWING
   }
   public enum StepEvent
   {
      LIFT_OFF, TOUCH_DOWN
   }
   private final ParameterMap params;
   private final DoubleYoVariable timestamp;
   private final PreallocatedQueue<QuadrupedTimedStep> stepQueue;
   private final QuadrantDependentList<QuadrupedTimedStep> stepCache;
   private final QuadrantDependentList<StateMachine<StepState, StepEvent>> stepStateMachine;
   private QuadrupedTaskSpaceEstimates taskSpaceEstimates;
   private QuadrupedTaskSpaceSetpoints taskSpaceSetpoints;
   private QuadrupedTaskSpaceControllerSettings taskSpaceControllerSettings;

   public QuadrupedTimedStepController(DoubleYoVariable timestamp, ParameterMapRepository parameterMapRepository,YoVariableRegistry registry)
   {
      params = parameterMapRepository.get(QuadrupedTimedStepController.class);
      params.setDefault(SWING_POSITION_PROPORTIONAL_GAINS, 50000, 50000, 100000);
      params.setDefault(SWING_POSITION_DERIVATIVE_GAINS, 500, 500, 500);
      params.setDefault(SWING_POSITION_INTEGRAL_GAINS, 0, 0, 0);
      params.setDefault(SWING_POSITION_MAX_INTEGRAL_ERROR, 0);
      params.setDefault(SWING_TRAJECTORY_GROUND_CLEARANCE, 0.1);
      params.setDefault(SWING_PRESSURE_LIMIT, 75);
      params.setDefault(SUPPORT_PRESSURE_LIMIT, Double.MAX_VALUE);

      this.timestamp = timestamp;
      this.stepQueue = new PreallocatedQueue<>(QuadrupedTimedStep.class, STEP_QUEUE_CAPACITY);
      this.stepCache = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         this.stepCache.set(robotQuadrant, new QuadrupedTimedStep(robotQuadrant));
      }
      stepStateMachine= new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression();
         StateMachineBuilder<StepState, StepEvent> stateMachineBuilder = new StateMachineBuilder<>(StepState.class, prefix + "StepState", registry);
         stateMachineBuilder.addState(StepState.SUPPORT, new SupportState(robotQuadrant));
         stateMachineBuilder.addState(StepState.SWING, new SwingState(robotQuadrant));
         stateMachineBuilder.addTransition(StepEvent.LIFT_OFF, StepState.SUPPORT, StepState.SWING);
         stateMachineBuilder.addTransition(StepEvent.TOUCH_DOWN, StepState.SWING, StepState.SUPPORT);
         stepStateMachine.set(robotQuadrant, stateMachineBuilder.build(StepState.SUPPORT));
      }
   }

   public boolean addStep(QuadrupedTimedStep step)
   {
      for (int i = 0; i < stepQueue.size(); i++)
      {
         if ((step.getRobotQuadrant() == stepQueue.get(i).getRobotQuadrant()) && (step.getTimeInterval().getStartTime() < step.getTimeInterval().getEndTime()))
         {
            return false;
         }
      }
      if ((step.getTimeInterval().getStartTime() < timestamp.getDoubleValue()) && stepQueue.enqueue())
      {
         stepQueue.getTail().set(step);
         return true;
      }
      else
      {
         return false;
      }
   }

   public void removeSteps()
   {
      for (int i = 0; i < stepQueue.size(); i++)
      {
         // keep ongoing steps in the queue
         QuadrupedTimedStep step = stepQueue.getHead();
         if (step.getTimeInterval().getStartTime() < timestamp.getDoubleValue())
         {
            stepQueue.enqueue();
            stepQueue.getTail().set(step);
         }
         // remove future steps from the queue
         stepQueue.dequeue();
      }
   }

   public PreallocatedQueue<QuadrupedTimedStep> getStepQueue()
   {
      return stepQueue;
   }

   public int getStepQueueSize()
   {
      return stepQueue.size();
   }

   private void triggerStepEvents()
   {
      double currentTime = timestamp.getDoubleValue();

      while ((stepQueue.size() > 0) && (stepQueue.getHead().getTimeInterval().getEndTime() < currentTime))
      {
         stepQueue.dequeue();
      }

      for (int i = 0; i < stepQueue.size(); i++)
      {
         QuadrupedTimedStep step = stepQueue.get(i);
         if (step.getTimeInterval().getStartTime() <= currentTime)
         {
            stepStateMachine.get(step.getRobotQuadrant()).trigger(StepEvent.LIFT_OFF);
            stepCache.get(step.getRobotQuadrant()).set(step);
         }
      }
   }

   public void compute(QuadrupedTaskSpaceControllerSettings taskSpaceControllerSettings, QuadrupedTaskSpaceSetpoints taskSpaceSetpoints, QuadrupedTaskSpaceEstimates taskSpaceEstimates)
   {
      this.taskSpaceEstimates = taskSpaceEstimates;
      this.taskSpaceSetpoints = taskSpaceSetpoints;
      this.taskSpaceControllerSettings = taskSpaceControllerSettings;

      triggerStepEvents();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         stepStateMachine.get(robotQuadrant).process();
      }
   }

   private class SupportState implements StateMachineState<StepEvent>
   {
      RobotQuadrant robotQuadrant;

      public SupportState(RobotQuadrant robotQuadrant)
      {
         this.robotQuadrant = robotQuadrant;
      }

      @Override public void onEntry()
      {
         // initialize contact state
         taskSpaceControllerSettings.setContactState(robotQuadrant, ContactState.IN_CONTACT);
         taskSpaceControllerSettings.setPressureUpperLimit(robotQuadrant, params.get(SUPPORT_PRESSURE_LIMIT));

         // disable sole position feedback
         taskSpaceControllerSettings.setSolePositionFeedbackGainsToZero(robotQuadrant);
      }

      @Override public StepEvent process()
      {
         double currentTime = timestamp.getDoubleValue();
         double liftOffTime = stepCache.get(robotQuadrant).getTimeInterval().getStartTime();

         // trigger lift off event
         if (currentTime > liftOffTime)
            return StepEvent.LIFT_OFF;
         else
            return null;
      }

      @Override public void onExit()
      {
      }
   }

   private class SwingState implements StateMachineState<StepEvent>
   {
      RobotQuadrant robotQuadrant;
      private final ThreeDoFSwingFootTrajectory swingTrajectory;

      public SwingState(RobotQuadrant robotQuadrant)
      {
         this.robotQuadrant = robotQuadrant;
         this.swingTrajectory =  new ThreeDoFSwingFootTrajectory();
      }

      @Override public void onEntry()
      {
         // initialize swing foot controller
         TimeInterval timeInterval = stepCache.get(robotQuadrant).getTimeInterval();
         FramePoint goalPosition = stepCache.get(robotQuadrant).getGoalPosition();
         FramePoint solePosition = taskSpaceEstimates.getSolePosition(robotQuadrant);
         solePosition.changeFrame(goalPosition.getReferenceFrame());
         swingTrajectory.initializeTrajectory(solePosition, goalPosition, params.get(SWING_TRAJECTORY_GROUND_CLEARANCE), timeInterval.getDuration());

         // initialize sole position feedback gains
         taskSpaceControllerSettings.setSolePositionFeedbackGains(robotQuadrant,
               params.getVolatileArray(SWING_POSITION_PROPORTIONAL_GAINS),
               params.getVolatileArray(SWING_POSITION_DERIVATIVE_GAINS),
               params.getVolatileArray(SWING_POSITION_INTEGRAL_GAINS),
               params.get(SWING_POSITION_MAX_INTEGRAL_ERROR)
         );

         // initialize contact state
         taskSpaceControllerSettings.setContactState(robotQuadrant, ContactState.NO_CONTACT);
         taskSpaceControllerSettings.setPressureUpperLimit(robotQuadrant, params.get(SWING_PRESSURE_LIMIT));
      }

      @Override public StepEvent process()
      {
         double currentTime = timestamp.getDoubleValue();
         double liftOffTime = stepCache.get(robotQuadrant).getTimeInterval().getStartTime();
         double touchDownTime = stepCache.get(robotQuadrant).getTimeInterval().getEndTime();

         // compute swing trajectory
         swingTrajectory.computeTrajectory(currentTime - liftOffTime);
         swingTrajectory.getPosition(taskSpaceSetpoints.getSolePosition(robotQuadrant));

         // trigger touch down event
         if (currentTime > touchDownTime)
            return StepEvent.TOUCH_DOWN;
         else
            return null;
      }

      @Override public void onExit()
      {
      }
   }
}
