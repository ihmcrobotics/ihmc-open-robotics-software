package us.ihmc.aware.controller.toolbox;

import us.ihmc.aware.params.DoubleArrayParameter;
import us.ihmc.aware.params.DoubleParameter;
import us.ihmc.aware.params.ParameterFactory;
import us.ihmc.aware.planning.ThreeDoFSwingFootTrajectory;
import us.ihmc.aware.state.StateMachine;
import us.ihmc.aware.state.StateMachineBuilder;
import us.ihmc.aware.state.StateMachineState;
import us.ihmc.aware.util.*;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedTimedStepController
{
   // parameters
   private final ParameterFactory parameterFactory = new ParameterFactory(getClass().getName());
   private final DoubleArrayParameter solePositionProportionalGainsParameter = parameterFactory.createDoubleArray("solePositionProportionalGains", 50000, 50000, 100000);
   private final DoubleArrayParameter solePositionDerivativeGainsParameter = parameterFactory.createDoubleArray("solePositionDerivativeGains", 500, 500, 500);
   private final DoubleArrayParameter solePositionIntegralGainsParameter = parameterFactory.createDoubleArray("solePositionIntegralGains", 0, 0, 0);
   private final DoubleParameter solePositionMaxIntegralErrorParameter = parameterFactory.createDouble("solePositionMaxIntegralError", 0);

   // control variables
   private final DoubleYoVariable timestamp;
   private final QuadrupedSolePositionController solePositionController;
   private final QuadrupedSolePositionController.Setpoints solePositionControllerSetpoints;
   private final QuadrantDependentList<FramePoint> solePositionEstimate;
   private final QuadrantDependentList<ContactState> contactState;

   // step queue
   private static int STEP_QUEUE_CAPACITY = 60;
   private final PreallocatedQueue<QuadrupedTimedStep> stepQueue;
   private final QuadrantDependentList<QuadrupedTimedStep> stepCache;

   // state machine
   public enum StepState
   {
      SUPPORT, SWING
   }
   public enum StepEvent
   {
      LIFT_OFF, TOUCH_DOWN
   }
   private final QuadrantDependentList<StateMachine<StepState, StepEvent>> stepStateMachine;

   public QuadrupedTimedStepController(QuadrupedSolePositionController solePositionController, DoubleYoVariable timestamp, YoVariableRegistry registry)
   {
      // control variables
      this.timestamp = timestamp;
      this.solePositionController = solePositionController;
      solePositionControllerSetpoints = new QuadrupedSolePositionController.Setpoints();
      solePositionEstimate = new QuadrantDependentList<>();
      contactState = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePositionEstimate.set(robotQuadrant, new FramePoint());
         contactState.set(robotQuadrant, ContactState.IN_CONTACT);
      }

      // step queue
      stepQueue = new PreallocatedQueue<>(QuadrupedTimedStep.class, STEP_QUEUE_CAPACITY);
      stepCache = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         stepCache.set(robotQuadrant, new QuadrupedTimedStep());
      }

      // state machine
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

   public void reset()
   {
      for (int i = 0; i < stepQueue.size(); i++)
      {
         stepQueue.dequeue();
      }
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         stepStateMachine.get(robotQuadrant).reset();
      }
      solePositionController.reset();
   }

   public void compute(QuadrantDependentList<ContactState> contactState, QuadrantDependentList<FrameVector> soleForceCommand, QuadrantDependentList<FrameVector> stepAdjustmentSetpoint, QuadrupedTaskSpaceEstimator.Estimates taskSpaceEstimates)
   {
      handleStepEvents();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePositionEstimate.get(robotQuadrant).setIncludingFrame(taskSpaceEstimates.getSolePosition(robotQuadrant));
         stepStateMachine.get(robotQuadrant).process();
         solePositionControllerSetpoints.getSolePosition(robotQuadrant).changeFrame(stepAdjustmentSetpoint.get(robotQuadrant).getReferenceFrame());
         solePositionControllerSetpoints.getSolePosition(robotQuadrant).add(stepAdjustmentSetpoint.get(robotQuadrant));
      }

      solePositionController.compute(soleForceCommand, solePositionControllerSetpoints, taskSpaceEstimates);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactState.set(robotQuadrant, this.contactState.get(robotQuadrant));
      }
   }

   private void handleStepEvents()
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

   private class SupportState implements StateMachineState<StepEvent>
   {
      RobotQuadrant robotQuadrant;

      public SupportState(RobotQuadrant robotQuadrant)
      {
         this.robotQuadrant = robotQuadrant;
      }

      @Override public void onEntry()
      {
         // initialize contact state and feedback gains
         contactState.set(robotQuadrant, ContactState.IN_CONTACT);
         solePositionController.getGains(robotQuadrant).reset();
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
         // initialize swing trajectory
         double groundClearance = stepCache.get(robotQuadrant).getGroundClearance();
         TimeInterval timeInterval = stepCache.get(robotQuadrant).getTimeInterval();
         FramePoint goalPosition = stepCache.get(robotQuadrant).getGoalPosition();
         FramePoint solePosition = solePositionEstimate.get(robotQuadrant);
         solePosition.changeFrame(goalPosition.getReferenceFrame());
         swingTrajectory.initializeTrajectory(solePosition, goalPosition, groundClearance, timeInterval.getDuration());

         // initialize contact state and feedback gains
         contactState.set(robotQuadrant, ContactState.NO_CONTACT);
         solePositionController.getGains(robotQuadrant).setProportionalGains(solePositionProportionalGainsParameter.get());
         solePositionController.getGains(robotQuadrant).setIntegralGains(solePositionIntegralGainsParameter.get(), solePositionMaxIntegralErrorParameter.get());
         solePositionController.getGains(robotQuadrant).setDerivativeGains(solePositionDerivativeGainsParameter.get());
      }

      @Override public StepEvent process()
      {
         double currentTime = timestamp.getDoubleValue();
         double liftOffTime = stepCache.get(robotQuadrant).getTimeInterval().getStartTime();
         double touchDownTime = stepCache.get(robotQuadrant).getTimeInterval().getEndTime();

         // compute swing trajectory
         swingTrajectory.computeTrajectory(currentTime - liftOffTime);
         swingTrajectory.getPosition(solePositionControllerSetpoints.getSolePosition(robotQuadrant));

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
