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
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.Set;

public class QuadrupedTimedStepController
{
   public static class Setpoints
   {
      QuadrantDependentList<FrameVector> stepAdjustment = new QuadrantDependentList<>();

      public Setpoints()
      {
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            stepAdjustment.set(robotQuadrant, new FrameVector());
         }
      }

      public void initialize(QuadrupedTaskSpaceEstimator.Estimates estimates)
      {
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            stepAdjustment.get(robotQuadrant).setToZero(ReferenceFrame.getWorldFrame());
         }
      }

      public FrameVector getStepAdjustment(RobotQuadrant robotQuadrant)
      {
         return stepAdjustment.get(robotQuadrant);
      }
   }

   // parameters
   private final ParameterFactory parameterFactory = new ParameterFactory(getClass().getName());
   private final DoubleArrayParameter solePositionProportionalGainsParameter = parameterFactory.createDoubleArray("solePositionProportionalGains", 50000, 50000, 100000);
   private final DoubleArrayParameter solePositionDerivativeGainsParameter = parameterFactory.createDoubleArray("solePositionDerivativeGains", 500, 500, 500);
   private final DoubleArrayParameter solePositionIntegralGainsParameter = parameterFactory.createDoubleArray("solePositionIntegralGains", 0, 0, 0);
   private final DoubleParameter solePositionMaxIntegralErrorParameter = parameterFactory.createDouble("solePositionMaxIntegralError", 0);
   private final DoubleParameter stepAdjustmentEnvelopeAttackParameter = parameterFactory.createDouble("stepAdjustmentEnvelopeAttack", 0.5);

   // control variables
   private final DoubleYoVariable timestamp;
   private final QuadrupedSolePositionController solePositionController;
   private final QuadrupedSolePositionController.Setpoints solePositionControllerSetpoints;
   private final QuadrantDependentList<ContactState> contactState;
   private final QuadrantDependentList<FramePoint> solePositionEstimate;
   private final QuadrantDependentList<FrameVector> stepAdjustmentSetpoint;

   // step queue
   private final PreallocatedQueue<QuadrupedTimedStep> stepQueue;

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
      contactState = new QuadrantDependentList<>();
      solePositionEstimate = new QuadrantDependentList<>();
      stepAdjustmentSetpoint = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactState.set(robotQuadrant, ContactState.IN_CONTACT);
         solePositionEstimate.set(robotQuadrant, new FramePoint());
         stepAdjustmentSetpoint.set(robotQuadrant, new FrameVector());
      }

      // step queue
      stepQueue = new PreallocatedQueue<>(QuadrupedTimedStep.class, 100);

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

   public void compute(QuadrantDependentList<ContactState> contactState, QuadrantDependentList<FrameVector> soleForceCommand, Setpoints setpoints, QuadrupedTaskSpaceEstimator.Estimates estimates)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         stepAdjustmentSetpoint.get(robotQuadrant).setIncludingFrame(setpoints.getStepAdjustment(robotQuadrant));
         solePositionEstimate.get(robotQuadrant).setIncludingFrame(estimates.getSolePosition(robotQuadrant));
         stepStateMachine.get(robotQuadrant).process();
         contactState.set(robotQuadrant, this.contactState.get(robotQuadrant));
      }
      solePositionController.compute(soleForceCommand, solePositionControllerSetpoints, estimates);
      handleStepEvents();
   }

   private void handleStepEvents()
   {
      double currentTime = timestamp.getDoubleValue();

      while ((stepQueue.size() > 0) && (currentTime > stepQueue.getHead().getTimeInterval().getEndTime()))
      {
         stepQueue.dequeue();
      }

      for (int i = 0; i < stepQueue.size(); i++)
      {
         QuadrupedTimedStep step = stepQueue.get(i);
         if (step.getTimeInterval().getStartTime() <= currentTime)
         {
            stepStateMachine.get(step.getRobotQuadrant()).trigger(StepEvent.LIFT_OFF);
         }
      }
   }

   private QuadrupedTimedStep getCurrentStep(RobotQuadrant robotQuadrant)
   {
      for (int i = 0; i < stepQueue.size(); i++)
      {
         QuadrupedTimedStep step = stepQueue.get(i);
         if (step.getRobotQuadrant() == robotQuadrant)
         {
            return step;
         }
      }
      return null;
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
         QuadrupedTimedStep timedStep = getCurrentStep(robotQuadrant);
         if (timedStep != null)
         {
            double currentTime = timestamp.getDoubleValue();
            double liftOffTime = timedStep.getTimeInterval().getStartTime();

            // trigger lift off event
            if (currentTime > liftOffTime)
            {
               return StepEvent.LIFT_OFF;
            }
         }
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
         QuadrupedTimedStep timedStep = getCurrentStep(robotQuadrant);
         double groundClearance = timedStep.getGroundClearance();
         TimeInterval timeInterval = timedStep.getTimeInterval();
         FramePoint goalPosition = timedStep.getGoalPosition();
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
         QuadrupedTimedStep timedStep = getCurrentStep(robotQuadrant);
         double currentTime = timestamp.getDoubleValue();
         double liftOffTime = timedStep.getTimeInterval().getStartTime();
         double touchDownTime = timedStep.getTimeInterval().getEndTime();

         // compute swing trajectory
         swingTrajectory.computeTrajectory(currentTime - liftOffTime);
         swingTrajectory.getPosition(solePositionControllerSetpoints.getSolePosition(robotQuadrant));

         // compute step adjustment envelope as a function of normalized step time
         double stepTime = (currentTime - liftOffTime) / (touchDownTime - liftOffTime);
         double envelope = Math.min(stepTime / Math.max(stepAdjustmentEnvelopeAttackParameter.get(), 0.001), 1.0);
         stepAdjustmentSetpoint.get(robotQuadrant).scale(envelope);
         solePositionControllerSetpoints.getSolePosition(robotQuadrant).changeFrame(stepAdjustmentSetpoint.get(robotQuadrant).getReferenceFrame());
         solePositionControllerSetpoints.getSolePosition(robotQuadrant).add(stepAdjustmentSetpoint.get(robotQuadrant));

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
