package us.ihmc.quadrupedRobotics.controller.force.toolbox;

import us.ihmc.quadrupedRobotics.params.DoubleArrayParameter;
import us.ihmc.quadrupedRobotics.params.DoubleParameter;
import us.ihmc.quadrupedRobotics.params.ParameterFactory;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.trajectory.ThreeDoFSwingFootTrajectory;
import us.ihmc.quadrupedRobotics.state.FiniteStateMachine;
import us.ihmc.quadrupedRobotics.state.FiniteStateMachineBuilder;
import us.ihmc.quadrupedRobotics.state.FiniteStateMachineState;
import us.ihmc.quadrupedRobotics.util.PreallocatedQueue;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

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
   private final ParameterFactory parameterFactory = new ParameterFactory(getClass());
   private final DoubleArrayParameter solePositionProportionalGainsParameter = parameterFactory.createDoubleArray("solePositionProportionalGains", 20000, 20000, 20000);
   private final DoubleArrayParameter solePositionDerivativeGainsParameter = parameterFactory.createDoubleArray("solePositionDerivativeGains", 200, 200, 200);
   private final DoubleArrayParameter solePositionIntegralGainsParameter = parameterFactory.createDoubleArray("solePositionIntegralGains", 0, 0, 0);
   private final DoubleParameter solePositionMaxIntegralErrorParameter = parameterFactory.createDouble("solePositionMaxIntegralError", 0);
   private final DoubleParameter solePressureUpperLimitParameter = parameterFactory.createDouble("solePressureUpperLimit", 75);
   private final DoubleParameter soleCoefficientOfFrictionParameter = parameterFactory.createDouble("soleCoefficientOfFrictionParameter", 75);
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
   private final QuadrantDependentList<FiniteStateMachine<StepState, StepEvent>> stepStateMachine;

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
      stepStateMachine = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression();
         FiniteStateMachineBuilder<StepState, StepEvent> stateMachineBuilder = new FiniteStateMachineBuilder<>(StepState.class, StepEvent.class, prefix + "StepState", registry);
         stateMachineBuilder.addState(StepState.SUPPORT, new SupportState(robotQuadrant));
         stateMachineBuilder.addState(StepState.SWING, new SwingState(robotQuadrant));
         stateMachineBuilder.addTransition(StepEvent.LIFT_OFF, StepState.SUPPORT, StepState.SWING);
         stateMachineBuilder.addTransition(StepEvent.TOUCH_DOWN, StepState.SWING, StepState.SUPPORT);
         stepStateMachine.set(robotQuadrant, stateMachineBuilder.build(StepState.SUPPORT));
      }
   }

   public boolean addStep(QuadrupedTimedStep timedStep)
   {
      for (int i = 0; i < stepQueue.size(); i++)
      {
         if ((timedStep.getRobotQuadrant() == stepQueue.get(i).getRobotQuadrant()) && (timedStep.getTimeInterval().getStartTime() < stepQueue.get(i).getTimeInterval().getEndTime()))
         {
            return false;
         }
      }
      if ((timestamp.getDoubleValue() <= timedStep.getTimeInterval().getStartTime()) && stepQueue.enqueue())
      {
         stepQueue.getTail().set(timedStep);
         return true;
      }
      else
      {
         return false;
      }
   }

   public void removeSteps()
   {
      int size = stepQueue.size();
      for (int i = 0; i < size; i++)
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

   public PreallocatedQueue<QuadrupedTimedStep> getQueue()
   {
      return stepQueue;
   }

   public int getQueueSize()
   {
      return stepQueue.size();
   }

   public int getQueueCapacity()
   {
      return stepQueue.capacity();
   }
   public QuadrupedTimedStep getCurrentStep(RobotQuadrant robotQuadrant)
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

   public void reset()
   {
      while (stepQueue.size() > 0)
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
      limitSoleForceCommand(soleForceCommand);
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

   private void limitSoleForceCommand(QuadrantDependentList<FrameVector> soleForceCommand)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         double coefficientOfFriction = soleCoefficientOfFrictionParameter.get();
         double pressureLimit = solePressureUpperLimitParameter.get();
         FrameVector soleForce = soleForceCommand.get(robotQuadrant);
         soleForce.changeFrame(ReferenceFrame.getWorldFrame());
         if (soleForce.getZ() < -pressureLimit)
         {
            // limit vertical force and project horizontal forces into friction pyramid
            soleForce.setX(Math.min(soleForce.getX(), coefficientOfFriction * pressureLimit));
            soleForce.setX(Math.max(soleForce.getX(), -coefficientOfFriction * pressureLimit));
            soleForce.setY(Math.min(soleForce.getY(), coefficientOfFriction * pressureLimit));
            soleForce.setY(Math.max(soleForce.getY(), -coefficientOfFriction * pressureLimit));
            soleForce.setZ(-pressureLimit);
         }
      }
   }

   private class SupportState implements FiniteStateMachineState<StepEvent>
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

   private class SwingState implements FiniteStateMachineState<StepEvent>
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
         swingTrajectory.initializeTrajectory(solePosition, goalPosition, groundClearance, timeInterval);

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
         swingTrajectory.computeTrajectory(currentTime);
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
