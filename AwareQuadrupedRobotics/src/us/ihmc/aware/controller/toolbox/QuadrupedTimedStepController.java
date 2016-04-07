package us.ihmc.aware.controller.toolbox;

import us.ihmc.aware.planning.ThreeDoFSwingFootTrajectory;
import us.ihmc.aware.state.StateMachine;
import us.ihmc.aware.state.StateMachineBuilder;
import us.ihmc.aware.state.StateMachineState;
import us.ihmc.aware.util.*;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedTimedStepController
{
   private static int STEP_QUEUE_CAPACITY = 60;

   public enum StepState
   {
      SUPPORT, SWING
   }
   public enum StepEvent
   {
      LIFT_OFF, TOUCH_DOWN
   }
   private final DoubleYoVariable timestamp;
   private final QuadrantDependentList<ContactState> contactState;
   private final QuadrantDependentList<FramePoint> solePositionEstimate;
   private final QuadrantDependentList<FramePoint> solePositionSetpoint;
   private final QuadrantDependentList<QuadrupedTimedStep> stepCache;
   private final PreallocatedQueue<QuadrupedTimedStep> stepQueue;
   private final QuadrantDependentList<StateMachine<StepState, StepEvent>> stepStateMachine;

   public QuadrupedTimedStepController(DoubleYoVariable timestamp, YoVariableRegistry registry)
   {
      this.timestamp = timestamp;
      contactState = new QuadrantDependentList<>();
      solePositionEstimate = new QuadrantDependentList<>();
      solePositionSetpoint = new QuadrantDependentList<>();
      stepCache = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactState.set(robotQuadrant, ContactState.IN_CONTACT);
         solePositionEstimate.set(robotQuadrant, new FramePoint());
         solePositionSetpoint.set(robotQuadrant, new FramePoint());
         stepCache.set(robotQuadrant, new QuadrupedTimedStep());
      }
      stepQueue = new PreallocatedQueue<>(QuadrupedTimedStep.class, STEP_QUEUE_CAPACITY);
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
   }

   public void compute(QuadrantDependentList<ContactState> contactState, QuadrantDependentList<FramePoint> solePositionSetpoint, QuadrantDependentList<FramePoint> solePositionEstimate)
   {
      handleStepEvents();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         this.solePositionEstimate.set(robotQuadrant, solePositionSetpoint.get(robotQuadrant));
         stepStateMachine.get(robotQuadrant).process();
         solePositionSetpoint.set(robotQuadrant, this.solePositionSetpoint.get(robotQuadrant));
         contactState.set(robotQuadrant, this.contactState.get(robotQuadrant));
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
         contactState.set(robotQuadrant, ContactState.IN_CONTACT);
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
         double groundClearance = stepCache.get(robotQuadrant).getGroundClearance();
         TimeInterval timeInterval = stepCache.get(robotQuadrant).getTimeInterval();
         FramePoint goalPosition = stepCache.get(robotQuadrant).getGoalPosition();
         FramePoint solePosition = solePositionEstimate.get(robotQuadrant);
         solePosition.changeFrame(goalPosition.getReferenceFrame());
         swingTrajectory.initializeTrajectory(solePosition, goalPosition, groundClearance, timeInterval.getDuration());

         // initialize contact state
         contactState.set(robotQuadrant, ContactState.NO_CONTACT);
      }

      @Override public StepEvent process()
      {
         double currentTime = timestamp.getDoubleValue();
         double liftOffTime = stepCache.get(robotQuadrant).getTimeInterval().getStartTime();
         double touchDownTime = stepCache.get(robotQuadrant).getTimeInterval().getEndTime();

         // compute swing trajectory
         swingTrajectory.computeTrajectory(currentTime - liftOffTime);
         swingTrajectory.getPosition(solePositionSetpoint.get(robotQuadrant));

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
