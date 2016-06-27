package us.ihmc.quadrupedRobotics.controller.force.toolbox;

import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
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
import us.ihmc.quadrupedRobotics.util.PreallocatedQueueSorter;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.BagOfBalls;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

import java.util.Comparator;

public class QuadrupedTimedStepController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   // parameters
   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final DoubleArrayParameter solePositionProportionalGainsParameter = parameterFactory
         .createDoubleArray("solePositionProportionalGains", 10000, 10000, 5000);
   private final DoubleArrayParameter solePositionDerivativeGainsParameter = parameterFactory.createDoubleArray("solePositionDerivativeGains", 200, 200, 200);
   private final DoubleArrayParameter solePositionIntegralGainsParameter = parameterFactory.createDoubleArray("solePositionIntegralGains", 0, 0, 0);
   private final DoubleParameter solePositionMaxIntegralErrorParameter = parameterFactory.createDouble("solePositionMaxIntegralError", 0);
   private final DoubleParameter solePressureUpperLimitParameter = parameterFactory.createDouble("solePressureUpperLimit", 50);
   private final DoubleParameter soleCoefficientOfFrictionParameter = parameterFactory.createDouble("soleCoefficientOfFriction", 75);
   private final DoubleParameter minimumStepAdjustmentTimeParameter = parameterFactory.createDouble("minimumStepAdjustmentTime", 0.1);

   // control variables
   private final DoubleYoVariable timestamp;
   private final QuadrupedSolePositionController solePositionController;
   private final QuadrupedSolePositionController.Setpoints solePositionControllerSetpoints;
   private final PreallocatedQueue<QuadrupedTimedStep> stepQueue;
   private final QuadrantDependentList<ContactState> contactState;
   private final QuadrantDependentList<FramePoint> solePositionEstimate;

   // graphics
   private final BagOfBalls stepQueueVisualization;
   private final FramePoint stepQueueVisualizationPosition;
   private static final QuadrantDependentList<AppearanceDefinition> stepQueueAppearance = new QuadrantDependentList<>(YoAppearance.Red(), YoAppearance.Blue(),
         YoAppearance.RGBColor(1, 0.5, 0.0), YoAppearance.RGBColor(0.0, 0.5, 1.0));

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
   private QuadrupedTimedStepTransitionCallback stepTransitionCallback;

   public QuadrupedTimedStepController(QuadrupedSolePositionController solePositionController, DoubleYoVariable timestamp, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry graphicsListRegistry)
   {
      // control variables
      this.timestamp = timestamp;
      this.solePositionController = solePositionController;
      solePositionControllerSetpoints = new QuadrupedSolePositionController.Setpoints();
      stepQueue = new PreallocatedQueue<>(QuadrupedTimedStep.class, 100);
      contactState = new QuadrantDependentList<>();
      solePositionEstimate = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactState.set(robotQuadrant, ContactState.IN_CONTACT);
         solePositionEstimate.set(robotQuadrant, new FramePoint());
      }

      // state machine
      stepStateMachine = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = robotQuadrant.getCamelCaseName();
         FiniteStateMachineBuilder<StepState, StepEvent> stateMachineBuilder = new FiniteStateMachineBuilder<>(StepState.class, StepEvent.class,
               prefix + "StepState", registry);
         stateMachineBuilder.addState(StepState.SUPPORT, new SupportState(robotQuadrant));
         stateMachineBuilder.addState(StepState.SWING, new SwingState(robotQuadrant));
         stateMachineBuilder.addTransition(StepEvent.LIFT_OFF, StepState.SUPPORT, StepState.SWING);
         stateMachineBuilder.addTransition(StepEvent.TOUCH_DOWN, StepState.SWING, StepState.SUPPORT);
         stepStateMachine.set(robotQuadrant, stateMachineBuilder.build(StepState.SUPPORT));
      }
      stepTransitionCallback = null;

      // graphics
      stepQueueVisualization = BagOfBalls.createRainbowBag(stepQueue.capacity(), 0.015, "xGaitSteps", registry, graphicsListRegistry);
      stepQueueVisualizationPosition = new FramePoint();
      parentRegistry.addChild(registry);
   }

   public void registerStepTransitionCallback(QuadrupedTimedStepTransitionCallback stepTransitionCallback)
   {
      this.stepTransitionCallback = stepTransitionCallback;
   }

   public boolean addStep(QuadrupedTimedStep timedStep)
   {
      for (int i = 0; i < stepQueue.size(); i++)
      {
         if (timedStep.getRobotQuadrant() == stepQueue.get(i).getRobotQuadrant())
         {
            if (timedStep.getTimeInterval().getStartTime() < stepQueue.get(i).getTimeInterval().getEndTime())
               return false;
         }
      }
      if ((timestamp.getDoubleValue() <= timedStep.getTimeInterval().getStartTime()) && stepQueue.enqueue())
      {
         stepQueue.getTail().set(timedStep);
         PreallocatedQueueSorter.sort(stepQueue, compareByEndTime);
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

   public QuadrupedTimedStep getLatestStep(RobotEnd robotEnd)
   {
      for (int i = 0; i < stepQueue.size(); i++)
      {
         QuadrupedTimedStep step = stepQueue.get(i);
         if (step.getRobotQuadrant().getEnd() == robotEnd)
         {
            return step;
         }
      }
      return null;
   }

   public QuadrupedTimedStep getLatestStep(RobotQuadrant robotQuadrant)
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

   public void compute(QuadrantDependentList<ContactState> contactState, QuadrantDependentList<FrameVector> soleForceCommand,
         QuadrupedTaskSpaceEstimator.Estimates estimates)
   {
      // compute sole forces and contact state
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePositionEstimate.get(robotQuadrant).setIncludingFrame(estimates.getSolePosition(robotQuadrant));
         stepStateMachine.get(robotQuadrant).process();
         contactState.set(robotQuadrant, this.contactState.get(robotQuadrant));
      }
      solePositionController.compute(soleForceCommand, solePositionControllerSetpoints, estimates);
      limitSoleForceCommand(soleForceCommand);

      // dequeue completed steps
      double currentTime = timestamp.getDoubleValue();
      while ((stepQueue.size() > 0) && (currentTime > stepQueue.getHead().getTimeInterval().getEndTime()))
      {
         stepQueue.dequeue();
      }
      updateGraphics();
   }

   private void updateGraphics()
   {
      for (int i = 0; i < stepQueue.size(); i++)
      {
         stepQueue.get(i).getGoalPosition(stepQueueVisualizationPosition);
         stepQueueVisualization.setBallLoop(stepQueueVisualizationPosition, stepQueueAppearance.get(stepQueue.get(i).getRobotQuadrant()));
      }
      stepQueueVisualizationPosition.setToZero();
      for (int i = stepQueue.size(); i < stepQueue.capacity(); i++)
      {
         stepQueueVisualization.setBallLoop(stepQueueVisualizationPosition);
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

   private Comparator<QuadrupedTimedStep> compareByEndTime = new Comparator<QuadrupedTimedStep>()
   {
      @Override
      public int compare(QuadrupedTimedStep a, QuadrupedTimedStep b)
      {
         return Double.compare(a.getTimeInterval().getEndTime(), b.getTimeInterval().getEndTime());
      }
   };

   private class SupportState implements FiniteStateMachineState<StepEvent>
   {
      RobotQuadrant robotQuadrant;

      public SupportState(RobotQuadrant robotQuadrant)
      {
         this.robotQuadrant = robotQuadrant;
      }

      @Override
      public void onEntry()
      {
         // initialize contact state and feedback gains
         contactState.set(robotQuadrant, ContactState.IN_CONTACT);
         solePositionController.getGains(robotQuadrant).reset();
      }

      @Override
      public StepEvent process()
      {
         QuadrupedTimedStep timedStep = getLatestStep(robotQuadrant);
         if (timedStep != null)
         {
            double currentTime = timestamp.getDoubleValue();
            double liftOffTime = timedStep.getTimeInterval().getStartTime();
            double touchDownTime = timedStep.getTimeInterval().getEndTime();

            // trigger lift off event
            if (currentTime >= liftOffTime && currentTime < touchDownTime)
            {
               if (stepTransitionCallback != null)
               {
                  stepTransitionCallback.onLiftOff(robotQuadrant, contactState);
               }
               contactState.set(robotQuadrant, ContactState.NO_CONTACT);
               return StepEvent.LIFT_OFF;
            }
         }
         return null;
      }

      @Override
      public void onExit()
      {
      }
   }

   private class SwingState implements FiniteStateMachineState<StepEvent>
   {
      RobotQuadrant robotQuadrant;
      private final ThreeDoFSwingFootTrajectory swingTrajectory;
      private final FramePoint goalPosition;

      public SwingState(RobotQuadrant robotQuadrant)
      {
         this.robotQuadrant = robotQuadrant;
         this.goalPosition = new FramePoint();
         this.swingTrajectory = new ThreeDoFSwingFootTrajectory();
      }

      @Override
      public void onEntry()
      {
         // initialize swing trajectory
         QuadrupedTimedStep timedStep = getLatestStep(robotQuadrant);
         double groundClearance = timedStep.getGroundClearance();
         TimeInterval timeInterval = timedStep.getTimeInterval();
         timedStep.getGoalPosition(goalPosition);
         FramePoint solePosition = solePositionEstimate.get(robotQuadrant);
         solePosition.changeFrame(goalPosition.getReferenceFrame());
         swingTrajectory.initializeTrajectory(solePosition, goalPosition, groundClearance, timeInterval);

         // initialize contact state and feedback gains
         solePositionController.getGains(robotQuadrant).setProportionalGains(solePositionProportionalGainsParameter.get());
         solePositionController.getGains(robotQuadrant).setIntegralGains(solePositionIntegralGainsParameter.get(), solePositionMaxIntegralErrorParameter.get());
         solePositionController.getGains(robotQuadrant).setDerivativeGains(solePositionDerivativeGainsParameter.get());
      }

      @Override
      public StepEvent process()
      {
         QuadrupedTimedStep timedStep = getLatestStep(robotQuadrant);
         double currentTime = timestamp.getDoubleValue();
         double touchDownTime = timedStep.getTimeInterval().getEndTime();

         // current goal position
         timedStep.getGoalPosition(goalPosition);
         goalPosition.changeFrame(ReferenceFrame.getWorldFrame());

         // compute swing trajectory
         if (touchDownTime - currentTime > minimumStepAdjustmentTimeParameter.get())
         {
            swingTrajectory.adjustTrajectory(goalPosition, currentTime);
         }
         swingTrajectory.computeTrajectory(currentTime);
         swingTrajectory.getPosition(solePositionControllerSetpoints.getSolePosition(robotQuadrant));

         // trigger touch down event
         if (currentTime >= touchDownTime)
         {
            if (stepTransitionCallback != null)
            {
               stepTransitionCallback.onTouchDown(robotQuadrant, contactState);
            }
            contactState.set(robotQuadrant, ContactState.IN_CONTACT);
            return StepEvent.TOUCH_DOWN;
         }
         else
            return null;
      }

      @Override
      public void onExit()
      {
      }
   }
}
