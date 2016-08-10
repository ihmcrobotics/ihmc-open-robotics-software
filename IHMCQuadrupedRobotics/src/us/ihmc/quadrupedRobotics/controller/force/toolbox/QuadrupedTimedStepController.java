package us.ihmc.quadrupedRobotics.controller.force.toolbox;

import us.ihmc.commonWalkingControlModules.controlModules.foot.SupportState;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.quadrupedRobotics.optimization.contactForceOptimization.QuadrupedContactForceLimits;
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
import us.ihmc.robotics.MathTools;
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
   private final DoubleParameter touchdownPressureLimitParameter = parameterFactory.createDouble("touchdownPressureLimit", 50);
   private final DoubleParameter contactPressureLowerLimitParameter = parameterFactory.createDouble("contactPressureLowerLimit", 50);
   private final DoubleParameter contactPressureUpperLimitParameter = parameterFactory.createDouble("contactPressureUpperLimit", 1000);
   private final DoubleParameter loadDurationParameter = parameterFactory.createDouble("loadDuration", 0.0);
   private final DoubleParameter unloadDurationParameter = parameterFactory.createDouble("unloadDuration", 0.0);
   private final DoubleParameter soleCoefficientOfFrictionParameter = parameterFactory.createDouble("soleCoefficientOfFriction", 75);
   private final DoubleParameter minimumStepAdjustmentTimeParameter = parameterFactory.createDouble("minimumStepAdjustmentTime", 0.1);
   private final DoubleParameter stepGoalOffsetZParameter = parameterFactory.createDouble("stepGoalOffsetZ", 0.0);

   // control variables
   private final DoubleYoVariable timestamp;
   private final QuadrantDependentList<QuadrupedSolePositionController> solePositionController;
   private final QuadrantDependentList<QuadrupedSolePositionController.Setpoints> solePositionControllerSetpoints;
   private final PreallocatedQueue<QuadrupedTimedStep> stepQueue;
   private final QuadrantDependentList<FramePoint> solePositionEstimate;
   private final QuadrantDependentList<FrameVector> soleForceCommand;
   private final QuadrantDependentList<ContactState> contactState;
   private final QuadrupedContactForceLimits contactForceLimits;
   private final QuadrupedTaskSpaceEstimator.Estimates taskSpaceEstimates;

   // graphics
   private final FramePoint stepQueueVisualizationPosition;
   private final QuadrantDependentList<BagOfBalls> stepQueueVisualization;
   private static final QuadrantDependentList<AppearanceDefinition> stepQueueAppearance = new QuadrantDependentList<>(YoAppearance.Red(), YoAppearance.Blue(),
         YoAppearance.RGBColor(1, 0.5, 0.0), YoAppearance.RGBColor(0.0, 0.5, 1.0));

   // state machine
   public enum StepState
   {
      LOAD, SUPPORT, UNLOAD, SWING
   }

   public enum StepEvent
   {
      TIMEOUT, UNLOAD, LIFT_OFF, TOUCH_DOWN
   }

   private final QuadrantDependentList<FiniteStateMachine<StepState, StepEvent>> stepStateMachine;
   private QuadrupedTimedStepTransitionCallback stepTransitionCallback;

   public QuadrupedTimedStepController(QuadrantDependentList<QuadrupedSolePositionController> solePositionController, DoubleYoVariable timestamp,
         YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      // control variables
      this.timestamp = timestamp;
      this.solePositionController = solePositionController;
      this.solePositionControllerSetpoints = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         this.solePositionControllerSetpoints.set(robotQuadrant, new QuadrupedSolePositionController.Setpoints(robotQuadrant));
      }
      stepQueue = new PreallocatedQueue<>(QuadrupedTimedStep.class, 100);
      contactState = new QuadrantDependentList<>();
      solePositionEstimate = new QuadrantDependentList<>();
      soleForceCommand = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePositionEstimate.set(robotQuadrant, new FramePoint());
         soleForceCommand.set(robotQuadrant, new FrameVector());
         contactState.set(robotQuadrant, ContactState.IN_CONTACT);
      }
      contactForceLimits = new QuadrupedContactForceLimits();
      taskSpaceEstimates = new QuadrupedTaskSpaceEstimator.Estimates();

      // state machine
      stepStateMachine = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = robotQuadrant.getCamelCaseName();
         FiniteStateMachineBuilder<StepState, StepEvent> stateMachineBuilder = new FiniteStateMachineBuilder<>(StepState.class, StepEvent.class,
               prefix + "StepState", registry);
         stateMachineBuilder.addState(StepState.LOAD, new LoadState(robotQuadrant));
         stateMachineBuilder.addState(StepState.SUPPORT, new SupportState(robotQuadrant));
         stateMachineBuilder.addState(StepState.UNLOAD, new UnloadState(robotQuadrant));
         stateMachineBuilder.addState(StepState.SWING, new SwingState(robotQuadrant));
         stateMachineBuilder.addTransition(StepEvent.TIMEOUT, StepState.LOAD, StepState.SUPPORT);
         stateMachineBuilder.addTransition(StepEvent.UNLOAD, StepState.SUPPORT, StepState.UNLOAD);
         stateMachineBuilder.addTransition(StepEvent.LIFT_OFF, StepState.UNLOAD, StepState.SWING);
         stateMachineBuilder.addTransition(StepEvent.TOUCH_DOWN, StepState.SWING, StepState.LOAD);
         stepStateMachine.set(robotQuadrant, stateMachineBuilder.build(StepState.SUPPORT));
      }
      stepTransitionCallback = null;

      // graphics
      stepQueueVisualization = new QuadrantDependentList<>();
      stepQueueVisualizationPosition = new FramePoint();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         AppearanceDefinition appearance = stepQueueAppearance.get(robotQuadrant);
         String prefix = "timedStepController" + robotQuadrant.getPascalCaseName() + "GoalPositions";
         stepQueueVisualization.set(robotQuadrant, new BagOfBalls(stepQueue.capacity(), 0.015, prefix, appearance, registry, graphicsListRegistry));
      }
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

   public QuadrupedTimedStep getCurrentStep(RobotEnd robotEnd)
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
         contactForceLimits.setPressureUpperLimit(robotQuadrant, contactPressureUpperLimitParameter.get());
         contactForceLimits.setPressureLowerLimit(robotQuadrant, contactPressureLowerLimitParameter.get());
      }
   }

   public void compute(QuadrantDependentList<ContactState> contactState, QuadrupedContactForceLimits contactForceLimits,
         QuadrantDependentList<FrameVector> soleForceCommand, QuadrupedTaskSpaceEstimator.Estimates taskSpaceEsimates)
   {
      // copy inputs
      this.taskSpaceEstimates.set(taskSpaceEsimates);

      // compute sole forces and contact state
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePositionEstimate.get(robotQuadrant).setIncludingFrame(taskSpaceEsimates.getSolePosition(robotQuadrant));
         stepStateMachine.get(robotQuadrant).process();
      }

      // dequeue completed steps
      double currentTime = timestamp.getDoubleValue();
      while ((stepQueue.size() > 0) && (currentTime > stepQueue.getHead().getTimeInterval().getEndTime()))
      {
         stepQueue.dequeue();
      }
      updateGraphics();

      // copy outputs
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         soleForceCommand.get(robotQuadrant).set(this.soleForceCommand.get(robotQuadrant));
         contactState.set(robotQuadrant, this.contactState.get(robotQuadrant));
      }
      contactForceLimits.set(this.contactForceLimits);
   }

   private void updateGraphics()
   {
      for (int i = 0; i < stepQueue.capacity(); i++)
      {
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            stepQueueVisualizationPosition.setToZero();
            stepQueueVisualization.get(robotQuadrant).setBall(stepQueueVisualizationPosition, i);
         }
      }
      for (int i = 0; i < stepQueue.size(); i++)
      {
         stepQueue.get(i).getGoalPosition(stepQueueVisualizationPosition);
         RobotQuadrant robotQuadrant = stepQueue.get(i).getRobotQuadrant();
         stepQueueVisualization.get(robotQuadrant).setBallLoop(stepQueueVisualizationPosition);
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

   private class LoadState implements FiniteStateMachineState<StepEvent>
   {
      private double initialTime;
      private RobotQuadrant robotQuadrant;

      public LoadState(RobotQuadrant robotQuadrant)
      {
         this.robotQuadrant = robotQuadrant;
      }

      @Override
      public void onEntry()
      {
         initialTime = timestamp.getDoubleValue();
      }

      @Override
      public StepEvent process()
      {
         double elapsedTime = timestamp.getDoubleValue() - initialTime;
         double loadDuration = Math.max(loadDurationParameter.get(), 0.001);

         if (elapsedTime > loadDuration)
         {
            return StepEvent.TIMEOUT;
         }

         // ramp up contact pressure
         double alpha = MathTools.clipToMinMax(Math.pow(elapsedTime / loadDuration, 2.0), 0.0, 1.0);
         double pressureLowerLimit = alpha * contactPressureLowerLimitParameter.get();
         double pressureUpperLimit = alpha * contactPressureUpperLimitParameter.get();
         contactForceLimits.setPressureLowerLimit(robotQuadrant, pressureLowerLimit);
         contactForceLimits.setPressureUpperLimit(robotQuadrant, pressureUpperLimit);
         return null;
      }

      @Override
      public void onExit()
      {
      }
   }

   private class SupportState implements FiniteStateMachineState<StepEvent>
   {
      private RobotQuadrant robotQuadrant;

      public SupportState(RobotQuadrant robotQuadrant)
      {
         this.robotQuadrant = robotQuadrant;
      }

      @Override
      public void onEntry()
      {
         // reset contact pressure limits
         contactForceLimits.setPressureLowerLimit(robotQuadrant, contactPressureLowerLimitParameter.get());
         contactForceLimits.setPressureUpperLimit(robotQuadrant, contactPressureUpperLimitParameter.get());
      }

      @Override
      public StepEvent process()
      {
         QuadrupedTimedStep timedStep = getCurrentStep(robotQuadrant);
         if (timedStep != null)
         {
            double currentTime = timestamp.getDoubleValue();
            double unloadTime = timedStep.getTimeInterval().getStartTime() - unloadDurationParameter.get();
            double touchDownTime = timedStep.getTimeInterval().getEndTime();

            // trigger unload event
            if (currentTime >= unloadTime && currentTime < touchDownTime)
            {
               return StepEvent.UNLOAD;
            }
         }

         return null;
      }

      @Override
      public void onExit()
      {
      }
   }

   private class UnloadState implements FiniteStateMachineState<StepEvent>
   {
      private double initialTime;
      private RobotQuadrant robotQuadrant;

      public UnloadState(RobotQuadrant robotQuadrant)
      {
         this.robotQuadrant = robotQuadrant;
      }

      @Override
      public void onEntry()
      {
         initialTime = timestamp.getDoubleValue();
      }

      @Override
      public StepEvent process()
      {
         double elapsedTime = timestamp.getDoubleValue() - initialTime;
         double unloadDuration = unloadDurationParameter.get();

         QuadrupedTimedStep timedStep = getCurrentStep(robotQuadrant);
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

         // ramp up contact pressure
         double alpha = MathTools.clipToMinMax(1.0 - Math.pow(elapsedTime / unloadDuration, 0.5), 0.0, 1.0);
         double pressureLowerLimit = alpha * contactPressureLowerLimitParameter.get();
         double pressureUpperLimit = alpha * contactPressureUpperLimitParameter.get();
         contactForceLimits.setPressureLowerLimit(robotQuadrant, pressureLowerLimit);
         contactForceLimits.setPressureUpperLimit(robotQuadrant, pressureUpperLimit);
         return null;
      }

      @Override
      public void onExit()
      {
      }
   }

   private class SwingState implements FiniteStateMachineState<StepEvent>
   {
      private RobotQuadrant robotQuadrant;
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
         QuadrupedTimedStep timedStep = getCurrentStep(robotQuadrant);
         double groundClearance = timedStep.getGroundClearance();
         TimeInterval timeInterval = timedStep.getTimeInterval();
         timedStep.getGoalPosition(goalPosition);
         goalPosition.changeFrame(ReferenceFrame.getWorldFrame());
         goalPosition.add(0.0, 0.0, stepGoalOffsetZParameter.get());
         FramePoint solePosition = solePositionEstimate.get(robotQuadrant);
         solePosition.changeFrame(goalPosition.getReferenceFrame());
         swingTrajectory.initializeTrajectory(solePosition, goalPosition, groundClearance, timeInterval);

         // initialize contact state and feedback gains
         solePositionController.get(robotQuadrant).reset();
         solePositionController.get(robotQuadrant).getGains().setProportionalGains(solePositionProportionalGainsParameter.get());
         solePositionController.get(robotQuadrant).getGains().setDerivativeGains(solePositionDerivativeGainsParameter.get());
         solePositionController.get(robotQuadrant).getGains()
               .setIntegralGains(solePositionIntegralGainsParameter.get(), solePositionMaxIntegralErrorParameter.get());
         solePositionControllerSetpoints.get(robotQuadrant).initialize(taskSpaceEstimates);
      }

      @Override
      public StepEvent process()
      {
         QuadrupedTimedStep timedStep = getCurrentStep(robotQuadrant);
         double currentTime = timestamp.getDoubleValue();
         double touchDownTime = timedStep.getTimeInterval().getEndTime();

         // current goal position
         timedStep.getGoalPosition(goalPosition);
         goalPosition.changeFrame(ReferenceFrame.getWorldFrame());
         goalPosition.add(0.0, 0.0, stepGoalOffsetZParameter.get());

         // compute swing trajectory
         if (touchDownTime - currentTime > minimumStepAdjustmentTimeParameter.get())
         {
            swingTrajectory.adjustTrajectory(goalPosition, currentTime);
         }
         swingTrajectory.computeTrajectory(currentTime);
         swingTrajectory.getPosition(solePositionControllerSetpoints.get(robotQuadrant).getSolePosition());

         // compute sole force
         solePositionController.get(robotQuadrant)
               .compute(soleForceCommand.get(robotQuadrant), solePositionControllerSetpoints.get(robotQuadrant), taskSpaceEstimates);
         soleForceCommand.get(robotQuadrant).changeFrame(ReferenceFrame.getWorldFrame());

         // limit sole forces
         double coefficientOfFriction = soleCoefficientOfFrictionParameter.get();
         double pressureLimit = touchdownPressureLimitParameter.get();
         FrameVector soleForce = soleForceCommand.get(robotQuadrant);
         if (soleForce.getZ() < -pressureLimit)
         {
            // limit vertical force and project horizontal forces into friction pyramid
            soleForce.setX(Math.min(soleForce.getX(), coefficientOfFriction * pressureLimit));
            soleForce.setX(Math.max(soleForce.getX(), -coefficientOfFriction * pressureLimit));
            soleForce.setY(Math.min(soleForce.getY(), coefficientOfFriction * pressureLimit));
            soleForce.setY(Math.max(soleForce.getY(), -coefficientOfFriction * pressureLimit));
            soleForce.setZ(-pressureLimit);
         }

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
         soleForceCommand.get(robotQuadrant).setToZero();
      }
   }
}
