package us.ihmc.quadrupedRobotics.controller.forceDevelopment;

import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedStepTransitionCallback;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimator;
import us.ihmc.quadrupedRobotics.optimization.contactForceOptimization.QuadrupedContactForceLimits;
import us.ihmc.robotics.dataStructures.parameter.DoubleArrayParameter;

import us.ihmc.robotics.dataStructures.parameter.DoubleParameter;
import us.ihmc.robotics.dataStructures.parameter.IntegerParameter;
import us.ihmc.robotics.dataStructures.parameter.ParameterFactory;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.trajectory.ThreeDoFSwingFootTrajectory;
import us.ihmc.quadrupedRobotics.state.FiniteStateMachine;
import us.ihmc.quadrupedRobotics.state.FiniteStateMachineBuilder;
import us.ihmc.quadrupedRobotics.state.FiniteStateMachineState;
import us.ihmc.quadrupedRobotics.util.*;
import us.ihmc.quadrupedRobotics.state.FiniteStateMachineStateChangedListener;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.filters.GlitchFilteredBooleanYoVariable;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;

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
   private final IntegerParameter touchdownTriggerWindowParameter = parameterFactory.createInteger("touchdownTriggerWindow", 1);
   private final DoubleParameter contactPressureLowerLimitParameter = parameterFactory.createDouble("contactPressureLowerLimit", 50);
   private final DoubleParameter contactPressureUpperLimitParameter = parameterFactory.createDouble("contactPressureUpperLimit", 1000);
   private final DoubleParameter minimumStepAdjustmentTimeParameter = parameterFactory.createDouble("minimumStepAdjustmentTime", 0.1);
   private final DoubleParameter stepGoalOffsetZParameter = parameterFactory.createDouble("stepGoalOffsetZ", 0.0);

   // control variables
   private final DoubleYoVariable timestamp;
   private final QuadrantDependentList<QuadrupedSolePositionController> solePositionController;
   private final QuadrantDependentList<QuadrupedSolePositionController.Setpoints> solePositionControllerSetpoints;
   private final PreallocatedList<QuadrupedTimedStep> stepSequence;
   private final QuadrantDependentList<FramePoint> solePositionEstimate;
   private final QuadrantDependentList<FrameVector> soleForceCommand;
   private final QuadrantDependentList<ContactState> contactState;
   private final QuadrupedContactForceLimits contactForceLimits;
   private final QuadrupedTaskSpaceEstimator.Estimates taskSpaceEstimates;

   // graphics
   private final FramePoint stepSequenceVisualizationPosition;
   private final QuadrantDependentList<BagOfBalls> stepSequenceVisualization;
   private static final QuadrantDependentList<AppearanceDefinition> stepSequenceAppearance = new QuadrantDependentList<>(YoAppearance.Red(), YoAppearance.Blue(),
         YoAppearance.RGBColor(1, 0.5, 0.0), YoAppearance.RGBColor(0.0, 0.5, 1.0));

   // state machine
   public enum StepState
   {
      SUPPORT, SWING
   }

   public enum StepEvent
   {
      TIMEOUT
   }

   private final QuadrantDependentList<FiniteStateMachine<StepState, StepEvent>> stepStateMachine;
   private QuadrupedStepTransitionCallback stepTransitionCallback;

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
      stepSequence = new PreallocatedList<>(100, QuadrupedTimedStep.class);
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
         stateMachineBuilder.addState(StepState.SUPPORT, new SupportState(robotQuadrant));
         stateMachineBuilder.addState(StepState.SWING, new SwingState(robotQuadrant));
         stateMachineBuilder.addTransition(StepEvent.TIMEOUT, StepState.SUPPORT, StepState.SWING);
         stateMachineBuilder.addTransition(StepEvent.TIMEOUT, StepState.SWING, StepState.SUPPORT);
         stepStateMachine.set(robotQuadrant, stateMachineBuilder.build(StepState.SUPPORT));
      }
      stepTransitionCallback = null;

      // graphics
      stepSequenceVisualization = new QuadrantDependentList<>();
      stepSequenceVisualizationPosition = new FramePoint();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         AppearanceDefinition appearance = stepSequenceAppearance.get(robotQuadrant);
         String prefix = "timedStepController" + robotQuadrant.getPascalCaseName() + "GoalPositions";
         stepSequenceVisualization.set(robotQuadrant, new BagOfBalls(stepSequence.capacity(), 0.015, prefix, appearance, registry, graphicsListRegistry));
      }
      parentRegistry.addChild(registry);
   }

   public void registerStepTransitionCallback(QuadrupedStepTransitionCallback stepTransitionCallback)
   {
      this.stepTransitionCallback = stepTransitionCallback;
   }

   public boolean addStep(QuadrupedTimedStep timedStep)
   {
      for (int i = 0; i < stepSequence.size(); i++)
      {
         if (timedStep.getRobotQuadrant() == stepSequence.get(i).getRobotQuadrant())
         {
            if (timedStep.getTimeInterval().getStartTime() < stepSequence.get(i).getTimeInterval().getEndTime())
               return false;
         }
      }
      if ((timestamp.getDoubleValue() <= timedStep.getTimeInterval().getStartTime()) && stepSequence.add())
      {
         stepSequence.get(stepSequence.size() - 1).set(timedStep);
         TimeIntervalTools.sortByEndTime(stepSequence);
         return true;
      }
      else
      {
         return false;
      }
   }

   public void removeSteps()
   {
      double currentTime = timestamp.getDoubleValue();
      TimeIntervalTools.removeStartTimesGreaterThan(currentTime, stepSequence);
   }

   public PreallocatedList<QuadrupedTimedStep> getStepSequence()
   {
      return stepSequence;
   }

   public int getStepSequenceSize()
   {
      return stepSequence.size();
   }

   public int getStepSequenceCapacity()
   {
      return stepSequence.capacity();
   }

   public QuadrupedTimedStep getCurrentStep(RobotEnd robotEnd)
   {
      for (int i = 0; i < stepSequence.size(); i++)
      {
         QuadrupedTimedStep step = stepSequence.get(i);
         if (step.getRobotQuadrant().getEnd() == robotEnd)
         {
            return step;
         }
      }
      return null;
   }

   public QuadrupedTimedStep getCurrentStep(RobotQuadrant robotQuadrant)
   {
      for (int i = 0; i < stepSequence.size(); i++)
      {
         QuadrupedTimedStep step = stepSequence.get(i);
         if (step.getRobotQuadrant() == robotQuadrant)
         {
            return step;
         }
      }
      return null;
   }

   public void attachStateChangedListener(FiniteStateMachineStateChangedListener stateChangedListener)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         stepStateMachine.get(robotQuadrant).attachStateChangedListener(stateChangedListener);
      }
   }

   public void reset()
   {
      stepSequence.clear();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         stepStateMachine.get(robotQuadrant).reset();
         contactForceLimits.setPressureUpperLimit(robotQuadrant, contactPressureUpperLimitParameter.get());
         contactForceLimits.setPressureLowerLimit(robotQuadrant, contactPressureLowerLimitParameter.get());
         contactState.set(robotQuadrant, ContactState.IN_CONTACT);
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
      TimeIntervalTools.removeEndTimesLessThan(currentTime, stepSequence);

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
      for (int i = 0; i < stepSequence.capacity(); i++)
      {
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            stepSequenceVisualizationPosition.setToZero();
            stepSequenceVisualization.get(robotQuadrant).setBall(stepSequenceVisualizationPosition, i);
         }
      }
      for (int i = 0; i < stepSequence.size(); i++)
      {
         stepSequence.get(i).getGoalPosition(stepSequenceVisualizationPosition);
         RobotQuadrant robotQuadrant = stepSequence.get(i).getRobotQuadrant();
         stepSequenceVisualization.get(robotQuadrant).setBallLoop(stepSequenceVisualizationPosition);
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
            double liftOffTime = timedStep.getTimeInterval().getStartTime();
            double touchDownTime = timedStep.getTimeInterval().getEndTime();

            // trigger swing phase
            if (currentTime >= liftOffTime && currentTime < touchDownTime)
            {
               if (stepTransitionCallback != null)
               {
                  stepTransitionCallback.onLiftOff(robotQuadrant);
               }
               contactState.set(robotQuadrant, ContactState.NO_CONTACT);
               return StepEvent.TIMEOUT;
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
      private RobotQuadrant robotQuadrant;
      private final ThreeDoFSwingFootTrajectory swingTrajectory;
      private final FramePoint goalPosition;
      private final GlitchFilteredBooleanYoVariable touchdownTrigger;

      public SwingState(RobotQuadrant robotQuadrant)
      {
         this.robotQuadrant = robotQuadrant;
         this.goalPosition = new FramePoint();
         this.swingTrajectory = new ThreeDoFSwingFootTrajectory(this.robotQuadrant.getPascalCaseName(), registry);
         this.touchdownTrigger = new GlitchFilteredBooleanYoVariable(this.robotQuadrant.getCamelCaseName() + "TouchdownTriggered", registry,
               touchdownTriggerWindowParameter.get());
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

         touchdownTrigger.set(false);
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

         // detect early touch-down
         FrameVector soleForceEstimate = taskSpaceEstimates.getSoleVirtualForce(robotQuadrant);
         soleForceEstimate.changeFrame(ReferenceFrame.getWorldFrame());
         double pressureEstimate = -soleForceEstimate.getZ();
         double relativeTimeInSwing = currentTime - timedStep.getTimeInterval().getStartTime();
         double normalizedTimeInSwing = relativeTimeInSwing / timedStep.getTimeInterval().getDuration();
         if (normalizedTimeInSwing > 0.5)
         {
            touchdownTrigger.update(pressureEstimate > touchdownPressureLimitParameter.get());
         }

         // compute sole force
         if (touchdownTrigger.getBooleanValue())
         {
            double pressureLimit = touchdownPressureLimitParameter.get();
            soleForceCommand.get(robotQuadrant).changeFrame(ReferenceFrame.getWorldFrame());
            soleForceCommand.get(robotQuadrant).set(0, 0, -pressureLimit);
         }
         else
         {
            solePositionController.get(robotQuadrant)
                  .compute(soleForceCommand.get(robotQuadrant), solePositionControllerSetpoints.get(robotQuadrant), taskSpaceEstimates);
            soleForceCommand.get(robotQuadrant).changeFrame(ReferenceFrame.getWorldFrame());
         }

         // trigger support phase
         if (currentTime >= touchDownTime)
         {
            if (stepTransitionCallback != null)
            {
               stepTransitionCallback.onTouchDown(robotQuadrant);
            }
            contactState.set(robotQuadrant, ContactState.IN_CONTACT);
            return StepEvent.TIMEOUT;
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
