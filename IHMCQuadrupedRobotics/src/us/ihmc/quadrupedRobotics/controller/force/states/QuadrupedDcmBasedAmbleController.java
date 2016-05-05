package us.ihmc.quadrupedRobotics.controller.force.states;

import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.*;
import us.ihmc.quadrupedRobotics.estimator.GroundPlaneEstimator;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.params.DoubleArrayParameter;
import us.ihmc.quadrupedRobotics.params.DoubleParameter;
import us.ihmc.quadrupedRobotics.params.ParameterFactory;
import us.ihmc.quadrupedRobotics.planning.*;
import us.ihmc.quadrupedRobotics.planning.trajectory.PiecewiseForwardDcmTrajectory;
import us.ihmc.quadrupedRobotics.planning.trajectory.PiecewisePeriodicDcmTrajectory;
import us.ihmc.quadrupedRobotics.providers.QuadrupedControllerInputProviderInterface;
import us.ihmc.quadrupedRobotics.state.FiniteStateMachine;
import us.ihmc.quadrupedRobotics.state.FiniteStateMachineBuilder;
import us.ihmc.quadrupedRobotics.state.FiniteStateMachineState;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.*;

import java.util.ArrayList;

public class QuadrupedDcmBasedAmbleController implements QuadrupedController
{
   private final QuadrupedControllerInputProviderInterface inputProvider;
   private final DoubleYoVariable robotTimestamp;
   private final double controlDT;
   private final double gravity;
   private final double mass;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   // parameters
   private final ParameterFactory parameterFactory = new ParameterFactory(getClass());
   private final DoubleParameter jointDampingParameter = parameterFactory.createDouble("jointDamping", 2);
   private final DoubleArrayParameter bodyOrientationProportionalGainsParameter = parameterFactory.createDoubleArray("bodyOrientationProportionalGains", 5000, 5000, 5000);
   private final DoubleArrayParameter bodyOrientationDerivativeGainsParameter = parameterFactory.createDoubleArray("bodyOrientationDerivativeGains", 750, 750, 750);
   private final DoubleArrayParameter bodyOrientationIntegralGainsParameter = parameterFactory.createDoubleArray("bodyOrientationIntegralGains", 0, 0, 0);
   private final DoubleParameter bodyOrientationMaxIntegralErrorParameter = parameterFactory.createDouble("bodyOrientationMaxIntegralError", 0);
   private final DoubleArrayParameter comPositionProportionalGainsParameter = parameterFactory.createDoubleArray("comPositionProportionalGains", 0, 0, 5000);
   private final DoubleArrayParameter comPositionDerivativeGainsParameter = parameterFactory.createDoubleArray("comPositionDerivativeGains", 0, 0, 750);
   private final DoubleArrayParameter comPositionIntegralGainsParameter = parameterFactory.createDoubleArray("comPositionIntegralGains", 0, 0, 0);
   private final DoubleParameter comPositionMaxIntegralErrorParameter = parameterFactory.createDouble("comPositionMaxIntegralError", 0);
   private final DoubleArrayParameter dcmPositionProportionalGainsParameter = parameterFactory.createDoubleArray("dcmPositionProportionalGains", 1, 1, 0);
   private final DoubleArrayParameter dcmPositionDerivativeGainsParameter = parameterFactory.createDoubleArray("dcmPositionDerivativeGains", 0, 0, 0);
   private final DoubleArrayParameter dcmPositionIntegralGainsParameter = parameterFactory.createDoubleArray("dcmPositionIntegralGains", 0, 0, 0);
   private final DoubleParameter dcmPositionMaxIntegralErrorParameter = parameterFactory.createDouble("dcmPositionMaxIntegralError", 0);
   private final DoubleParameter initialTransitionDurationParameter = parameterFactory.createDouble("initialTransitionDuration", 1.00);
   private final DoubleParameter stanceWidthNominalParameter = parameterFactory.createDouble("stanceWidthNominal", 0.25);
   private final DoubleParameter stanceLengthNominalParameter = parameterFactory.createDouble("stanceLengthNominal", 1.1);
   private final DoubleParameter stepGroundClearanceParameter = parameterFactory.createDouble("stepGroundClearance", 0.10);
   private final DoubleParameter stepDurationParameter = parameterFactory.createDouble("stepDuration", 0.20);
   private final DoubleParameter endPairPhaseShiftParameter = parameterFactory.createDouble("endPairPhaseShift", 90);

   // frames
   private final ReferenceFrame supportFrame;
   private final ReferenceFrame worldFrame;

   // feedback controllers
   private final FramePoint dcmPositionEstimate;
   private final DivergentComponentOfMotionController.Setpoints dcmPositionControllerSetpoints;
   private final DivergentComponentOfMotionController dcmPositionController;
   private final QuadrupedComPositionController.Setpoints comPositionControllerSetpoints;
   private final QuadrupedComPositionController comPositionController;
   private final QuadrupedBodyOrientationController.Setpoints bodyOrientationControllerSetpoints;
   private final QuadrupedBodyOrientationController bodyOrientationController;
   private final QuadrupedTimedStepController.Setpoints timedStepControllerSetpoints;
   private final QuadrupedTimedStepController timedStepController;

   // task space controller
   private final QuadrupedTaskSpaceEstimator.Estimates taskSpaceEstimates;
   private final QuadrupedTaskSpaceEstimator taskSpaceEstimator;
   private final QuadrupedTaskSpaceController.Commands taskSpaceControllerCommands;
   private final QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings;
   private final QuadrupedTaskSpaceController taskSpaceController;

   // planning
   private double bodyYawSetpoint;
   private final GroundPlaneEstimator groundPlaneEstimator;
   private final QuadrantDependentList<FramePoint> groundPlanePositions;

   // state machine
   public enum AmbleState
   {
      INITIAL_TRANSITION, FORWARD_AMBLE, REVERSE_AMBLE
   }
   public enum AmbleEvent
   {
      TIMEOUT, FORWARD, REVERSE
   }
   private final FiniteStateMachine<AmbleState, AmbleEvent> ambleStateMachine;

   public QuadrupedDcmBasedAmbleController(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedForceControllerToolbox controllerToolbox,
         QuadrupedControllerInputProviderInterface inputProvider)
   {
      this.inputProvider = inputProvider;
      this.robotTimestamp = runtimeEnvironment.getRobotTimestamp();
      this.controlDT = runtimeEnvironment.getControlDT();
      this.gravity = 9.81;
      this.mass = runtimeEnvironment.getFullRobotModel().getTotalMass();

      // frames
      QuadrupedReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      worldFrame = ReferenceFrame.getWorldFrame();

      // feedback controllers
      dcmPositionEstimate = new FramePoint();
      dcmPositionControllerSetpoints = new DivergentComponentOfMotionController.Setpoints();
      dcmPositionController = controllerToolbox.getDcmPositionController();
      comPositionControllerSetpoints = new QuadrupedComPositionController.Setpoints();
      comPositionController = controllerToolbox.getComPositionController();
      bodyOrientationControllerSetpoints = new QuadrupedBodyOrientationController.Setpoints();
      bodyOrientationController = controllerToolbox.getBodyOrientationController();
      timedStepControllerSetpoints = new QuadrupedTimedStepController.Setpoints();
      timedStepController = controllerToolbox.getTimedStepController();

      // task space controllers
      taskSpaceEstimates = new QuadrupedTaskSpaceEstimator.Estimates();
      taskSpaceEstimator = controllerToolbox.getTaskSpaceEstimator();
      taskSpaceControllerCommands = new QuadrupedTaskSpaceController.Commands();
      taskSpaceControllerSettings = new QuadrupedTaskSpaceController.Settings();
      taskSpaceController = controllerToolbox.getTaskSpaceController();

      // planning
      groundPlaneEstimator = new GroundPlaneEstimator();
      groundPlanePositions = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         groundPlanePositions.set(robotQuadrant, new FramePoint());
      }

      // state machine
      FiniteStateMachineBuilder<AmbleState, AmbleEvent> ambleStateMachineBuilder = new FiniteStateMachineBuilder<>(AmbleState.class, AmbleEvent.class, "AmbleState", registry);
      ambleStateMachineBuilder.addState(AmbleState.INITIAL_TRANSITION, new InitialTransitionState());
      ambleStateMachineBuilder.addState(AmbleState.FORWARD_AMBLE, new ForwardAmbleState());
      ambleStateMachineBuilder.addState(AmbleState.REVERSE_AMBLE, new ReverseAmbleState());
      ambleStateMachineBuilder.addTransition(AmbleEvent.TIMEOUT, AmbleState.INITIAL_TRANSITION, AmbleState.FORWARD_AMBLE);
      ambleStateMachineBuilder.addTransition(AmbleEvent.REVERSE, AmbleState.FORWARD_AMBLE, AmbleState.REVERSE_AMBLE);
      ambleStateMachineBuilder.addTransition(AmbleEvent.FORWARD, AmbleState.REVERSE_AMBLE, AmbleState.FORWARD_AMBLE);
      ambleStateMachine = ambleStateMachineBuilder.build(AmbleState.INITIAL_TRANSITION);
      runtimeEnvironment.getParentRegistry().addChild(registry);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   private void updateEstimates()
   {
      // update task space estimates
      taskSpaceEstimator.compute(taskSpaceEstimates);

      // update dcm estimate
      taskSpaceEstimates.getComPosition().changeFrame(worldFrame);
      taskSpaceEstimates.getComVelocity().changeFrame(worldFrame);
      dcmPositionEstimate.changeFrame(worldFrame);
      dcmPositionEstimate.set(taskSpaceEstimates.getComVelocity());
      dcmPositionEstimate.scale(1.0 / dcmPositionController.getNaturalFrequency());
      dcmPositionEstimate.add(taskSpaceEstimates.getComPosition());

      // update ground plane estimate
      groundPlaneEstimator.compute(groundPlanePositions);
   }

   private void updateSetpoints()
   {
      // update state machines
      ambleStateMachine.process();

      // update desired horizontal com forces
      dcmPositionController.compute(taskSpaceControllerCommands.getComForce(), dcmPositionControllerSetpoints, dcmPositionEstimate);
      taskSpaceControllerCommands.getComForce().changeFrame(supportFrame);

      // update desired com position, velocity, and vertical force
      comPositionControllerSetpoints.getComPosition().changeFrame(supportFrame);
      comPositionControllerSetpoints.getComPosition().set(inputProvider.getComPositionInput());
      comPositionControllerSetpoints.getComVelocity().changeFrame(supportFrame);
      comPositionControllerSetpoints.getComVelocity().set(inputProvider.getComVelocityInput());
      comPositionControllerSetpoints.getComForceFeedforward().changeFrame(supportFrame);
      comPositionControllerSetpoints.getComForceFeedforward().set(taskSpaceControllerCommands.getComForce());
      comPositionControllerSetpoints.getComForceFeedforward().setZ(mass * gravity);
      comPositionController.compute(taskSpaceControllerCommands.getComForce(), comPositionControllerSetpoints, taskSpaceEstimates);

      // update desired body orientation, angular velocity, and torque
      if (ambleStateMachine.getState() != AmbleState.INITIAL_TRANSITION)
      {
         bodyYawSetpoint += inputProvider.getPlanarVelocityInput().getZ() * controlDT;
      }
      bodyOrientationControllerSetpoints.getBodyOrientation().changeFrame(worldFrame);
      bodyOrientationControllerSetpoints.getBodyOrientation().setYawPitchRoll(bodyYawSetpoint,
            RotationTools.computePitch(inputProvider.getBodyOrientationInput()) + groundPlaneEstimator.getPitch(bodyYawSetpoint),
            RotationTools.computeRoll(inputProvider.getBodyOrientationInput()));
      bodyOrientationControllerSetpoints.getBodyAngularVelocity().setToZero();
      bodyOrientationControllerSetpoints.getComTorqueFeedforward().setToZero();
      bodyOrientationController.compute(taskSpaceControllerCommands.getComTorque(), bodyOrientationControllerSetpoints, taskSpaceEstimates);

      // update desired contact state and sole forces
      FramePoint dcmPositionSetpoint = dcmPositionControllerSetpoints.getDcmPosition();
      dcmPositionSetpoint.changeFrame(worldFrame);
      dcmPositionEstimate.changeFrame(worldFrame);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         timedStepControllerSetpoints.getStepAdjustment(robotQuadrant).changeFrame(worldFrame);
         timedStepControllerSetpoints.getStepAdjustment(robotQuadrant).set(dcmPositionEstimate.getX(), dcmPositionEstimate.getY(), 0.0);
         timedStepControllerSetpoints.getStepAdjustment(robotQuadrant).sub(dcmPositionSetpoint.getX(), dcmPositionSetpoint.getY(), 0.0);
      }
      timedStepController.compute(taskSpaceControllerSettings.getContactState(), taskSpaceControllerCommands.getSoleForce(), timedStepControllerSetpoints, taskSpaceEstimates);

      // update joint setpoints
      taskSpaceController.compute(taskSpaceControllerSettings, taskSpaceControllerCommands);
   }

   @Override public ControllerEvent process()
   {
      updateEstimates();
      updateSetpoints();
      return null;
   }

   @Override public void onEntry()
   {
      // initialize estimates
      dcmPositionController.setComHeight(inputProvider.getComPositionInput().getZ());
      updateEstimates();

      // initialize feedback controllers
      dcmPositionControllerSetpoints.initialize(dcmPositionEstimate);
      dcmPositionController.reset();
      dcmPositionController.getGains().setProportionalGains(dcmPositionProportionalGainsParameter.get());
      dcmPositionController.getGains().setIntegralGains(dcmPositionIntegralGainsParameter.get(), dcmPositionMaxIntegralErrorParameter.get());
      dcmPositionController.getGains().setDerivativeGains(dcmPositionDerivativeGainsParameter.get());
      comPositionControllerSetpoints.initialize(taskSpaceEstimates);
      comPositionController.reset();
      comPositionController.getGains().setProportionalGains(comPositionProportionalGainsParameter.get());
      comPositionController.getGains().setIntegralGains(comPositionIntegralGainsParameter.get(), comPositionMaxIntegralErrorParameter.get());
      comPositionController.getGains().setDerivativeGains(comPositionDerivativeGainsParameter.get());
      bodyOrientationControllerSetpoints.initialize(taskSpaceEstimates);
      bodyOrientationController.reset();
      bodyOrientationController.getGains().setProportionalGains(bodyOrientationProportionalGainsParameter.get());
      bodyOrientationController.getGains().setIntegralGains(bodyOrientationIntegralGainsParameter.get(), bodyOrientationMaxIntegralErrorParameter.get());
      bodyOrientationController.getGains().setDerivativeGains(bodyOrientationDerivativeGainsParameter.get());
      timedStepControllerSetpoints.initialize(taskSpaceEstimates);
      timedStepController.reset();

      // initialize task space controller
      taskSpaceControllerSettings.initialize();
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointDamping(jointDampingParameter.get());
      taskSpaceControllerSettings.getContactForceOptimizationSettings().setComForceCommandWeights(1.0, 1.0, 1.0);
      taskSpaceControllerSettings.getContactForceOptimizationSettings().setComTorqueCommandWeights(1.0, 1.0, 1.0);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         taskSpaceControllerSettings.getContactForceOptimizationSettings().setContactForceCommandWeights(robotQuadrant, 0.0, 0.0, 0.0);
         taskSpaceControllerSettings.setContactState(robotQuadrant, ContactState.IN_CONTACT);
      }
      taskSpaceController.reset();

      // initialize body yaw trajectory
      taskSpaceEstimates.getBodyOrientation().changeFrame(worldFrame);
      bodyYawSetpoint = taskSpaceEstimates.getBodyOrientation().getYaw();

      // initialize state machine
      ambleStateMachine.reset();
   }

   @Override public void onExit()
   {
   }

   private class InitialTransitionState implements FiniteStateMachineState<AmbleEvent>
   {
      double initialTime;
      private final QuadrupedTimedStep timedStep;
      private final FramePoint footholdPosition;

      public InitialTransitionState()
      {
         timedStep = new QuadrupedTimedStep();
         footholdPosition = new FramePoint();
      }

      @Override public void onEntry()
      {
         initialTime = robotTimestamp.getDoubleValue();
         RobotQuadrant firstQuadrant, secondQuadrant;
         double stepTimeShift;

         // select initial step quadrants
         if (endPairPhaseShiftParameter.get() < 90)
         {
            firstQuadrant = RobotQuadrant.HIND_LEFT;
            stepTimeShift = Math.max(stepDurationParameter.get() * endPairPhaseShiftParameter.get() / 180.0, controlDT);
         }
         else
         {
            firstQuadrant = RobotQuadrant.FRONT_LEFT;
            stepTimeShift = Math.max(stepDurationParameter.get() * (1.0 - endPairPhaseShiftParameter.get() / 180.0), controlDT);
         }
         secondQuadrant = firstQuadrant.getNextRegularGaitSwingQuadrant();

         // trigger first step
         footholdPosition.setIncludingFrame(taskSpaceEstimates.getSolePosition(firstQuadrant));
         timedStep.setRobotQuadrant(firstQuadrant);
         timedStep.setGroundClearance(stepGroundClearanceParameter.get());
         timedStep.getTimeInterval().setInterval(0.0, stepDurationParameter.get());
         timedStep.getTimeInterval().shiftInterval(initialTime + initialTransitionDurationParameter.get());
         timedStep.getGoalPosition().setIncludingFrame(footholdPosition);
         timedStepController.addStep(timedStep);

         // trigger second step
         footholdPosition.setIncludingFrame(taskSpaceEstimates.getSolePosition(secondQuadrant));
         timedStep.setRobotQuadrant(secondQuadrant);
         timedStep.setGroundClearance(stepGroundClearanceParameter.get());
         timedStep.getTimeInterval().shiftInterval(stepTimeShift);
         timedStep.getGoalPosition().setIncludingFrame(footholdPosition);
         timedStepController.addStep(timedStep);

         // initialize ground plane points
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            groundPlanePositions.get(robotQuadrant).setIncludingFrame(taskSpaceEstimates.getSolePosition(robotQuadrant));
            groundPlanePositions.get(robotQuadrant).changeFrame(ReferenceFrame.getWorldFrame());
         }
      }

      @Override public AmbleEvent process()
      {
         double currentTime = robotTimestamp.getDoubleValue();

         if (currentTime > initialTime + initialTransitionDurationParameter.get())
         {
            return AmbleEvent.TIMEOUT;
         }
         return null;
      }

      @Override public void onExit()
      {
      }
   }

   private class ForwardAmbleState implements FiniteStateMachineState<AmbleEvent>, QuadrupedTimedStepTransitionCallback
   {
      private final FramePoint previewDcmPositionAtEndOfStep;
      private final PiecewiseForwardDcmTrajectory previewDcmTrajectory;
      private final QuadrupedTimedStepCopPlanner previewCopPlanner;
      private final FramePoint currentFootholdPosition;
      private final QuadrupedTimedStep nextStep;

      public ForwardAmbleState()
      {
         previewDcmPositionAtEndOfStep = new FramePoint();
         previewDcmTrajectory = new PiecewiseForwardDcmTrajectory(2 * timedStepController.getQueueCapacity() + 1, gravity, dcmPositionController.getComHeight());
         previewCopPlanner = new QuadrupedTimedStepCopPlanner(timedStepController.getQueueCapacity());
         currentFootholdPosition = new FramePoint();
         nextStep = new QuadrupedTimedStep();
      }

      @Override public void onEntry()
      {
         double currentTime = robotTimestamp.getDoubleValue();
         timedStepController.registerStepTransitionCallback(this);

         // compute nominal dcm trajectory
         int nIntervals = previewCopPlanner.compute(timedStepController.getQueue(), taskSpaceEstimates.getSolePosition(), taskSpaceControllerSettings.getContactState(), currentTime);
         previewDcmTrajectory.setComHeight(dcmPositionController.getComHeight());
         previewDcmTrajectory.initializeTrajectory(nIntervals, previewCopPlanner.getTimeAtStartOfInterval(), previewCopPlanner.getCopAtStartOfInterval(), dcmPositionEstimate);
      }

      @Override public void onLiftOff(RobotQuadrant robotQuadrant, QuadrantDependentList<ContactState> contactState)
      {
         double currentTime = robotTimestamp.getDoubleValue();
         QuadrupedTimedStep currentStep = timedStepController.getCurrentStep(robotQuadrant);
         RobotQuadrant lastStepQuadrant = robotQuadrant.getNextReversedRegularGaitSwingQuadrant();
         RobotQuadrant nextStepQuadrant = robotQuadrant.getNextRegularGaitSwingQuadrant();
         RobotQuadrant currentStepQuadrant = robotQuadrant;

         // enqueue next step
         double lastStepEndTime = timedStepController.getCurrentStep(lastStepQuadrant).getTimeInterval().getEndTime();
         double nextStepStartTime = lastStepEndTime + controlDT;
         nextStep.setRobotQuadrant(nextStepQuadrant);
         nextStep.getTimeInterval().setStartTime(nextStepStartTime);
         nextStep.getTimeInterval().setEndTime(lastStepEndTime + stepDurationParameter.get());
         timedStepController.addStep(nextStep);

         // compute nominal dcm trajectory
         int nIntervals = previewCopPlanner.compute(timedStepController.getQueue(), taskSpaceEstimates.getSolePosition(), contactState, currentTime);
         previewDcmTrajectory.setComHeight(dcmPositionController.getComHeight());
         previewDcmTrajectory.initializeTrajectory(nIntervals, previewCopPlanner.getTimeAtStartOfInterval(), previewCopPlanner.getCopAtStartOfInterval(), dcmPositionEstimate);

         // compute current step duration
         double phaseShift = robotQuadrant.isQuadrantInFront() ? endPairPhaseShiftParameter.get() : 180.0 - endPairPhaseShiftParameter.get();
         double currentStepEndTime = nextStepStartTime + Math.max(stepDurationParameter.get() * phaseShift / 180.0, controlDT);
         double currentStepDuration = Math.min(Math.max(currentStepEndTime - currentTime, stepDurationParameter.get()), 1.5 * stepDurationParameter.get());
         currentStepEndTime = currentTime + currentStepDuration;
         currentStep.getTimeInterval().setEndTime(currentStepEndTime);

         // compute current step goal position and ground clearance
         currentFootholdPosition.setIncludingFrame(taskSpaceEstimates.getSolePosition(currentStepQuadrant));
         previewDcmTrajectory.computeTrajectory(currentStepEndTime);
         previewDcmTrajectory.getPosition(previewDcmPositionAtEndOfStep);
         currentStep.setGoalPosition(currentFootholdPosition);
         currentStep.setGroundClearance(stepGroundClearanceParameter.get());
      }

      @Override public void onTouchDown(RobotQuadrant robotQuadrant, QuadrantDependentList<ContactState> contactState)
      {
      }

      @Override public AmbleEvent process()
      {
         // compute nominal dcm trajectory
         previewDcmTrajectory.computeTrajectory(robotTimestamp.getDoubleValue());
         previewDcmTrajectory.getPosition(dcmPositionControllerSetpoints.getDcmPosition());
         previewDcmTrajectory.getVelocity(dcmPositionControllerSetpoints.getDcmVelocity());
         return null;
      }

      @Override public void onExit()
      {
      }
   }

   private class ReverseAmbleState implements FiniteStateMachineState<AmbleEvent>, QuadrupedTimedStepTransitionCallback
   {
      public ReverseAmbleState()
      {
      }

      @Override public void onEntry()
      {
      }

      @Override public void onLiftOff(RobotQuadrant robotQuadrant, QuadrantDependentList<ContactState> contactState)
      {
      }

      @Override public void onTouchDown(RobotQuadrant robotQuadrant, QuadrantDependentList<ContactState> contactState)
      {
      }

      @Override public AmbleEvent process()
      {
         return null;
      }

      @Override public void onExit()
      {
      }
   }
}
