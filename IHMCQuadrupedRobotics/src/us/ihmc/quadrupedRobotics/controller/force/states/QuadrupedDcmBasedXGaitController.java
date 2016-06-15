package us.ihmc.quadrupedRobotics.controller.force.states;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
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
import us.ihmc.quadrupedRobotics.planning.trajectory.PiecewiseReverseDcmTrajectory;
import us.ihmc.quadrupedRobotics.planning.trajectory.ThreeDoFMinimumJerkTrajectory;
import us.ihmc.quadrupedRobotics.providers.QuadrupedControllerInputProviderInterface;
import us.ihmc.quadrupedRobotics.providers.QuadrupedXGaitSettingsProvider;
import us.ihmc.quadrupedRobotics.state.FiniteStateMachine;
import us.ihmc.quadrupedRobotics.state.FiniteStateMachineBuilder;
import us.ihmc.quadrupedRobotics.state.FiniteStateMachineState;
import us.ihmc.quadrupedRobotics.util.PreallocatedQueue;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.*;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.BagOfBalls;

import java.util.ArrayList;

public class QuadrupedDcmBasedXGaitController implements QuadrupedController
{
   private final QuadrupedControllerInputProviderInterface inputProvider;
   private final QuadrupedXGaitSettingsProvider settingsProvider;
   private final DoubleYoVariable robotTimestamp;
   private final double controlDT;
   private final double gravity;
   private final double mass;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   // parameters
   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final DoubleArrayParameter bodyOrientationProportionalGainsParameter = parameterFactory.createDoubleArray("bodyOrientationProportionalGains", 5000, 5000, 5000);
   private final DoubleArrayParameter bodyOrientationDerivativeGainsParameter = parameterFactory.createDoubleArray("bodyOrientationDerivativeGains", 750, 750, 750);
   private final DoubleArrayParameter bodyOrientationIntegralGainsParameter = parameterFactory.createDoubleArray("bodyOrientationIntegralGains", 0, 0, 0);
   private final DoubleParameter bodyOrientationMaxIntegralErrorParameter = parameterFactory.createDouble("bodyOrientationMaxIntegralError", 0);
   private final DoubleArrayParameter comPositionProportionalGainsParameter = parameterFactory.createDoubleArray("comPositionProportionalGains", 0, 0, 5000);
   private final DoubleArrayParameter comPositionDerivativeGainsParameter = parameterFactory.createDoubleArray("comPositionDerivativeGains", 0, 0, 750);
   private final DoubleArrayParameter comPositionIntegralGainsParameter = parameterFactory.createDoubleArray("comPositionIntegralGains", 0, 0, 0);
   private final DoubleParameter comPositionMaxIntegralErrorParameter = parameterFactory.createDouble("comPositionMaxIntegralError", 0);
   private final DoubleParameter comPositionGravityCompensationParameter = parameterFactory.createDouble("comPositionGravityCompensation", 1);
   private final DoubleArrayParameter dcmPositionProportionalGainsParameter = parameterFactory.createDoubleArray("dcmPositionProportionalGains", 1, 1, 0);
   private final DoubleArrayParameter dcmPositionDerivativeGainsParameter = parameterFactory.createDoubleArray("dcmPositionDerivativeGains", 0, 0, 0);
   private final DoubleArrayParameter dcmPositionIntegralGainsParameter = parameterFactory.createDoubleArray("dcmPositionIntegralGains", 0, 0, 0);
   private final DoubleParameter dcmPositionMaxIntegralErrorParameter = parameterFactory.createDouble("dcmPositionMaxIntegralError", 0);
   private final DoubleParameter jointDampingParameter = parameterFactory.createDouble("jointDamping", 2);
   private final DoubleParameter jointPositionLimitDampingParameter = parameterFactory.createDouble("jointPositionLimitDamping", 10);
   private final DoubleParameter jointPositionLimitStiffnessParameter = parameterFactory.createDouble("jointPositionLimitStiffness", 100);
   private final DoubleParameter contactPressureLowerLimitParameter = parameterFactory.createDouble("contactPressureLowerLimit", 50);
   private final DoubleParameter initialTransitionDurationParameter = parameterFactory.createDouble("initialTransitionDuration", 1.00);
   private final DoubleParameter footholdDistanceLowerLimitParameter = parameterFactory.createDouble("footholdDistanceLowerLimit", 0.15);

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
   private final QuadrupedTimedStepController timedStepController;

   // task space controller
   private final QuadrupedTaskSpaceEstimator.Estimates taskSpaceEstimates;
   private final QuadrupedTaskSpaceEstimator taskSpaceEstimator;
   private final QuadrupedTaskSpaceController.Commands taskSpaceControllerCommands;
   private final QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings;
   private final QuadrupedTaskSpaceController taskSpaceController;

   // planning
   private static int NUMBER_OF_PREVIEW_STEPS = 16;
   private double bodyYawSetpoint;
   private final GroundPlaneEstimator groundPlaneEstimator;
   private final QuadrantDependentList<FramePoint> groundPlanePositions;
   private final QuadrupedXGaitSettings xGaitSettings;
   private final QuadrupedXGaitPlanner xGaitStepPlanner;
   private final ArrayList<QuadrupedTimedStep> xGaitPreviewSteps;
   private final QuadrupedTimedStepPressurePlanner timedStepPressurePlanner;
   private final QuadrantDependentList<FrameVector> timedStepAdjustmentAtContactSwitch;

   // state machine
   public enum XGaitState
   {
      INITIAL, ACTIVE
   }
   public enum XGaitEvent
   {
      TIMEOUT
   }
   private final FiniteStateMachine<XGaitState, XGaitEvent> xGaitStateMachine;

   public QuadrupedDcmBasedXGaitController(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedForceControllerToolbox controllerToolbox,
         QuadrupedControllerInputProviderInterface inputProvider, QuadrupedXGaitSettingsProvider settingsProvider)
   {
      this.inputProvider = inputProvider;
      this.settingsProvider = settingsProvider;
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
      xGaitStepPlanner = new QuadrupedXGaitPlanner();
      xGaitPreviewSteps = new ArrayList<>(NUMBER_OF_PREVIEW_STEPS);
      for (int i = 0; i < NUMBER_OF_PREVIEW_STEPS; i++)
      {
         xGaitPreviewSteps.add(new QuadrupedTimedStep());
      }
      xGaitSettings = new QuadrupedXGaitSettings();
      timedStepPressurePlanner = new QuadrupedTimedStepPressurePlanner(NUMBER_OF_PREVIEW_STEPS + 4);
      timedStepAdjustmentAtContactSwitch = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         timedStepAdjustmentAtContactSwitch.set(robotQuadrant, new FrameVector());
      }

      // state machine
      FiniteStateMachineBuilder<XGaitState, XGaitEvent> xgaitStateMachineBuilder = new FiniteStateMachineBuilder<>(XGaitState.class, XGaitEvent.class, "XGaitState", registry);
      xgaitStateMachineBuilder.addState(XGaitState.INITIAL, new InitialXGaitState());
      xgaitStateMachineBuilder.addState(XGaitState.ACTIVE, new ActiveXGaitState());
      xgaitStateMachineBuilder.addTransition(XGaitEvent.TIMEOUT, XGaitState.INITIAL, XGaitState.ACTIVE);
      xGaitStateMachine = xgaitStateMachineBuilder.build(XGaitState.INITIAL);
      runtimeEnvironment.getParentRegistry().addChild(registry);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   private void updateGains()
   {
      dcmPositionController.getGains().setProportionalGains(dcmPositionProportionalGainsParameter.get());
      dcmPositionController.getGains().setIntegralGains(dcmPositionIntegralGainsParameter.get(), dcmPositionMaxIntegralErrorParameter.get());
      dcmPositionController.getGains().setDerivativeGains(dcmPositionDerivativeGainsParameter.get());
      comPositionController.getGains().setProportionalGains(comPositionProportionalGainsParameter.get());
      comPositionController.getGains().setIntegralGains(comPositionIntegralGainsParameter.get(), comPositionMaxIntegralErrorParameter.get());
      comPositionController.getGains().setDerivativeGains(comPositionDerivativeGainsParameter.get());
      bodyOrientationController.getGains().setProportionalGains(bodyOrientationProportionalGainsParameter.get());
      bodyOrientationController.getGains().setIntegralGains(bodyOrientationIntegralGainsParameter.get(), bodyOrientationMaxIntegralErrorParameter.get());
      bodyOrientationController.getGains().setDerivativeGains(bodyOrientationDerivativeGainsParameter.get());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointDamping(jointDampingParameter.get());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitDamping(jointPositionLimitDampingParameter.get());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitStiffness(jointPositionLimitStiffnessParameter.get());
      taskSpaceControllerSettings.getContactForceLimits().setPressureLowerLimit(contactPressureLowerLimitParameter.get());
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
   }

   private void updateSetpoints()
   {
      // update state machines
      xGaitStateMachine.process();

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
      comPositionControllerSetpoints.getComForceFeedforward().setZ(comPositionGravityCompensationParameter.get() * mass * gravity);
      comPositionController.compute(taskSpaceControllerCommands.getComForce(), comPositionControllerSetpoints, taskSpaceEstimates);

      // update desired body orientation, angular velocity, and torque
      if (xGaitStateMachine.getState() != XGaitState.INITIAL)
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
      timedStepController.compute(taskSpaceControllerSettings.getContactState(), taskSpaceControllerCommands.getSoleForce(), taskSpaceEstimates);

      // update joint setpoints
      taskSpaceController.compute(taskSpaceControllerSettings, taskSpaceControllerCommands);
   }

   private void updateSettings()
   {
      settingsProvider.getXGaitSettings(xGaitSettings);

      // increase stance dimensions to prevent self collisions
      double strideLength = Math.abs(2 * inputProvider.getPlanarVelocityInput().getX() * xGaitSettings.getStepDuration());
      double strideWidth = Math.abs(2 * inputProvider.getPlanarVelocityInput().getY() * xGaitSettings.getStepDuration());
      strideLength += Math.abs(xGaitSettings.getStanceWidth() / 2 * Math.sin(2 * inputProvider.getPlanarVelocityInput().getZ() * xGaitSettings.getStepDuration()));
      strideWidth += Math.abs(xGaitSettings.getStanceLength() / 2 * Math.sin(2 * inputProvider.getPlanarVelocityInput().getZ() * xGaitSettings.getStepDuration()));
      xGaitSettings.setStanceLength(Math.max(xGaitSettings.getStanceLength(), strideLength / 2 + footholdDistanceLowerLimitParameter.get()));
      xGaitSettings.setStanceWidth(Math.max(xGaitSettings.getStanceWidth(), strideWidth / 2 + footholdDistanceLowerLimitParameter.get()));
   }

   @Override public ControllerEvent process()
   {
      updateGains();
      updateSettings();
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
      comPositionControllerSetpoints.initialize(taskSpaceEstimates);
      comPositionController.reset();
      bodyOrientationControllerSetpoints.initialize(taskSpaceEstimates);
      bodyOrientationController.reset();
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
      updateSettings();
      xGaitStateMachine.reset();
   }

   @Override public void onExit()
   {
      xGaitStateMachine.reset();
      timedStepController.removeSteps();
   }

   private class InitialXGaitState implements FiniteStateMachineState<XGaitEvent>
   {
      double initialTransitionTime;
      private RobotQuadrant initialQuadrant;
      private final FramePoint initialSupportCentroid;
      private final ThreeDoFMinimumJerkTrajectory transitionDcmTrajectory;
      private final PiecewiseReverseDcmTrajectory reverseDcmTrajectory;
      private final FramePoint reverseDcmPositionAtSoS;

      public InitialXGaitState()
      {
         initialSupportCentroid = new FramePoint();
         transitionDcmTrajectory = new ThreeDoFMinimumJerkTrajectory();
         reverseDcmTrajectory = new PiecewiseReverseDcmTrajectory(2 * xGaitPreviewSteps.size() + 1, gravity, dcmPositionController.getComHeight());
         reverseDcmPositionAtSoS = new FramePoint();
      }

      @Override public void onEntry()
      {
         double currentTime = robotTimestamp.getDoubleValue();

         initialSupportCentroid.setToZero(supportFrame);
         initialQuadrant = (xGaitSettings.getEndPhaseShift() < 90) ? RobotQuadrant.HIND_LEFT : RobotQuadrant.FRONT_LEFT;
         initialTransitionTime = currentTime + initialTransitionDurationParameter.get();

         // initialize ground plane
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            groundPlanePositions.get(robotQuadrant).setIncludingFrame(taskSpaceEstimates.getSolePosition(robotQuadrant));
            groundPlanePositions.get(robotQuadrant).changeFrame(ReferenceFrame.getWorldFrame());
         }
         groundPlaneEstimator.compute(groundPlanePositions);

         // initialize step plan
         xGaitStepPlanner.computeInitialPlan(xGaitPreviewSteps, inputProvider.getPlanarVelocityInput(),
               initialQuadrant, initialSupportCentroid, initialTransitionTime, bodyYawSetpoint, xGaitSettings);
         for (int i = 0; i < xGaitPreviewSteps.size(); i++)
         {
            timedStepController.addStep(xGaitPreviewSteps.get(i));
            groundPlaneEstimator.projectZ(timedStepController.getQueue().get(i).getGoalPosition());
         }

         // initialize dcm height
         dcmPositionController.setComHeight(inputProvider.getComPositionInput().getZ());

         // compute reverse dcm trajectory
         int nIntervals = timedStepPressurePlanner.compute(xGaitPreviewSteps, taskSpaceEstimates.getSolePosition(), taskSpaceControllerSettings.getContactState(), currentTime);
         reverseDcmTrajectory.setComHeight(dcmPositionController.getComHeight());
         reverseDcmTrajectory.initializeTrajectory(nIntervals, timedStepPressurePlanner.getTimeAtStartOfInterval(), timedStepPressurePlanner.getCenterOfPressureAtStartOfInterval(), timedStepPressurePlanner.getTimeAtStartOfInterval(nIntervals - 1),
               timedStepPressurePlanner.getCenterOfPressureAtStartOfInterval(nIntervals - 1));
         reverseDcmTrajectory.computeTrajectory(initialTransitionTime);
         reverseDcmTrajectory.getPosition(reverseDcmPositionAtSoS);
         reverseDcmPositionAtSoS.changeFrame(worldFrame);

         // compute transition dcm trajectory
         transitionDcmTrajectory.initializeTrajectory(dcmPositionEstimate, reverseDcmPositionAtSoS, currentTime, initialTransitionTime);
      }

      @Override public XGaitEvent process()
      {
         // compute nominal dcm trajectory
         transitionDcmTrajectory.computeTrajectory(robotTimestamp.getDoubleValue());
         transitionDcmTrajectory.getPosition(dcmPositionControllerSetpoints.getDcmPosition());
         transitionDcmTrajectory.getVelocity(dcmPositionControllerSetpoints.getDcmVelocity());

         if (robotTimestamp.getDoubleValue() > initialTransitionTime + controlDT)
            return XGaitEvent.TIMEOUT;
         else
            return null;
      }

      @Override public void onExit()
      {
      }
   }

   private class ActiveXGaitState implements FiniteStateMachineState<XGaitEvent>, QuadrupedTimedStepTransitionCallback
   {
      private final PiecewiseForwardDcmTrajectory forwardDcmTrajectory;
      private EndDependentList<QuadrupedTimedStep> latestSteps;
      private final FrameVector stepAdjustment;

      public ActiveXGaitState()
      {
         forwardDcmTrajectory = new PiecewiseForwardDcmTrajectory(2 * (xGaitPreviewSteps.size() + 4) + 1, gravity, dcmPositionController.getComHeight());
         latestSteps = new EndDependentList<>();
         for (RobotEnd robotEnd : RobotEnd.values)
         {
            latestSteps.set(robotEnd, new QuadrupedTimedStep());
         }
         stepAdjustment = new FrameVector();
      }

      @Override public void onEntry()
      {
         timedStepController.registerStepTransitionCallback(this);

         // initialize latest steps
         for (RobotEnd robotEnd : RobotEnd.values)
         {
            latestSteps.get(robotEnd).set(timedStepController.getEarliestStep(robotEnd));
         }
      }

      @Override public void onLiftOff(RobotQuadrant thisStepQuadrant, QuadrantDependentList<ContactState> thisContactState)
      {
         // update ground plane estimate
         groundPlanePositions.get(thisStepQuadrant).setIncludingFrame(taskSpaceEstimates.getSolePosition(thisStepQuadrant));
         groundPlanePositions.get(thisStepQuadrant).changeFrame(ReferenceFrame.getWorldFrame());
         groundPlaneEstimator.compute(groundPlanePositions);

         // update latest step
         RobotEnd thisStepEnd = thisStepQuadrant.getEnd();
         QuadrupedTimedStep thisStep = timedStepController.getEarliestStep(thisStepQuadrant);
         latestSteps.get(thisStepEnd).set(thisStep);
      }

      @Override public void onTouchDown(RobotQuadrant thisStepQuadrant, QuadrantDependentList<ContactState> thisContactState)
      {
      }

      @Override public XGaitEvent process()
      {
         double currentTime = robotTimestamp.getDoubleValue();

         // initialize dcm height
         dcmPositionController.setComHeight(inputProvider.getComPositionInput().getZ());

         // compute step plan
         xGaitStepPlanner.computeOnlinePlan(xGaitPreviewSteps, latestSteps, inputProvider.getPlanarVelocityInput(), currentTime, bodyYawSetpoint, xGaitSettings);
         timedStepController.removeSteps();
         for (RobotEnd robotEnd : RobotEnd.values)
         {
            timedStepController.addStep(latestSteps.get(robotEnd));
         }
         for (int i = 0; i < xGaitPreviewSteps.size(); i++)
         {
            timedStepController.addStep(xGaitPreviewSteps.get(i));
         }

         // compute step adjustment
         computeStepAdjustmentBasedOnDcm(stepAdjustment, timedStepController.getQueue(), taskSpaceEstimates.getSolePosition(), taskSpaceControllerSettings.getContactState(), dcmPositionEstimate, currentTime, dcmPositionController.getNaturalFrequency());
         for (RobotEnd robotEnd : RobotEnd.values)
         {
            latestSteps.get(robotEnd).getGoalPosition().add(stepAdjustment.getVector());
            groundPlaneEstimator.projectZ(latestSteps.get(robotEnd).getGoalPosition());
         }
         for (int i = 0; i < timedStepController.getQueue().size(); i++)
         {
            timedStepController.getQueue().get(i).getGoalPosition().add(stepAdjustment.getVector());
            groundPlaneEstimator.projectZ(timedStepController.getQueue().get(i).getGoalPosition());
         }

         // compute nominal dcm trajectory
         int nIntervals = timedStepPressurePlanner.compute(timedStepController.getQueue(), taskSpaceEstimates.getSolePosition(), taskSpaceControllerSettings.getContactState(), currentTime);
         forwardDcmTrajectory.setComHeight(dcmPositionController.getComHeight());
         forwardDcmTrajectory.initializeTrajectory(nIntervals, timedStepPressurePlanner.getTimeAtStartOfInterval(), timedStepPressurePlanner.getCenterOfPressureAtStartOfInterval(), dcmPositionEstimate);
         forwardDcmTrajectory.computeTrajectory(currentTime);
         forwardDcmTrajectory.getPosition(dcmPositionControllerSetpoints.getDcmPosition());
         forwardDcmTrajectory.getVelocity(dcmPositionControllerSetpoints.getDcmVelocity());

         return null;
      }

      @Override public void onExit()
      {
         timedStepController.registerStepTransitionCallback(null);
      }

      private DenseMatrix64F x0 = new DenseMatrix64F(100, 1);
      private DenseMatrix64F b = new DenseMatrix64F(100, 1);
      private DenseMatrix64F c = new DenseMatrix64F(100, 1);
      private DenseMatrix64F temp = new DenseMatrix64F(1, 1);

      private void computeStepAdjustmentBasedOnDcm(FrameVector stepAdjustment, PreallocatedQueue<QuadrupedTimedStep> queuedSteps, QuadrantDependentList<FramePoint> currentSolePosition, QuadrantDependentList<ContactState> currentContactState, FramePoint currentDcmEstimate, double currentTime, double naturalFrequency)
      {
         stepAdjustment.changeFrame(worldFrame);
         currentDcmEstimate.changeFrame(worldFrame);

         int nIntervals = timedStepPressurePlanner.compute(queuedSteps, currentSolePosition, currentContactState, currentTime);
         x0.reshape(nIntervals, 1);
         b.reshape(nIntervals, 1);
         c.reshape(nIntervals, 1);

         for (Direction direction : Direction.values2D())
         {
            for (int i = 0; i < nIntervals; i++)
            {
               timedStepPressurePlanner.getCenterOfPressureAtStartOfInterval(i).changeFrame(worldFrame);
               x0.set(i, 0, timedStepPressurePlanner.getCenterOfPressureAtStartOfInterval(i).get(direction));
               b.set(i, 0, timedStepPressurePlanner.getNormalizedPressureContributedByQueuedSteps(i));
            }

            for (int i = nIntervals - 2; i >= 0; i--)
            {
               double expn = Math.exp(naturalFrequency * (timedStepPressurePlanner.getTimeAtStartOfInterval(nIntervals - 1) - timedStepPressurePlanner.getTimeAtStartOfInterval(i + 1)));
               double expi = Math.exp(naturalFrequency * (timedStepPressurePlanner.getTimeAtStartOfInterval(i + 1) - timedStepPressurePlanner.getTimeAtStartOfInterval(i)));
               c.set(i, 0, expn * (1 - expi));
            }
            CommonOps.scale(-1, c);
            c.set(nIntervals - 1, 0, 1);

            CommonOps.multTransA(c, x0, temp);
            double ctx0 = temp.get(0, 0);

            CommonOps.multTransA(c, b, temp);
            double ctb = temp.get(0, 0);

            double previewTime = timedStepPressurePlanner.getTimeAtStartOfInterval(nIntervals - 1) - timedStepPressurePlanner.getTimeAtStartOfInterval(0);
            double y0 = Math.exp(naturalFrequency * previewTime) * currentDcmEstimate.get(direction);

            double u = (-ctx0 + y0) / ctb;
            stepAdjustment.set(direction, u);
         }
      }
   }
}
