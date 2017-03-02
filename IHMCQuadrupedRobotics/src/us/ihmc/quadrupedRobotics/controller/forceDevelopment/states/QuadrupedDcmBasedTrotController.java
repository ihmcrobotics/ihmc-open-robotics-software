package us.ihmc.quadrupedRobotics.controller.forceDevelopment.states;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.DivergentComponentOfMotionController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.DivergentComponentOfMotionEstimator;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.LinearInvertedPendulumModel;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedBodyOrientationController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedComPositionController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimator;
import us.ihmc.quadrupedRobotics.controller.forceDevelopment.QuadrupedTimedStepController;
import us.ihmc.quadrupedRobotics.estimator.GroundPlaneEstimator;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.trajectory.PiecewiseForwardDcmTrajectory;
import us.ihmc.quadrupedRobotics.planning.trajectory.PiecewisePeriodicDcmTrajectory;
import us.ihmc.quadrupedRobotics.planning.trajectory.ThreeDoFMinimumJerkTrajectory;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPlanarVelocityInputProvider;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPostureInputProviderInterface;
import us.ihmc.quadrupedRobotics.providers.QuadrupedXGaitSettingsInputProvider;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.dataStructures.parameter.DoubleArrayParameter;
import us.ihmc.robotics.dataStructures.parameter.DoubleParameter;
import us.ihmc.robotics.dataStructures.parameter.ParameterFactory;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachine;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachineBuilder;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachineState;

public class QuadrupedDcmBasedTrotController implements QuadrupedController
{
   private final QuadrupedPostureInputProviderInterface inputProvider;
   private final QuadrupedPlanarVelocityInputProvider planarVelocityProvider;
   private final QuadrupedXGaitSettingsInputProvider xGaitSettingsProvider;
   private final DoubleYoVariable robotTimestamp;
   private final double controlDT;
   private final double gravity;
   private final double mass;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   // parameters
   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final DoubleParameter jointDampingParameter = parameterFactory.createDouble("jointDamping", 2.0);
   private final DoubleArrayParameter bodyOrientationProportionalGainsParameter = parameterFactory
         .createDoubleArray("bodyOrientationProportionalGains", 5000, 5000, 5000);
   private final DoubleArrayParameter bodyOrientationDerivativeGainsParameter = parameterFactory
         .createDoubleArray("bodyOrientationDerivativeGains", 750, 750, 750);
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
   private final DoubleParameter vrpPositionRateLimitParameter = parameterFactory.createDouble("vrpPositionRateLimit", Double.MAX_VALUE);
   private final DoubleParameter quadSupportDurationParameter = parameterFactory.createDouble("quadSupportDuration", 1.0);
   private final DoubleParameter doubleSupportDurationParameter = parameterFactory.createDouble("doubleSupportDuration", 0.33);
   private final DoubleParameter stanceWidthNominalParameter = parameterFactory.createDouble("stanceWidthNominal", 0.35);
   private final DoubleParameter stanceLengthNominalParameter = parameterFactory.createDouble("stanceLengthNominal", 1.1);
   private final DoubleParameter stepGroundClearanceParameter = parameterFactory.createDouble("stepGroundClearance", 0.1);

   // frames
   private final ReferenceFrame supportFrame;
   private final ReferenceFrame worldFrame;

   // feedback controllers
   private final LinearInvertedPendulumModel lipModel;
   private final FramePoint dcmPositionEstimate;
   private final DivergentComponentOfMotionEstimator dcmPositionEstimator;
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
   private double bodyYawSetpoint;
   private final GroundPlaneEstimator groundPlaneEstimator;
   private final QuadrantDependentList<FramePoint> groundPlanePositions;
   private final PiecewisePeriodicDcmTrajectory nominalPeriodicDcmTrajectory;

   // state machine
   public enum TrotState
   {
      QUAD_SUPPORT, HIND_LEFT_FRONT_RIGHT_SUPPORT, HIND_RIGHT_FRONT_LEFT_SUPPORT
   }

   public enum TrotEvent
   {
      TIMEOUT
   }

   private final FiniteStateMachine<TrotState, TrotEvent> trotStateMachine;

   public QuadrupedDcmBasedTrotController(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedForceControllerToolbox controllerToolbox,
         QuadrupedPostureInputProviderInterface inputProvider, QuadrupedPlanarVelocityInputProvider planarVelocityProvider,
         QuadrupedXGaitSettingsInputProvider xGaitSettingsInputProvider)

   {
      this.inputProvider = inputProvider;
      this.planarVelocityProvider = planarVelocityProvider;
      this.xGaitSettingsProvider = xGaitSettingsInputProvider;
      this.robotTimestamp = runtimeEnvironment.getRobotTimestamp();
      this.controlDT = runtimeEnvironment.getControlDT();
      this.gravity = 9.81;
      this.mass = runtimeEnvironment.getFullRobotModel().getTotalMass();

      // frames
      QuadrupedReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      worldFrame = ReferenceFrame.getWorldFrame();

      // feedback controllers
      lipModel = controllerToolbox.getLinearInvertedPendulumModel();
      dcmPositionEstimate = new FramePoint();
      dcmPositionEstimator = controllerToolbox.getDcmPositionEstimator();
      dcmPositionControllerSetpoints = new DivergentComponentOfMotionController.Setpoints();
      dcmPositionController = controllerToolbox.getDcmPositionController();
      comPositionControllerSetpoints = new QuadrupedComPositionController.Setpoints();
      comPositionController = controllerToolbox.getComPositionController();
      bodyOrientationControllerSetpoints = new QuadrupedBodyOrientationController.Setpoints();
      bodyOrientationController = controllerToolbox.getBodyOrientationController();
      timedStepController = new QuadrupedTimedStepController(controllerToolbox.getSolePositionController(), runtimeEnvironment.getRobotTimestamp(), registry,
            runtimeEnvironment.getGraphicsListRegistry());

      // task space controllers
      taskSpaceEstimates = new QuadrupedTaskSpaceEstimator.Estimates();
      taskSpaceEstimator = controllerToolbox.getTaskSpaceEstimator();
      taskSpaceControllerCommands = new QuadrupedTaskSpaceController.Commands();
      taskSpaceControllerSettings = new QuadrupedTaskSpaceController.Settings();
      taskSpaceController = controllerToolbox.getTaskSpaceController();

      // planning
      groundPlaneEstimator = controllerToolbox.getGroundPlaneEstimator();
      groundPlanePositions = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         groundPlanePositions.set(robotQuadrant, new FramePoint());
      }
      nominalPeriodicDcmTrajectory = new PiecewisePeriodicDcmTrajectory(1, gravity, inputProvider.getComPositionInput().getZ());

      // state machine
      FiniteStateMachineBuilder<TrotState, TrotEvent> stateMachineBuilder = new FiniteStateMachineBuilder<>(TrotState.class, TrotEvent.class, "TrotState",
            registry);
      stateMachineBuilder.addState(TrotState.QUAD_SUPPORT, new QuadSupportState());
      stateMachineBuilder.addState(TrotState.HIND_LEFT_FRONT_RIGHT_SUPPORT, new DoubleSupportState(RobotQuadrant.HIND_LEFT, RobotQuadrant.FRONT_RIGHT));
      stateMachineBuilder.addState(TrotState.HIND_RIGHT_FRONT_LEFT_SUPPORT, new DoubleSupportState(RobotQuadrant.HIND_RIGHT, RobotQuadrant.FRONT_LEFT));
      stateMachineBuilder.addTransition(TrotEvent.TIMEOUT, TrotState.QUAD_SUPPORT, TrotState.HIND_LEFT_FRONT_RIGHT_SUPPORT);
      stateMachineBuilder.addTransition(TrotEvent.TIMEOUT, TrotState.HIND_LEFT_FRONT_RIGHT_SUPPORT, TrotState.HIND_RIGHT_FRONT_LEFT_SUPPORT);
      stateMachineBuilder.addTransition(TrotEvent.TIMEOUT, TrotState.HIND_RIGHT_FRONT_LEFT_SUPPORT, TrotState.HIND_LEFT_FRONT_RIGHT_SUPPORT);
      trotStateMachine = stateMachineBuilder.build(TrotState.QUAD_SUPPORT);

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
      dcmPositionEstimator.compute(dcmPositionEstimate, taskSpaceEstimates.getComVelocity());

      // update ground plane estimate
      groundPlaneEstimator.compute(groundPlanePositions);
   }

   private void updateSetpoints()
   {
      // update desired horizontal com forces
      trotStateMachine.process();
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
      if (trotStateMachine.getState() != TrotState.QUAD_SUPPORT)
      {
         bodyYawSetpoint += planarVelocityProvider.get().getZ() * controlDT;
      }
      bodyOrientationControllerSetpoints.getBodyOrientation().changeFrame(worldFrame);
      bodyOrientationControllerSetpoints.getBodyOrientation().setYawPitchRoll(bodyYawSetpoint,
                                                                              inputProvider.getBodyOrientationInput().getPitch() + groundPlaneEstimator.getPitch(bodyYawSetpoint),
                                                                              inputProvider.getBodyOrientationInput().getRoll());
      bodyOrientationControllerSetpoints.getBodyAngularVelocity().setToZero();
      bodyOrientationControllerSetpoints.getComTorqueFeedforward().setToZero();
      bodyOrientationController.compute(taskSpaceControllerCommands.getComTorque(), bodyOrientationControllerSetpoints, taskSpaceEstimates);

      // update desired contact state and sole forces
      timedStepController.compute(taskSpaceControllerSettings.getContactState(), taskSpaceControllerSettings.getContactForceLimits(),
            taskSpaceControllerCommands.getSoleForce(), taskSpaceEstimates);

      // update joint setpoints
      taskSpaceController.compute(taskSpaceControllerSettings, taskSpaceControllerCommands);
   }

   @Override
   public ControllerEvent process()
   {
      updateEstimates();
      updateSetpoints();
      return null;
   }

   @Override
   public void onEntry()
   {
      // initialize estimates
      lipModel.setComHeight(inputProvider.getComPositionInput().getZ());
      updateEstimates();

      // initialize feedback controllers
      dcmPositionControllerSetpoints.initialize(dcmPositionEstimate);
      dcmPositionController.reset();
      dcmPositionController.setVrpPositionRateLimit(vrpPositionRateLimitParameter.get());
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
      trotStateMachine.reset();
   }

   @Override
   public void onExit()
   {
      trotStateMachine.reset();
      timedStepController.removeSteps();
   }

   private void computeNominalCmpPositions(RobotQuadrant hindSupportQuadrant, RobotQuadrant frontSupportQuadrant, FramePoint nominalCmpPositionAtSoS,
         FramePoint nominalCmpPositionAtEoS)
   {
      taskSpaceEstimates.getSolePosition(hindSupportQuadrant).changeFrame(worldFrame);
      taskSpaceEstimates.getSolePosition(frontSupportQuadrant).changeFrame(worldFrame);
      nominalCmpPositionAtSoS.setToZero(worldFrame);
      nominalCmpPositionAtSoS.add(taskSpaceEstimates.getSolePosition(hindSupportQuadrant));
      nominalCmpPositionAtSoS.add(taskSpaceEstimates.getSolePosition(frontSupportQuadrant));
      nominalCmpPositionAtSoS.scale(0.5);

      double bodyYaw = bodyYawSetpoint + planarVelocityProvider.get().getZ() * doubleSupportDurationParameter.get();
      double xStride = planarVelocityProvider.get().getX() * doubleSupportDurationParameter.get();
      double yStride = planarVelocityProvider.get().getY() * doubleSupportDurationParameter.get();
      double xOffset = Math.cos(bodyYaw) * xStride - Math.sin(bodyYaw) * yStride;
      double yOffset = Math.sin(bodyYaw) * xStride + Math.cos(bodyYaw) * yStride;
      nominalCmpPositionAtEoS.setIncludingFrame(nominalCmpPositionAtSoS);
      nominalCmpPositionAtEoS.changeFrame(worldFrame);
      nominalCmpPositionAtEoS.add(xOffset, yOffset, 0.0);
   }

   private void computeNominalDcmPositions(FramePoint nominalCmpPositionAtSoS, FramePoint nominalCmpPositionAtEoS, FramePoint nominalDcmPositionAtSoS,
         FramePoint nominalDcmPositionAtEoS)
   {
      double timeAtEoS = doubleSupportDurationParameter.get();
      double relativeYawAtEoS = planarVelocityProvider.get().getZ() * timeAtEoS;
      nominalPeriodicDcmTrajectory.setComHeight(inputProvider.getComPositionInput().getZ());
      nominalPeriodicDcmTrajectory.initializeTrajectory(0.0, nominalCmpPositionAtSoS, timeAtEoS, nominalCmpPositionAtEoS, relativeYawAtEoS);
      nominalPeriodicDcmTrajectory.computeTrajectory(0.0);
      nominalPeriodicDcmTrajectory.getPosition(nominalDcmPositionAtSoS);
      nominalPeriodicDcmTrajectory.computeTrajectory(timeAtEoS);
      nominalPeriodicDcmTrajectory.getPosition(nominalDcmPositionAtEoS);
   }

   private void computeFootholdPosition(RobotQuadrant robotQuadrant, FramePoint cmpPosition, double bodyYaw, FramePoint footholdPosition)
   {
      // compute foothold position based on the nominal stance and desired cmp
      taskSpaceEstimates.getSolePosition(robotQuadrant).changeFrame(worldFrame);
      double xStance = robotQuadrant.getEnd().negateIfHindEnd(stanceLengthNominalParameter.get() / 2);
      double yStance = robotQuadrant.getSide().negateIfRightSide(stanceWidthNominalParameter.get() / 2);
      double xOffset = Math.cos(bodyYaw) * xStance - Math.sin(bodyYaw) * yStance;
      double yOffset = Math.sin(bodyYaw) * xStance + Math.cos(bodyYaw) * yStance;
      footholdPosition.setIncludingFrame(cmpPosition);
      footholdPosition.changeFrame(worldFrame);
      footholdPosition.add(xOffset, yOffset, 0.0);
      groundPlaneEstimator.projectZ(footholdPosition);
   }

   private class QuadSupportState implements FiniteStateMachineState<TrotEvent>
   {
      private final ThreeDoFMinimumJerkTrajectory dcmTrajectory;
      private final FramePoint cmpPositionAtSoSNominal;
      private final FramePoint cmpPositionAtEoSNominal;
      private final FramePoint dcmPositionAtSoSNominal;
      private final FramePoint dcmPositionAtEoSNominal;
      private final TimeInterval timeInterval;

      public QuadSupportState()
      {
         dcmTrajectory = new ThreeDoFMinimumJerkTrajectory();
         cmpPositionAtSoSNominal = new FramePoint();
         cmpPositionAtEoSNominal = new FramePoint();
         dcmPositionAtSoSNominal = new FramePoint();
         dcmPositionAtEoSNominal = new FramePoint();
         timeInterval = new TimeInterval();
      }

      @Override
      public void onEntry()
      {
         timeInterval.setInterval(robotTimestamp.getDoubleValue(), robotTimestamp.getDoubleValue() + quadSupportDurationParameter.get());

         // initialize lip com height
         lipModel.setComHeight(inputProvider.getComPositionInput().getZ());

         // compute desired dcm position at start of step
         computeNominalCmpPositions(RobotQuadrant.HIND_LEFT, RobotQuadrant.FRONT_RIGHT, cmpPositionAtSoSNominal, cmpPositionAtEoSNominal);
         computeNominalDcmPositions(cmpPositionAtSoSNominal, cmpPositionAtEoSNominal, dcmPositionAtSoSNominal, dcmPositionAtEoSNominal);

         // compute desired dcm trajectory
         dcmTrajectory.initializeTrajectory(dcmPositionEstimate, dcmPositionAtSoSNominal, timeInterval);

         // initialize ground plane points
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            groundPlanePositions.get(robotQuadrant).setIncludingFrame(taskSpaceEstimates.getSolePosition(robotQuadrant));
            groundPlanePositions.get(robotQuadrant).changeFrame(ReferenceFrame.getWorldFrame());
         }
      }

      @Override
      public TrotEvent process()
      {
         double currentTime = robotTimestamp.getDoubleValue();

         // compute dcm setpoint
         dcmTrajectory.computeTrajectory(currentTime);
         dcmTrajectory.getPosition(dcmPositionControllerSetpoints.getDcmPosition());
         dcmTrajectory.getVelocity(dcmPositionControllerSetpoints.getDcmVelocity());

         // trigger touch down event
         if (currentTime > timeInterval.getEndTime())
            return TrotEvent.TIMEOUT;
         else
            return null;
      }

      @Override
      public void onExit()
      {
      }
   }

   private class DoubleSupportState implements FiniteStateMachineState<TrotEvent>
   {
      private final RobotQuadrant supportQuadrants[];
      private final RobotQuadrant swingQuadrants[];
      private final PiecewiseForwardDcmTrajectory dcmTrajectory;
      private final FramePoint cmpPositionAtSoSNominal;
      private final FramePoint cmpPositionAtEoSNominal;
      private final FramePoint dcmPositionAtSoSNominal;
      private final FramePoint dcmPositionAtEoSNominal;
      private final FramePoint cmpPositionAtEoS;
      private final FramePoint dcmPositionAtEoS;
      private final FramePoint footholdPosition;
      private final QuadrupedTimedStep timedStep;
      private final Point3D timedStepGoalPosition;
      private final QuadrantDependentList<Point3D> timedStepGoalPositionAtSoS;

      public DoubleSupportState(RobotQuadrant hindSupportQuadrant, RobotQuadrant frontSupportQuadrant)
      {
         supportQuadrants = new RobotQuadrant[] {hindSupportQuadrant, frontSupportQuadrant};
         swingQuadrants = new RobotQuadrant[] {hindSupportQuadrant.getAcrossBodyQuadrant(), frontSupportQuadrant.getAcrossBodyQuadrant()};
         dcmTrajectory = new PiecewiseForwardDcmTrajectory(1, gravity, lipModel.getComHeight());
         cmpPositionAtSoSNominal = new FramePoint();
         cmpPositionAtEoSNominal = new FramePoint();
         dcmPositionAtSoSNominal = new FramePoint();
         dcmPositionAtEoSNominal = new FramePoint();
         cmpPositionAtEoS = new FramePoint();
         dcmPositionAtEoS = new FramePoint();
         footholdPosition = new FramePoint();
         timedStep = new QuadrupedTimedStep();
         timedStepGoalPosition = new Point3D();
         timedStepGoalPositionAtSoS = new QuadrantDependentList<>();
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            timedStepGoalPositionAtSoS.set(robotQuadrant, new Point3D());
         }
      }

      @Override
      public void onEntry()
      {
         double initialTime = robotTimestamp.getDoubleValue();

         // initialize dcm controller height
         lipModel.setComHeight(inputProvider.getComPositionInput().getZ());

         // compute desired dcm position at end of step
         computeNominalCmpPositions(supportQuadrants[0], supportQuadrants[1], cmpPositionAtSoSNominal, cmpPositionAtEoSNominal);
         computeNominalDcmPositions(cmpPositionAtSoSNominal, cmpPositionAtEoSNominal, dcmPositionAtSoSNominal, dcmPositionAtEoSNominal);

         // compute desired dcm trajectory
         dcmPositionEstimate.changeFrame(worldFrame);
         dcmTrajectory.setComHeight(lipModel.getComHeight());
         dcmTrajectory.initializeTrajectory(initialTime, cmpPositionAtSoSNominal, dcmPositionEstimate);
         dcmTrajectory.computeTrajectory(initialTime + doubleSupportDurationParameter.get());
         dcmTrajectory.getPosition(dcmPositionAtEoS);

         // compute desired cmp position at end of step
         cmpPositionAtEoS.set(dcmPositionAtEoS);
         cmpPositionAtEoS.sub(dcmPositionAtEoSNominal);
         cmpPositionAtEoS.add(cmpPositionAtEoSNominal);
         cmpPositionAtEoS.setZ(cmpPositionAtEoSNominal.getZ());

         // compute desired body yaw at end of step
         double bodyYawAtSoS = bodyYawSetpoint;
         double bodyYawAtEoS = bodyYawAtSoS + planarVelocityProvider.get().getZ() * doubleSupportDurationParameter.get();

         for (int i = 0; i < 2; i++)
         {
            // compute foothold position to track the periodic dcm trajectory using deadbeat control
            computeFootholdPosition(swingQuadrants[i], cmpPositionAtEoS, bodyYawAtEoS, footholdPosition);

            // trigger step
            timedStep.setRobotQuadrant(swingQuadrants[i]);
            timedStep.setGroundClearance(stepGroundClearanceParameter.get());
            timedStep.getTimeInterval().setStartTime(initialTime);
            timedStep.getTimeInterval().setEndTime(initialTime + doubleSupportDurationParameter.get());
            timedStep.setGoalPosition(footholdPosition);
            timedStepController.addStep(timedStep);
            timedStep.getGoalPosition(timedStepGoalPositionAtSoS.get(swingQuadrants[i]));

            // initialize ground plane points
            groundPlanePositions.get(swingQuadrants[i]).setIncludingFrame(taskSpaceEstimates.getSolePosition(swingQuadrants[i]));
            groundPlanePositions.get(swingQuadrants[i]).changeFrame(ReferenceFrame.getWorldFrame());
         }
      }

      @Override
      public TrotEvent process()
      {
         double currentTime = robotTimestamp.getDoubleValue();

         // compute dcm setpoint
         dcmTrajectory.computeTrajectory(currentTime);
         dcmTrajectory.getPosition(dcmPositionControllerSetpoints.getDcmPosition());
         dcmTrajectory.getVelocity(dcmPositionControllerSetpoints.getDcmVelocity());

         // adjust swing foot goal position based on dcm tracking error
         FramePoint dcmPositionSetpoint = dcmPositionControllerSetpoints.getDcmPosition();
         dcmPositionSetpoint.changeFrame(worldFrame);
         dcmPositionEstimate.changeFrame(worldFrame);
         for (int i = 0; i < 2; i++)
         {
            QuadrupedTimedStep step = timedStepController.getCurrentStep(swingQuadrants[i]);
            timedStepGoalPosition.set(dcmPositionEstimate.getX() - dcmPositionSetpoint.getX(), dcmPositionEstimate.getY() - dcmPositionSetpoint.getY(), 0.0);
            timedStepGoalPosition.add(timedStepGoalPositionAtSoS.get(swingQuadrants[i]));
            step.setGoalPosition(timedStepGoalPosition);
         }

         // trigger touch down event
         if (currentTime > timedStep.getTimeInterval().getEndTime())
            return TrotEvent.TIMEOUT;
         else
            return null;
      }

      @Override
      public void onExit()
      {
      }
   }
}
