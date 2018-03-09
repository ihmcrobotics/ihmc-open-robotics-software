package us.ihmc.quadrupedRobotics.controller.forceDevelopment.states;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.quadrupedRobotics.controlModules.DivergentComponentOfMotionController;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedBodyOrientationManager;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedControlManagerFactory;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.*;
import us.ihmc.quadrupedRobotics.controller.forceDevelopment.QuadrupedTimedStepController;
import us.ihmc.quadrupedRobotics.estimator.GroundPlaneEstimator;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.trajectory.PiecewiseForwardDcmTrajectory;
import us.ihmc.quadrupedRobotics.planning.trajectory.PiecewisePeriodicDcmTrajectory;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPlanarVelocityInputProvider;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPostureInputProviderInterface;
import us.ihmc.quadrupedRobotics.providers.YoQuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.ParameterizedPID3DGains;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachine;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachineBuilder;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachineState;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedDcmBasedTrotController implements QuadrupedController
{
   private final QuadrupedPostureInputProviderInterface inputProvider;
   private final QuadrupedPlanarVelocityInputProvider planarVelocityProvider;
   private final YoQuadrupedXGaitSettingsReadOnly xGaitSettingsProvider;
   private final YoDouble robotTimestamp;
   private final double controlDT;
   private final double gravity;
   private final double mass;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   // parameters
   private final ParameterizedPID3DGains comPositionGainsParameter;
   private final ParameterizedPID3DGains dcmPositionGainsParameter;

   private final DoubleParameter jointDampingParameter = new DoubleParameter("jointDamping", registry, 2.0);
   private final DoubleParameter comPositionGravityCompensationParameter = new DoubleParameter("comPositionGravityCompensation", registry, 1);
   private final DoubleParameter vrpPositionRateLimitParameter = new DoubleParameter("vrpPositionRateLimit", registry, Double.MAX_VALUE);
   private final DoubleParameter quadSupportDurationParameter = new DoubleParameter("quadSupportDuration", registry, 1.0);
   private final DoubleParameter doubleSupportDurationParameter = new DoubleParameter("doubleSupportDuration", registry, 0.33);
   private final DoubleParameter stanceWidthNominalParameter = new DoubleParameter("stanceWidthNominal", registry, 0.35);
   private final DoubleParameter stanceLengthNominalParameter = new DoubleParameter("stanceLengthNominal", registry, 1.1);
   private final DoubleParameter stepGroundClearanceParameter = new DoubleParameter("stepGroundClearance", registry, 0.1);

   // frames
   private final ReferenceFrame supportFrame;
   private final ReferenceFrame worldFrame;

   // feedback controllers
   private final LinearInvertedPendulumModel lipModel;
   private final FramePoint3D vrpPositionSetpoint = new FramePoint3D();
   private final FramePoint3D cmpPositionSetpoint = new FramePoint3D();
   private final FramePoint3D dcmPositionEstimate;
   private final DivergentComponentOfMotionEstimator dcmPositionEstimator;
   private final DivergentComponentOfMotionController dcmPositionController;
   private final QuadrupedComPositionController.Setpoints comPositionControllerSetpoints;
   private final QuadrupedComPositionController comPositionController;
   private final QuadrupedBodyOrientationManager bodyOrientationManager;
   private final QuadrupedTimedStepController timedStepController;

   // task space controller
   private final QuadrupedTaskSpaceController.Commands taskSpaceControllerCommands;
   private final QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings;
   private final QuadrupedTaskSpaceController taskSpaceController;

   // planning
   private double bodyYawSetpoint;
   private final GroundPlaneEstimator groundPlaneEstimator;
   private final QuadrantDependentList<FramePoint3D> groundPlanePositions;
   private final PiecewisePeriodicDcmTrajectory nominalPeriodicDcmTrajectory;

   private final FrameQuaternion desiredBodyOrientation = new FrameQuaternion();

   private final QuadrupedForceControllerToolbox controllerToolbox;

   private final FramePoint3D dcmPositionSetpoint = new FramePoint3D();
   private final FrameVector3D dcmVelocitySetpoint = new FrameVector3D();

   // state machine
   public enum TrotState
   {
      QUAD_SUPPORT, HIND_LEFT_FRONT_RIGHT_SUPPORT, HIND_RIGHT_FRONT_LEFT_SUPPORT
   }

   public enum TrotEvent
   {
      TIMEOUT
   }

   private final FiniteStateMachine<TrotState, TrotEvent, FiniteStateMachineState<TrotEvent>> trotStateMachine;

   public QuadrupedDcmBasedTrotController(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedForceControllerToolbox controllerToolbox,
                                          QuadrupedControlManagerFactory controlManagerFactory, QuadrupedPostureInputProviderInterface inputProvider,
                                          QuadrupedPlanarVelocityInputProvider planarVelocityProvider, YoQuadrupedXGaitSettingsReadOnly xGaitSettingsInputProvider)

   {
      this.controllerToolbox = controllerToolbox;
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
      dcmPositionEstimate = new FramePoint3D();
      comPositionControllerSetpoints = new QuadrupedComPositionController.Setpoints();
      dcmPositionEstimator = new DivergentComponentOfMotionEstimator(referenceFrames.getCenterOfMassZUpFrame(), lipModel, registry, runtimeEnvironment.getGraphicsListRegistry());
      dcmPositionController = new DivergentComponentOfMotionController(referenceFrames.getCenterOfMassZUpFrame(), runtimeEnvironment.getControlDT(), lipModel, registry);
      comPositionController = new QuadrupedComPositionController(referenceFrames.getCenterOfMassZUpFrame(), runtimeEnvironment.getControlDT(), registry);

      bodyOrientationManager = controlManagerFactory.getOrCreateBodyOrientationManager();

      QuadrantDependentList<QuadrupedSolePositionController> solePositionControllers = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         solePositionControllers.set(robotQuadrant, controlManagerFactory.getOrCreateSolePositionController(robotQuadrant));

      timedStepController = new QuadrupedTimedStepController(controllerToolbox, solePositionControllers, runtimeEnvironment.getRobotTimestamp(), registry,
            runtimeEnvironment.getGraphicsListRegistry());

      DefaultPID3DGains comPositionDefaultGains = new DefaultPID3DGains();
      comPositionDefaultGains.setProportionalGains(0.0, 0.0, 5000.0);
      comPositionDefaultGains.setDerivativeGains(0.0, 0.0, 750.0);
      comPositionGainsParameter = new ParameterizedPID3DGains("_comPosition", GainCoupling.NONE, false, comPositionDefaultGains, registry);

      DefaultPID3DGains dcmPositionDefaultGains = new DefaultPID3DGains();
      dcmPositionDefaultGains.setProportionalGains(1.0, 1.0, 0.0);
      dcmPositionGainsParameter = new ParameterizedPID3DGains("_dcmPosition", GainCoupling.NONE, false, dcmPositionDefaultGains, registry);

      // task space controllers
      taskSpaceControllerCommands = new QuadrupedTaskSpaceController.Commands();
      taskSpaceControllerSettings = new QuadrupedTaskSpaceController.Settings();
      taskSpaceController = controllerToolbox.getTaskSpaceController();

      // planning
      groundPlaneEstimator = controllerToolbox.getGroundPlaneEstimator();
      groundPlanePositions = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         groundPlanePositions.set(robotQuadrant, new FramePoint3D());
      }
      nominalPeriodicDcmTrajectory = new PiecewisePeriodicDcmTrajectory(1, gravity, inputProvider.getComPositionInput().getZ());

      // state machine
      FiniteStateMachineBuilder<TrotState, TrotEvent, FiniteStateMachineState<TrotEvent>> stateMachineBuilder = new FiniteStateMachineBuilder<>(TrotState.class, TrotEvent.class, "TrotState",
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
      controllerToolbox.update();

      // update dcm estimate
      dcmPositionEstimator.compute(dcmPositionEstimate, controllerToolbox.getTaskSpaceEstimates().getComVelocity());

      // update ground plane estimate
      groundPlaneEstimator.compute(groundPlanePositions);
   }

   private void updateSetpoints()
   {
      // update desired horizontal com forces
      trotStateMachine.process();
      dcmPositionController.compute(vrpPositionSetpoint, dcmPositionEstimate, dcmPositionSetpoint, dcmVelocitySetpoint);

      cmpPositionSetpoint.set(vrpPositionSetpoint);
      cmpPositionSetpoint.subZ(lipModel.getComHeight());
      lipModel.computeComForce(taskSpaceControllerCommands.getComForce(), cmpPositionSetpoint);

      taskSpaceControllerCommands.getComForce().changeFrame(supportFrame);

      // update desired com position, velocity, and vertical force
      comPositionControllerSetpoints.getComPosition().changeFrame(supportFrame);
      comPositionControllerSetpoints.getComPosition().set(inputProvider.getComPositionInput());
      comPositionControllerSetpoints.getComVelocity().changeFrame(supportFrame);
      comPositionControllerSetpoints.getComVelocity().set(inputProvider.getComVelocityInput());
      comPositionControllerSetpoints.getComForceFeedforward().changeFrame(supportFrame);
      comPositionControllerSetpoints.getComForceFeedforward().set(taskSpaceControllerCommands.getComForce());
      comPositionControllerSetpoints.getComForceFeedforward().setZ(comPositionGravityCompensationParameter.getValue() * mass * gravity);
      comPositionController.compute(taskSpaceControllerCommands.getComForce(), comPositionControllerSetpoints, controllerToolbox.getTaskSpaceEstimates());

      // update desired body orientation, angular velocity, and torque
      if (trotStateMachine.getCurrentStateEnum() != TrotState.QUAD_SUPPORT)
      {
         bodyYawSetpoint += planarVelocityProvider.get().getZ() * controlDT;
      }
      desiredBodyOrientation.setToZero(worldFrame);
      desiredBodyOrientation.setYawPitchRoll(bodyYawSetpoint, 0.0, 0.0);
      bodyOrientationManager.compute(taskSpaceControllerCommands.getComTorque(), desiredBodyOrientation);

      // update desired contact state and sole forces
      timedStepController.compute(taskSpaceControllerSettings.getContactState(), taskSpaceControllerSettings.getContactForceLimits(),
            taskSpaceControllerCommands.getSoleForce(), controllerToolbox.getTaskSpaceEstimates());

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

      QuadrupedTaskSpaceEstimates taskSpaceEstimates = controllerToolbox.getTaskSpaceEstimates();
      // initialize feedback controllers
      dcmPositionSetpoint.set(dcmPositionEstimate);
      dcmVelocitySetpoint.setToZero();

      dcmPositionController.reset();

      comPositionControllerSetpoints.initialize(taskSpaceEstimates.getComPosition());
      comPositionController.reset();
      comPositionController.getGains().set(comPositionGainsParameter);
      bodyOrientationManager.initialize(taskSpaceEstimates.getBodyOrientation());
      timedStepController.reset();

      // initialize task space controller
      taskSpaceControllerSettings.initialize();
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointDamping(jointDampingParameter.getValue());
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

   private void computeNominalCmpPositions(RobotQuadrant hindSupportQuadrant, RobotQuadrant frontSupportQuadrant, FramePoint3D nominalCmpPositionAtSoS,
         FramePoint3D nominalCmpPositionAtEoS)
   {
      QuadrupedTaskSpaceEstimates taskSpaceEstimates = controllerToolbox.getTaskSpaceEstimates();
      taskSpaceEstimates.getSolePosition(hindSupportQuadrant).changeFrame(worldFrame);
      taskSpaceEstimates.getSolePosition(frontSupportQuadrant).changeFrame(worldFrame);
      nominalCmpPositionAtSoS.setToZero(worldFrame);
      nominalCmpPositionAtSoS.add(taskSpaceEstimates.getSolePosition(hindSupportQuadrant));
      nominalCmpPositionAtSoS.add(taskSpaceEstimates.getSolePosition(frontSupportQuadrant));
      nominalCmpPositionAtSoS.scale(0.5);

      double bodyYaw = bodyYawSetpoint + planarVelocityProvider.get().getZ() * doubleSupportDurationParameter.getValue();
      double xStride = planarVelocityProvider.get().getX() * doubleSupportDurationParameter.getValue();
      double yStride = planarVelocityProvider.get().getY() * doubleSupportDurationParameter.getValue();
      double xOffset = Math.cos(bodyYaw) * xStride - Math.sin(bodyYaw) * yStride;
      double yOffset = Math.sin(bodyYaw) * xStride + Math.cos(bodyYaw) * yStride;
      nominalCmpPositionAtEoS.setIncludingFrame(nominalCmpPositionAtSoS);
      nominalCmpPositionAtEoS.changeFrame(worldFrame);
      nominalCmpPositionAtEoS.add(xOffset, yOffset, 0.0);
   }

   private void computeNominalDcmPositions(FramePoint3D nominalCmpPositionAtSoS, FramePoint3D nominalCmpPositionAtEoS, FramePoint3D nominalDcmPositionAtSoS,
         FramePoint3D nominalDcmPositionAtEoS)
   {
      double timeAtEoS = doubleSupportDurationParameter.getValue();
      double relativeYawAtEoS = planarVelocityProvider.get().getZ() * timeAtEoS;
      nominalPeriodicDcmTrajectory.setComHeight(inputProvider.getComPositionInput().getZ());
      nominalPeriodicDcmTrajectory.initializeTrajectory(0.0, nominalCmpPositionAtSoS, timeAtEoS, nominalCmpPositionAtEoS, relativeYawAtEoS);
      nominalPeriodicDcmTrajectory.computeTrajectory(0.0);
      nominalPeriodicDcmTrajectory.getPosition(nominalDcmPositionAtSoS);
      nominalPeriodicDcmTrajectory.computeTrajectory(timeAtEoS);
      nominalPeriodicDcmTrajectory.getPosition(nominalDcmPositionAtEoS);
   }

   private void computeFootholdPosition(RobotQuadrant robotQuadrant, FramePoint3D cmpPosition, double bodyYaw, FramePoint3D footholdPosition)
   {
      // compute foothold position based on the nominal stance and desired cmp
      QuadrupedTaskSpaceEstimates taskSpaceEstimates = controllerToolbox.getTaskSpaceEstimates();
      taskSpaceEstimates.getSolePosition(robotQuadrant).changeFrame(worldFrame);
      double xStance = robotQuadrant.getEnd().negateIfHindEnd(stanceLengthNominalParameter.getValue() / 2);
      double yStance = robotQuadrant.getSide().negateIfRightSide(stanceWidthNominalParameter.getValue() / 2);
      double xOffset = Math.cos(bodyYaw) * xStance - Math.sin(bodyYaw) * yStance;
      double yOffset = Math.sin(bodyYaw) * xStance + Math.cos(bodyYaw) * yStance;
      footholdPosition.setIncludingFrame(cmpPosition);
      footholdPosition.changeFrame(worldFrame);
      footholdPosition.add(xOffset, yOffset, 0.0);
      groundPlaneEstimator.projectZ(footholdPosition);
   }

   private class QuadSupportState implements FiniteStateMachineState<TrotEvent>
   {
      private final FrameTrajectory3D dcmTrajectory;
      private final FramePoint3D cmpPositionAtSoSNominal;
      private final FramePoint3D cmpPositionAtEoSNominal;
      private final FramePoint3D dcmPositionAtSoSNominal;
      private final FramePoint3D dcmPositionAtEoSNominal;
      private final TimeInterval timeInterval;

      public QuadSupportState()
      {
         dcmTrajectory = new FrameTrajectory3D(6, worldFrame);
         cmpPositionAtSoSNominal = new FramePoint3D();
         cmpPositionAtEoSNominal = new FramePoint3D();
         dcmPositionAtSoSNominal = new FramePoint3D();
         dcmPositionAtEoSNominal = new FramePoint3D();
         timeInterval = new TimeInterval();
      }

      @Override
      public void onEntry()
      {
         timeInterval.setInterval(robotTimestamp.getDoubleValue(), robotTimestamp.getDoubleValue() + quadSupportDurationParameter.getValue());

         // initialize lip com height
         lipModel.setComHeight(inputProvider.getComPositionInput().getZ());

         // compute desired dcm position at start of step
         computeNominalCmpPositions(RobotQuadrant.HIND_LEFT, RobotQuadrant.FRONT_RIGHT, cmpPositionAtSoSNominal, cmpPositionAtEoSNominal);
         computeNominalDcmPositions(cmpPositionAtSoSNominal, cmpPositionAtEoSNominal, dcmPositionAtSoSNominal, dcmPositionAtEoSNominal);

         // compute desired dcm trajectory
         dcmTrajectory.setQuinticWithZeroTerminalVelocityAndAcceleration(timeInterval.getStartTime(), timeInterval.getEndTime(), dcmPositionEstimate, dcmPositionAtSoSNominal);

         // initialize ground plane points
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            groundPlanePositions.get(robotQuadrant).setIncludingFrame(controllerToolbox.getTaskSpaceEstimates().getSolePosition(robotQuadrant));
            groundPlanePositions.get(robotQuadrant).changeFrame(ReferenceFrame.getWorldFrame());
         }
      }

      @Override
      public TrotEvent process()
      {
         double currentTime = robotTimestamp.getDoubleValue();

         // compute dcm setpoint
         dcmTrajectory.compute(currentTime);
         dcmTrajectory.getFramePosition(dcmPositionSetpoint);
         dcmTrajectory.getFrameVelocity(dcmVelocitySetpoint);

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
      private final FramePoint3D cmpPositionAtSoSNominal;
      private final FramePoint3D cmpPositionAtEoSNominal;
      private final FramePoint3D dcmPositionAtSoSNominal;
      private final FramePoint3D dcmPositionAtEoSNominal;
      private final FramePoint3D cmpPositionAtEoS;
      private final FramePoint3D dcmPositionAtEoS;
      private final FramePoint3D footholdPosition;
      private final QuadrupedTimedStep timedStep;
      private final Point3D timedStepGoalPosition;
      private final QuadrantDependentList<Point3D> timedStepGoalPositionAtSoS;

      public DoubleSupportState(RobotQuadrant hindSupportQuadrant, RobotQuadrant frontSupportQuadrant)
      {
         supportQuadrants = new RobotQuadrant[] {hindSupportQuadrant, frontSupportQuadrant};
         swingQuadrants = new RobotQuadrant[] {hindSupportQuadrant.getAcrossBodyQuadrant(), frontSupportQuadrant.getAcrossBodyQuadrant()};
         dcmTrajectory = new PiecewiseForwardDcmTrajectory(1, gravity, lipModel.getComHeight());
         cmpPositionAtSoSNominal = new FramePoint3D();
         cmpPositionAtEoSNominal = new FramePoint3D();
         dcmPositionAtSoSNominal = new FramePoint3D();
         dcmPositionAtEoSNominal = new FramePoint3D();
         cmpPositionAtEoS = new FramePoint3D();
         dcmPositionAtEoS = new FramePoint3D();
         footholdPosition = new FramePoint3D();
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
         dcmTrajectory.computeTrajectory(initialTime + doubleSupportDurationParameter.getValue());
         dcmTrajectory.getPosition(dcmPositionAtEoS);

         // compute desired cmp position at end of step
         cmpPositionAtEoS.set(dcmPositionAtEoS);
         cmpPositionAtEoS.sub(dcmPositionAtEoSNominal);
         cmpPositionAtEoS.add(cmpPositionAtEoSNominal);
         cmpPositionAtEoS.setZ(cmpPositionAtEoSNominal.getZ());

         // compute desired body yaw at end of step
         double bodyYawAtSoS = bodyYawSetpoint;
         double bodyYawAtEoS = bodyYawAtSoS + planarVelocityProvider.get().getZ() * doubleSupportDurationParameter.getValue();

         for (int i = 0; i < 2; i++)
         {
            // compute foothold position to track the periodic dcm trajectory using deadbeat control
            computeFootholdPosition(swingQuadrants[i], cmpPositionAtEoS, bodyYawAtEoS, footholdPosition);

            // trigger step
            timedStep.setRobotQuadrant(swingQuadrants[i]);
            timedStep.setGroundClearance(stepGroundClearanceParameter.getValue());
            timedStep.getTimeInterval().setStartTime(initialTime);
            timedStep.getTimeInterval().setEndTime(initialTime + doubleSupportDurationParameter.getValue());
            timedStep.setGoalPosition(footholdPosition);
            timedStepController.addStep(timedStep);
            timedStep.getGoalPosition(timedStepGoalPositionAtSoS.get(swingQuadrants[i]));

            // initialize ground plane points
            groundPlanePositions.get(swingQuadrants[i]).setIncludingFrame(controllerToolbox.getTaskSpaceEstimates().getSolePosition(swingQuadrants[i]));
            groundPlanePositions.get(swingQuadrants[i]).changeFrame(ReferenceFrame.getWorldFrame());
         }
      }

      @Override
      public TrotEvent process()
      {
         double currentTime = robotTimestamp.getDoubleValue();

         // compute dcm setpoint
         dcmTrajectory.computeTrajectory(currentTime);
         dcmTrajectory.getPosition(dcmPositionSetpoint);
         dcmTrajectory.getVelocity(dcmVelocitySetpoint);

         // adjust swing foot goal position based on dcm tracking error
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
