package us.ihmc.aware.controller.force;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.aware.controller.toolbox.*;
import us.ihmc.aware.params.DoubleArrayParameter;
import us.ihmc.aware.params.DoubleParameter;
import us.ihmc.aware.params.ParameterFactory;
import us.ihmc.aware.planning.GroundPlaneEstimator;
import us.ihmc.aware.parameters.QuadrupedRuntimeEnvironment;
import us.ihmc.aware.planning.PiecewiseForwardDcmTrajectory;
import us.ihmc.aware.planning.PiecewisePeriodicDcmTrajectory;
import us.ihmc.aware.planning.ThreeDoFMinimumJerkTrajectory;
import us.ihmc.aware.planning.ThreeDoFSwingFootTrajectory;
import us.ihmc.aware.state.StateMachine;
import us.ihmc.aware.state.StateMachineBuilder;
import us.ihmc.aware.state.StateMachineState;
import us.ihmc.aware.util.ContactState;
import us.ihmc.quadrupedRobotics.dataProviders.QuadrupedControllerInputProviderInterface;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedVirtualModelBasedPaceController implements QuadrupedForceController
{
   private final SDFFullRobotModel fullRobotModel;
   private final DoubleYoVariable robotTimestamp;
   private final double controlDT;
   private final double gravity;
   private final double mass;
   private final QuadrupedControllerInputProviderInterface inputProvider;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   // parameters
   private final ParameterFactory parameterFactory = new ParameterFactory(getClass().getSimpleName());
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
   private final DoubleArrayParameter solePositionProportionalGainsParameter = parameterFactory.createDoubleArray("solePositionProportionalGains", 50000, 50000, 100000);
   private final DoubleArrayParameter solePositionDerivativeGainsParameter = parameterFactory.createDoubleArray("solePositionDerivativeGains", 500, 500, 500);
   private final DoubleArrayParameter solePositionIntegralGainsParameter = parameterFactory.createDoubleArray("solePositionIntegralGains", 0, 0, 0);
   private final DoubleParameter solePositionMaxIntegralErrorParameter = parameterFactory.createDouble("solePositionMaxIntegralError", 0);
   private final DoubleParameter swingTrajectoryGroundClearanceParameter = parameterFactory.createDouble("swingTrajectoryGroundClearance", 0.10);
   private final DoubleParameter quadSupportDurationParameter = parameterFactory.createDouble("quadSupportDuration", 1.00);
   private final DoubleParameter doubleSupportDurationParameter = parameterFactory.createDouble("doubleSupportDuration", 0.33);
   private final DoubleParameter stanceWidthNominalParameter = parameterFactory.createDouble("stanceWidthNominal", 0.25);
   private final DoubleParameter stanceLengthNominalParameter = parameterFactory.createDouble("stanceLengthNominal", 1.1);
   private final DoubleParameter noContactPressureLimitParameter = parameterFactory.createDouble("noContactPressureLimit", 75);

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
   private final QuadrupedSolePositionController.Setpoints solePositionControllerSetpoints;
   private final QuadrupedSolePositionController solePositionController;

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
   private final QuadrantDependentList<ThreeDoFSwingFootTrajectory> swingFootTrajectory;
   private final double[] timeAtSoS;

   // state machine
   public enum PaceState
   {
      QUAD_SUPPORT, HIND_LEFT_FRONT_LEFT_SUPPORT, HIND_RIGHT_FRONT_RIGHT_SUPPORT
   }
   public enum PaceEvent
   {
      TIMEOUT
   }
   private final StateMachine<PaceState, PaceEvent> paceStateMachine;

   public QuadrupedVirtualModelBasedPaceController(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedControllerInputProviderInterface inputProvider,
         QuadrupedForceControllerToolbox controllerToolbox)
   {
      this.fullRobotModel = runtimeEnvironment.getFullRobotModel();
      this.robotTimestamp = runtimeEnvironment.getRobotTimestamp();
      this.controlDT = runtimeEnvironment.getControlDT();
      this.gravity = 9.81;
      this.mass = fullRobotModel.getTotalMass();
      this.inputProvider = inputProvider;

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
      solePositionControllerSetpoints = new QuadrupedSolePositionController.Setpoints();
      solePositionController = controllerToolbox.getSolePositionController();

      // task space controllers
      taskSpaceEstimates = new QuadrupedTaskSpaceEstimator.Estimates();
      taskSpaceEstimator = controllerToolbox.getTaskSpaceEstimator();
      taskSpaceControllerCommands = new QuadrupedTaskSpaceController.Commands();
      taskSpaceControllerSettings = new QuadrupedTaskSpaceController.Settings();
      taskSpaceController = controllerToolbox.getTaskSpaceController();

      // planning
      groundPlaneEstimator = new GroundPlaneEstimator();
      groundPlanePositions = new QuadrantDependentList<>();
      swingFootTrajectory = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         groundPlanePositions.set(robotQuadrant, new FramePoint());
         swingFootTrajectory.set(robotQuadrant, new ThreeDoFSwingFootTrajectory());
      }
      nominalPeriodicDcmTrajectory = new PiecewisePeriodicDcmTrajectory(2, gravity, inputProvider.getComPositionInput().getZ(), null);
      timeAtSoS = new double[2];

      // state machine
      StateMachineBuilder<PaceState, PaceEvent> stateMachineBuilder = new StateMachineBuilder<>(PaceState.class, "PaceState", registry);
      stateMachineBuilder.addState(PaceState.QUAD_SUPPORT, new QuadSupportState());
      stateMachineBuilder.addState(PaceState.HIND_LEFT_FRONT_LEFT_SUPPORT, new DoubleSupportState(RobotQuadrant.HIND_LEFT, RobotQuadrant.FRONT_LEFT));
      stateMachineBuilder.addState(PaceState.HIND_RIGHT_FRONT_RIGHT_SUPPORT, new DoubleSupportState(RobotQuadrant.HIND_RIGHT, RobotQuadrant.FRONT_RIGHT));
      stateMachineBuilder.addTransition(PaceEvent.TIMEOUT, PaceState.QUAD_SUPPORT, PaceState.HIND_LEFT_FRONT_LEFT_SUPPORT);
      stateMachineBuilder.addTransition(PaceEvent.TIMEOUT, PaceState.HIND_LEFT_FRONT_LEFT_SUPPORT, PaceState.HIND_RIGHT_FRONT_RIGHT_SUPPORT);
      stateMachineBuilder.addTransition(PaceEvent.TIMEOUT, PaceState.HIND_RIGHT_FRONT_RIGHT_SUPPORT, PaceState.HIND_LEFT_FRONT_LEFT_SUPPORT);
      paceStateMachine = stateMachineBuilder.build(PaceState.QUAD_SUPPORT);

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
      // update desired dcm and sole setpoints
      paceStateMachine.process();
      solePositionController.compute(taskSpaceControllerCommands.getSoleForce(), solePositionControllerSetpoints, taskSpaceEstimates);

      // update desired horizontal com forces
      dcmPositionController.compute(taskSpaceControllerCommands.getComForce(), dcmPositionControllerSetpoints, dcmPositionEstimate);

      // update desired com position, velocity, and vertical force
      comPositionControllerSetpoints.getComPosition().changeFrame(supportFrame);
      comPositionControllerSetpoints.getComPosition().set(inputProvider.getComPositionInput());
      comPositionControllerSetpoints.getComVelocity().changeFrame(supportFrame);
      comPositionControllerSetpoints.getComVelocity().set(inputProvider.getComVelocityInput());
      comPositionControllerSetpoints.getComForceFeedforward().setIncludingFrame(taskSpaceControllerCommands.getComForce());
      comPositionControllerSetpoints.getComForceFeedforward().changeFrame(supportFrame);
      comPositionControllerSetpoints.getComForceFeedforward().setZ(mass * gravity);
      comPositionController.compute(taskSpaceControllerCommands.getComForce(), comPositionControllerSetpoints, taskSpaceEstimates);

      // update desired body orientation, angular velocity, and torque
      if (paceStateMachine.getState() != PaceState.QUAD_SUPPORT)
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

      // update joint setpoints
      taskSpaceController.compute(taskSpaceControllerSettings, taskSpaceControllerCommands);
   }

   @Override public QuadrupedForceControllerEvent process()
   {
      updateEstimates();
      updateSetpoints();
      return null;
   }

   @Override public void onEntry()
   {
      updateEstimates();

      // initialize feedback controllers
      dcmPositionControllerSetpoints.initialize(dcmPositionEstimate);
      dcmPositionController.reset();
      dcmPositionController.setComHeight(inputProvider.getComPositionInput().getZ());
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
      solePositionControllerSetpoints.initialize(taskSpaceEstimates);
      solePositionController.reset();

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
      paceStateMachine.reset();
   }

   @Override public void onExit()
   {
   }

   private void computeNominalCmpPositions(RobotQuadrant hindQuadrant, RobotQuadrant frontQuadrant, FramePoint[] nominalCmpPositionAtSoS, FramePoint nominalCmpPositionAtEoS)
   {
      double xStride = inputProvider.getPlanarVelocityInput().getX() * doubleSupportDurationParameter.get();
      double yStride = inputProvider.getPlanarVelocityInput().getY() * doubleSupportDurationParameter.get() * 2;
      double bodyYaw, yStance, xOffset, yOffset;

      // cmp position
      bodyYaw = bodyYawSetpoint;
      taskSpaceEstimates.getSolePosition(hindQuadrant).changeFrame(worldFrame);
      taskSpaceEstimates.getSolePosition(frontQuadrant).changeFrame(worldFrame);
      nominalCmpPositionAtSoS[0].setToZero(worldFrame);
      nominalCmpPositionAtSoS[0].add(taskSpaceEstimates.getSolePosition(hindQuadrant));
      nominalCmpPositionAtSoS[0].add(taskSpaceEstimates.getSolePosition(frontQuadrant));
      nominalCmpPositionAtSoS[0].scale(0.5);

      // cmp position after 1 step
      bodyYaw = bodyYaw + inputProvider.getPlanarVelocityInput().getZ() * doubleSupportDurationParameter.get();
      yStance = hindQuadrant.getSide().negateIfLeftSide(stanceWidthNominalParameter.get());
      xOffset =-Math.sin(bodyYaw) * yStance + Math.cos(bodyYaw) * xStride - Math.sin(bodyYaw) * yStride;
      yOffset = Math.cos(bodyYaw) * yStance + Math.sin(bodyYaw) * xStride + Math.cos(bodyYaw) * yStride;
      nominalCmpPositionAtSoS[1].setIncludingFrame(nominalCmpPositionAtSoS[0]);
      nominalCmpPositionAtSoS[1].changeFrame(worldFrame);
      nominalCmpPositionAtSoS[1].add(xOffset, yOffset, 0.0);

      // cmp position after 2 steps
      bodyYaw = bodyYaw + inputProvider.getPlanarVelocityInput().getZ() * doubleSupportDurationParameter.get();
      yStance = hindQuadrant.getSide().negateIfRightSide(stanceWidthNominalParameter.get());
      xOffset =-Math.sin(bodyYaw) * yStance + Math.cos(bodyYaw) * xStride - Math.sin(bodyYaw) * yStride;
      yOffset = Math.cos(bodyYaw) * yStance + Math.sin(bodyYaw) * xStride + Math.cos(bodyYaw) * yStride;
      nominalCmpPositionAtEoS.setIncludingFrame(nominalCmpPositionAtSoS[1]);
      nominalCmpPositionAtEoS.changeFrame(worldFrame);
      nominalCmpPositionAtEoS.add(xOffset, yOffset, 0.0);
   }

   private void computeNominalDcmPositions(FramePoint[] nominalCmpPositionAtSoS, FramePoint nominalCmpPositionAtEoS, FramePoint[] nominalDcmPositionAtSoS, FramePoint nominalDcmPositionAtEoS)
   {
      timeAtSoS[0] = 0;
      timeAtSoS[1] = doubleSupportDurationParameter.get();
      double timeAtEoS = doubleSupportDurationParameter.get() * 2;
      double relativeYawAtEoS = 2 * inputProvider.getPlanarVelocityInput().getZ() * doubleSupportDurationParameter.get();
      nominalPeriodicDcmTrajectory.setComHeight(inputProvider.getComPositionInput().getZ());
      nominalPeriodicDcmTrajectory.initializeTrajectory(2, timeAtSoS, nominalCmpPositionAtSoS, timeAtEoS, nominalCmpPositionAtEoS, relativeYawAtEoS);
      for (int i = 0; i < 2; i++)
      {
         nominalPeriodicDcmTrajectory.computeTrajectory(timeAtSoS[i]);
         nominalPeriodicDcmTrajectory.getPosition(nominalDcmPositionAtSoS[i]);
      }
      nominalPeriodicDcmTrajectory.computeTrajectory(timeAtEoS);
      nominalPeriodicDcmTrajectory.getPosition(nominalDcmPositionAtEoS);
   }

   private void computeFootholdPosition(RobotQuadrant robotQuadrant, FramePoint cmpPosition, double bodyYaw, FramePoint footholdPosition)
   {
      // compute foothold position based on the nominal stance and desired cmp
      taskSpaceEstimates.getSolePosition(robotQuadrant).changeFrame(worldFrame);
      double xStance = robotQuadrant.getEnd().negateIfHindEnd(stanceLengthNominalParameter.get() / 2);
      double yStance = 0.0;
      double xOffset = Math.cos(bodyYaw) * xStance - Math.sin(bodyYaw) * yStance;
      double yOffset = Math.sin(bodyYaw) * xStance + Math.cos(bodyYaw) * yStance;
      footholdPosition.setIncludingFrame(cmpPosition);
      footholdPosition.changeFrame(worldFrame);
      footholdPosition.add(xOffset, yOffset, 0.0);
      groundPlaneEstimator.projectZ(footholdPosition);
   }

   private class QuadSupportState implements StateMachineState<PaceEvent>
   {
      private double initialTime;
      private final ThreeDoFMinimumJerkTrajectory dcmTrajectory;
      private final FramePoint[] cmpPositionAtSoSNominal;
      private final FramePoint cmpPositionAtEoSNominal;
      private final FramePoint[] dcmPositionAtSoSNominal;
      private final FramePoint dcmPositionAtEoSNominal;

      public QuadSupportState()
      {
         initialTime = 0.0;
         dcmTrajectory = new ThreeDoFMinimumJerkTrajectory();
         cmpPositionAtSoSNominal = new FramePoint[] {new FramePoint(), new FramePoint()};
         cmpPositionAtEoSNominal = new FramePoint();
         dcmPositionAtSoSNominal = new FramePoint[] {new FramePoint(), new FramePoint()};
         dcmPositionAtEoSNominal = new FramePoint();
      }

      @Override public void onEntry()
      {
         initialTime = robotTimestamp.getDoubleValue();

         // initialize dcm controller height
         dcmPositionController.setComHeight(inputProvider.getComPositionInput().getZ());

         // compute desired dcm position at start of step
         computeNominalCmpPositions(RobotQuadrant.HIND_LEFT, RobotQuadrant.FRONT_LEFT, cmpPositionAtSoSNominal, cmpPositionAtEoSNominal);
         computeNominalDcmPositions(cmpPositionAtSoSNominal, cmpPositionAtEoSNominal, dcmPositionAtSoSNominal, dcmPositionAtEoSNominal);

         // compute desired dcm trajectory
         dcmTrajectory.initializeTrajectory(dcmPositionEstimate, dcmPositionAtSoSNominal[0], quadSupportDurationParameter.get());

         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            // initialize contact state
            taskSpaceControllerSettings.setContactState(robotQuadrant, ContactState.IN_CONTACT);
            taskSpaceControllerSettings.getContactForceLimits().setPressureUpperLimit(robotQuadrant, Double.MAX_VALUE);

            // initialize ground plane points
            groundPlanePositions.get(robotQuadrant).setIncludingFrame(taskSpaceEstimates.getSolePosition(robotQuadrant));
            groundPlanePositions.get(robotQuadrant).changeFrame(ReferenceFrame.getWorldFrame());
         }
      }

      @Override public PaceEvent process()
      {
         double currentTime = robotTimestamp.getDoubleValue();

         // compute dcm setpoint
         dcmTrajectory.computeTrajectory(currentTime - initialTime);
         dcmTrajectory.getPosition(dcmPositionControllerSetpoints.getDcmPosition());
         dcmTrajectory.getVelocity(dcmPositionControllerSetpoints.getDcmVelocity());

         // trigger touch down event
         if (currentTime > initialTime + quadSupportDurationParameter.get())
            return PaceEvent.TIMEOUT;
         else
            return null;
      }

      @Override public void onExit()
      {
      }
   }

   private class DoubleSupportState implements StateMachineState<PaceEvent>
   {
      private double initialTime;
      private final RobotQuadrant supportQuadrants[];
      private final RobotQuadrant swingQuadrants[];
      private final PiecewiseForwardDcmTrajectory dcmTrajectory;
      private final FramePoint[] cmpPositionAtSoSNominal;
      private final FramePoint cmpPositionAtEoSNominal;
      private final FramePoint[] dcmPositionAtSoSNominal;
      private final FramePoint dcmPositionAtEoSNominal;
      private final FramePoint cmpPositionAtEoS;
      private final FramePoint dcmPositionAtSoS;
      private final FramePoint footholdPosition;

      public DoubleSupportState(RobotQuadrant hindSupportQuadrant, RobotQuadrant frontSupportQuadrant)
      {
         initialTime = 0.0;
         supportQuadrants = new RobotQuadrant[] {hindSupportQuadrant, frontSupportQuadrant};
         swingQuadrants = new RobotQuadrant[] {hindSupportQuadrant.getAcrossBodyQuadrant(), frontSupportQuadrant.getAcrossBodyQuadrant()};
         dcmTrajectory = new PiecewiseForwardDcmTrajectory(1, gravity, dcmPositionController.getComHeight(), null);
         cmpPositionAtSoSNominal = new FramePoint[] {new FramePoint(), new FramePoint()};
         cmpPositionAtEoSNominal = new FramePoint();
         dcmPositionAtSoSNominal = new FramePoint[] {new FramePoint(), new FramePoint()};
         dcmPositionAtEoSNominal = new FramePoint();
         cmpPositionAtEoS = new FramePoint();
         dcmPositionAtSoS = new FramePoint();
         footholdPosition = new FramePoint();
      }

      @Override public void onEntry()
      {
         initialTime = robotTimestamp.getDoubleValue();

         // initialize dcm controller height
         dcmPositionController.setComHeight(inputProvider.getComPositionInput().getZ());

         // compute desired dcm position at end of step
         computeNominalCmpPositions(supportQuadrants[0], supportQuadrants[1], cmpPositionAtSoSNominal, cmpPositionAtEoSNominal);
         computeNominalDcmPositions(cmpPositionAtSoSNominal, cmpPositionAtEoSNominal, dcmPositionAtSoSNominal, dcmPositionAtEoSNominal);

         // compute desired dcm trajectory
         dcmPositionEstimate.changeFrame(worldFrame);
         dcmTrajectory.setComHeight(dcmPositionController.getComHeight());
         dcmTrajectory.initializeTrajectory(initialTime, cmpPositionAtSoSNominal[0], dcmPositionEstimate);
         dcmTrajectory.computeTrajectory(initialTime + doubleSupportDurationParameter.get());
         dcmTrajectory.getPosition(dcmPositionAtSoS);

         // compute desired cmp position at end of step
         cmpPositionAtEoS.set(dcmPositionAtSoS);
         cmpPositionAtEoS.sub(dcmPositionAtSoSNominal[1]);
         cmpPositionAtEoS.add(cmpPositionAtSoSNominal[1]);

         // compute desired body yaw at end of step
         double bodyYawAtSoS = bodyYawSetpoint;
         double bodyYawAtEoS = bodyYawAtSoS + inputProvider.getPlanarVelocityInput().getZ() * doubleSupportDurationParameter.get();

         for (int i = 0; i < 2; i++)
         {
            RobotQuadrant swingQuadrant = swingQuadrants[i];
            RobotQuadrant supportQuadrant = supportQuadrants[i];

            // compute foothold position to track the periodic dcm trajectory using deadbeat control
            computeFootholdPosition(swingQuadrant, cmpPositionAtEoS, bodyYawAtEoS, footholdPosition);

            // initialize swing foot trajectory
            FramePoint solePosition = taskSpaceEstimates.getSolePosition(swingQuadrant);
            solePosition.changeFrame(footholdPosition.getReferenceFrame());
            swingFootTrajectory.get(swingQuadrant).initializeTrajectory(solePosition, footholdPosition,
               swingTrajectoryGroundClearanceParameter.get(), doubleSupportDurationParameter.get());

            // initialize sole position feedback gains
            solePositionController.getGains(swingQuadrant).setProportionalGains(solePositionProportionalGainsParameter.get());
            solePositionController.getGains(swingQuadrant).setIntegralGains(solePositionIntegralGainsParameter.get(), solePositionMaxIntegralErrorParameter.get());
            solePositionController.getGains(swingQuadrant).setDerivativeGains(solePositionDerivativeGainsParameter.get());
            solePositionController.getGains(supportQuadrant).reset();

            // initialize contact state
            taskSpaceControllerSettings.setContactState(swingQuadrant, ContactState.NO_CONTACT);
            taskSpaceControllerSettings.setContactState(supportQuadrant, ContactState.IN_CONTACT);
            taskSpaceControllerSettings.getContactForceLimits().setPressureUpperLimit(swingQuadrant, noContactPressureLimitParameter.get());
            taskSpaceControllerSettings.getContactForceLimits().setPressureUpperLimit(supportQuadrant, Double.MAX_VALUE);

            // initialize ground plane points
            groundPlanePositions.get(swingQuadrants[i]).setIncludingFrame(taskSpaceEstimates.getSolePosition(swingQuadrants[i]));
            groundPlanePositions.get(swingQuadrants[i]).changeFrame(ReferenceFrame.getWorldFrame());
         }
      }

      @Override public PaceEvent process()
      {
         double currentTime = robotTimestamp.getDoubleValue();

         // compute dcm setpoint
         dcmTrajectory.computeTrajectory(currentTime);
         dcmTrajectory.getPosition(dcmPositionControllerSetpoints.getDcmPosition());
         dcmTrajectory.getVelocity(dcmPositionControllerSetpoints.getDcmVelocity());

         for (int i = 0; i < 2; i++)
         {
            RobotQuadrant swingQuadrant = swingQuadrants[i];

            // compute nominal sole position setpoint
            swingFootTrajectory.get(swingQuadrant).computeTrajectory(currentTime - initialTime);
            swingFootTrajectory.get(swingQuadrant).getPosition(solePositionControllerSetpoints.getSolePosition(swingQuadrant));

            // shift the swing foot trajectory in the direction of the dcm tracking error
            FramePoint dcmPositionSetpoint = dcmPositionControllerSetpoints.getDcmPosition();
            dcmPositionEstimate.changeFrame(worldFrame);
            dcmPositionSetpoint.changeFrame(worldFrame);
            solePositionControllerSetpoints.getSolePosition(swingQuadrant).changeFrame(worldFrame);
            solePositionControllerSetpoints.getSolePosition(swingQuadrant).add(dcmPositionEstimate.getX(), dcmPositionEstimate.getY(), 0.0);
            solePositionControllerSetpoints.getSolePosition(swingQuadrant).sub(dcmPositionSetpoint.getX(), dcmPositionSetpoint.getY(), 0.0);
         }

         // trigger touch down event
         if (currentTime > initialTime + doubleSupportDurationParameter.get())
            return PaceEvent.TIMEOUT;
         else
            return null;
      }

      @Override public void onExit()
      {
      }
   }
}
