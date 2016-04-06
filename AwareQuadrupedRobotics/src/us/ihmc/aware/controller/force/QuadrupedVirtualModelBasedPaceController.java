package us.ihmc.aware.controller.force;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.aware.params.DoubleArrayParameter;
import us.ihmc.aware.params.DoubleParameter;
import us.ihmc.aware.params.ParameterFactory;
import us.ihmc.aware.controller.common.DivergentComponentOfMotionController;
import us.ihmc.aware.controller.common.GroundPlaneEstimator;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceCommands;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceController;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceControllerSettings;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceEstimates;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceEstimator;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceSetpoints;
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
import us.ihmc.robotics.geometry.FrameVector;
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
   private final DoubleArrayParameter swingPositionProportionalGainsParameter = parameterFactory.createDoubleArray("swingPositionProportionalGains", 50000, 50000, 100000);
   private final DoubleArrayParameter swingPositionDerivativeGainsParameter = parameterFactory.createDoubleArray("swingPositionDerivativeGains", 500, 500, 500);
   private final DoubleArrayParameter swingPositionIntegralGainsParameter = parameterFactory.createDoubleArray("swingPositionIntegralGains", 0, 0, 0);
   private final DoubleParameter swingPositionMaxIntegralErrorParameter = parameterFactory.createDouble("swingPositionMaxIntegralError", 0);
   private final DoubleParameter swingTrajectoryGroundClearanceParameter = parameterFactory.createDouble("swingTrajectoryGroundClearance", 0.10);
   private final DoubleParameter quadSupportDurationParameter = parameterFactory.createDouble("quadSupportDuration", 1.00);
   private final DoubleParameter doubleSupportDurationParameter = parameterFactory.createDouble("doubleSupportDuration", 0.33);
   private final DoubleParameter stanceWidthNominalParameter = parameterFactory.createDouble("stanceWidthNominal", 0.25);
   private final DoubleParameter stanceLengthNominalParameter = parameterFactory.createDouble("stanceLengthNominal", 1.1);
   private final DoubleParameter noContactPressureLimitParameter = parameterFactory.createDouble("noContactPressureLimit", 75);

   // frames
   private final ReferenceFrame supportFrame;
   private final ReferenceFrame worldFrame;

   // dcm controller
   private final FramePoint dcmPositionEstimate;
   private final FramePoint dcmPositionSetpoint;
   private final FrameVector dcmVelocitySetpoint;
   private final DivergentComponentOfMotionController dcmPositionController;

   // task space controller
   private final QuadrupedTaskSpaceCommands taskSpaceCommands;
   private final QuadrupedTaskSpaceSetpoints taskSpaceSetpoints;
   private final QuadrupedTaskSpaceEstimates taskSpaceEstimates;
   private final QuadrupedTaskSpaceEstimator taskSpaceEstimator;
   private final QuadrupedTaskSpaceController taskSpaceController;
   private final QuadrupedTaskSpaceControllerSettings taskSpaceControllerSettings;

   // planning
   private double bodyYawSetpoint;
   private final GroundPlaneEstimator groundPlaneEstimator;
   private final PiecewisePeriodicDcmTrajectory nominalPeriodicDcmTrajectory;
   private final QuadrantDependentList<ThreeDoFSwingFootTrajectory> swingFootTrajectory;
   private final double[] timeAtSoS;

   // state machine
   public enum TrotState
   {
      QUAD_SUPPORT, HIND_LEFT_FRONT_LEFT_SUPPORT, HIND_RIGHT_FRONT_RIGHT_SUPPORT
   }
   public enum TrotEvent
   {
      TIMEOUT
   }
   private final StateMachine<TrotState, TrotEvent> trotStateMachine;

   public QuadrupedVirtualModelBasedPaceController(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedControllerInputProviderInterface inputProvider,
         QuadrupedForceControllerContext controllerContext)
   {
      this.fullRobotModel = runtimeEnvironment.getFullRobotModel();
      this.robotTimestamp = runtimeEnvironment.getRobotTimestamp();
      this.controlDT = runtimeEnvironment.getControlDT();
      this.gravity = 9.81;
      this.mass = fullRobotModel.getTotalMass();
      this.inputProvider = inputProvider;

      // frames
      QuadrupedReferenceFrames referenceFrames = controllerContext.getReferenceFrames();
      supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      worldFrame = ReferenceFrame.getWorldFrame();

      // dcm controller
      dcmPositionEstimate = new FramePoint();
      dcmPositionSetpoint = new FramePoint();
      dcmVelocitySetpoint = new FrameVector();
      dcmPositionController = controllerContext.getDcmPositionController();

      // task space controllers
      taskSpaceCommands = new QuadrupedTaskSpaceCommands();
      taskSpaceSetpoints = new QuadrupedTaskSpaceSetpoints();
      taskSpaceEstimates = new QuadrupedTaskSpaceEstimates();
      taskSpaceControllerSettings = new QuadrupedTaskSpaceControllerSettings();
      taskSpaceEstimator = controllerContext.getTaskSpaceEstimator();
      taskSpaceController = controllerContext.getTaskSpaceController();

      // planning
      groundPlaneEstimator = new GroundPlaneEstimator();
      swingFootTrajectory = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         swingFootTrajectory.set(robotQuadrant, new ThreeDoFSwingFootTrajectory());
      }
      nominalPeriodicDcmTrajectory = new PiecewisePeriodicDcmTrajectory(2, gravity, inputProvider.getComPositionInput().getZ(), null);
      timeAtSoS = new double[2];

      // state machine
      StateMachineBuilder<TrotState, TrotEvent> stateMachineBuilder = new StateMachineBuilder<>(TrotState.class, "TrotState", registry);
      stateMachineBuilder.addState(TrotState.QUAD_SUPPORT, new QuadSupportState());
      stateMachineBuilder.addState(TrotState.HIND_LEFT_FRONT_LEFT_SUPPORT, new DoubleSupportState(RobotQuadrant.HIND_LEFT, RobotQuadrant.FRONT_LEFT));
      stateMachineBuilder.addState(TrotState.HIND_RIGHT_FRONT_RIGHT_SUPPORT, new DoubleSupportState(RobotQuadrant.HIND_RIGHT, RobotQuadrant.FRONT_RIGHT));
      stateMachineBuilder.addTransition(TrotEvent.TIMEOUT, TrotState.QUAD_SUPPORT, TrotState.HIND_LEFT_FRONT_LEFT_SUPPORT);
      stateMachineBuilder.addTransition(TrotEvent.TIMEOUT, TrotState.HIND_LEFT_FRONT_LEFT_SUPPORT, TrotState.HIND_RIGHT_FRONT_RIGHT_SUPPORT);
      stateMachineBuilder.addTransition(TrotEvent.TIMEOUT, TrotState.HIND_RIGHT_FRONT_RIGHT_SUPPORT, TrotState.HIND_LEFT_FRONT_LEFT_SUPPORT);
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
      taskSpaceEstimates.getComPosition().changeFrame(worldFrame);
      taskSpaceEstimates.getComVelocity().changeFrame(worldFrame);
      dcmPositionEstimate.changeFrame(worldFrame);
      dcmPositionEstimate.set(taskSpaceEstimates.getComVelocity());
      dcmPositionEstimate.scale(1.0 / dcmPositionController.getNaturalFrequency());
      dcmPositionEstimate.add(taskSpaceEstimates.getComPosition());
   }

   private void updateSetpoints()
   {
      // update desired dcm and sole setpoints
      trotStateMachine.process();

      // update desired horizontal com forces
      dcmPositionController.compute(taskSpaceSetpoints.getComForceFeedforward(), dcmPositionSetpoint, dcmVelocitySetpoint, dcmPositionEstimate);

      // update desired com position, velocity, and vertical force
      taskSpaceSetpoints.getComPosition().changeFrame(supportFrame);
      taskSpaceSetpoints.getComPosition().set(inputProvider.getComPositionInput());
      taskSpaceSetpoints.getComVelocity().setToZero();
      taskSpaceSetpoints.getComForceFeedforward().changeFrame(worldFrame);
      taskSpaceSetpoints.getComForceFeedforward().setZ(mass * gravity);

      // update desired body orientation, angular velocity, and torque
      if (trotStateMachine.getState() != TrotState.QUAD_SUPPORT)
      {
         bodyYawSetpoint += inputProvider.getPlanarVelocityInput().getZ() * controlDT;
      }
      taskSpaceSetpoints.getBodyOrientation().changeFrame(worldFrame);
      taskSpaceSetpoints.getBodyOrientation().setYawPitchRoll(bodyYawSetpoint,
            RotationTools.computePitch(inputProvider.getBodyOrientationInput()) + groundPlaneEstimator.getPitch(bodyYawSetpoint),
                  RotationTools.computeRoll(inputProvider.getBodyOrientationInput()));
      taskSpaceSetpoints.getBodyAngularVelocity().setToZero();
      taskSpaceSetpoints.getComTorqueFeedforward().setToZero();

      // update joint setpoints
      taskSpaceController.compute(taskSpaceControllerSettings, taskSpaceSetpoints, taskSpaceEstimates, taskSpaceCommands);
      taskSpaceSetpoints.getBodyOrientation().changeFrame(worldFrame);
   }

   @Override public QuadrupedForceControllerEvent process()
   {
      updateEstimates();
      updateSetpoints();
      return null;
   }

   @Override public void onEntry()
   {
      // initialize dcm controller
      dcmPositionController.setComHeight(inputProvider.getComPositionInput().getZ());
      dcmPositionController.setGains(
            dcmPositionProportionalGainsParameter.get(),
            dcmPositionDerivativeGainsParameter.get(),
            dcmPositionIntegralGainsParameter.get(),
            dcmPositionMaxIntegralErrorParameter.get()
      );
      dcmPositionController.reset();

      // initialize task space controller
      taskSpaceEstimator.compute(taskSpaceEstimates);
      taskSpaceSetpoints.initialize(taskSpaceEstimates);
      taskSpaceControllerSettings.initialize();
      taskSpaceControllerSettings.setJointDamping(jointDampingParameter.get());
      taskSpaceControllerSettings.setComForceCommandWeights(1.0, 1.0, 1.0);
      taskSpaceControllerSettings.setComTorqueCommandWeights(1.0, 1.0, 1.0);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         taskSpaceControllerSettings.setSoleForceCommandWeights(robotQuadrant, 0.0, 0.0, 0.0);
         taskSpaceControllerSettings.setContactState(robotQuadrant, ContactState.IN_CONTACT);
         taskSpaceControllerSettings.setSolePositionFeedbackGainsToZero(robotQuadrant);
      }
      taskSpaceControllerSettings.setBodyOrientationFeedbackGains(
            bodyOrientationProportionalGainsParameter.get(),
            bodyOrientationDerivativeGainsParameter.get(),
            bodyOrientationIntegralGainsParameter.get(),
            bodyOrientationMaxIntegralErrorParameter.get()
      );
      taskSpaceControllerSettings.setComPositionFeedbackGains(
            comPositionProportionalGainsParameter.get(),
            comPositionDerivativeGainsParameter.get(),
            comPositionIntegralGainsParameter.get(),
            comPositionMaxIntegralErrorParameter.get()
      );
      taskSpaceController.reset();

      // initialize body yaw trajectory
      taskSpaceEstimates.getBodyOrientation().changeFrame(worldFrame);
      bodyYawSetpoint = taskSpaceEstimates.getBodyOrientation().getYaw();

      // initialize state machine
      trotStateMachine.reset();
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

   private class QuadSupportState implements StateMachineState<TrotEvent>
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

         // initialize contact state
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            taskSpaceControllerSettings.setContactState(robotQuadrant, ContactState.IN_CONTACT);
            taskSpaceControllerSettings.setPressureUpperLimit(robotQuadrant, Double.MAX_VALUE);
         }

         // compute ground plane estimate
         groundPlaneEstimator.compute(taskSpaceEstimates.getSolePosition());
      }

      @Override public TrotEvent process()
      {
         double currentTime = robotTimestamp.getDoubleValue();

         // compute dcm setpoint
         dcmTrajectory.computeTrajectory(currentTime - initialTime);
         dcmTrajectory.getPosition(dcmPositionSetpoint);
         dcmTrajectory.getVelocity(dcmVelocitySetpoint);

         // trigger touch down event
         if (currentTime > initialTime + quadSupportDurationParameter.get())
            return TrotEvent.TIMEOUT;
         else
            return null;
      }

      @Override public void onExit()
      {
      }
   }

   private class DoubleSupportState implements StateMachineState<TrotEvent>
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
         taskSpaceSetpoints.getBodyOrientation().changeFrame(worldFrame);
         double bodyYawAtSoS = taskSpaceSetpoints.getBodyOrientation().getYaw();
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
            taskSpaceControllerSettings.setSolePositionFeedbackGainsToZero(supportQuadrant);
            taskSpaceControllerSettings.setSolePositionFeedbackGains(swingQuadrant,
                  swingPositionProportionalGainsParameter.get(),
                  swingPositionDerivativeGainsParameter.get(),
                  swingPositionIntegralGainsParameter.get(),
                  swingPositionMaxIntegralErrorParameter.get()
            );

            // initialize contact state
            taskSpaceControllerSettings.setContactState(swingQuadrant, ContactState.NO_CONTACT);
            taskSpaceControllerSettings.setContactState(supportQuadrant, ContactState.IN_CONTACT);
            taskSpaceControllerSettings.setPressureUpperLimit(swingQuadrant, noContactPressureLimitParameter.get());
            taskSpaceControllerSettings.setPressureUpperLimit(supportQuadrant, Double.MAX_VALUE);
         }

         // compute ground plane estimate
         groundPlaneEstimator.compute(taskSpaceEstimates.getSolePosition());
      }

      @Override public TrotEvent process()
      {
         double currentTime = robotTimestamp.getDoubleValue();

         // compute dcm setpoint
         dcmTrajectory.computeTrajectory(currentTime);
         dcmTrajectory.getPosition(dcmPositionSetpoint);
         dcmTrajectory.getVelocity(dcmVelocitySetpoint);

         for (int i = 0; i < 2; i++)
         {
            RobotQuadrant swingQuadrant = swingQuadrants[i];

            // compute nominal sole position setpoint
            swingFootTrajectory.get(swingQuadrant).computeTrajectory(currentTime - initialTime);
            swingFootTrajectory.get(swingQuadrant).getPosition(taskSpaceSetpoints.getSolePosition(swingQuadrant));

            // shift the swing foot trajectory in the direction of the dcm tracking error
            dcmPositionEstimate.changeFrame(worldFrame);
            dcmPositionSetpoint.changeFrame(worldFrame);
            taskSpaceSetpoints.getSolePosition(swingQuadrant).changeFrame(worldFrame);
            taskSpaceSetpoints.getSolePosition(swingQuadrant).add(dcmPositionEstimate.getX(), dcmPositionEstimate.getY(), 0.0);
            taskSpaceSetpoints.getSolePosition(swingQuadrant).sub(dcmPositionSetpoint.getX(), dcmPositionSetpoint.getY(), 0.0);
         }

         // trigger touch down event
         if (currentTime > initialTime + doubleSupportDurationParameter.get())
            return TrotEvent.TIMEOUT;
         else
            return null;
      }

      @Override public void onExit()
      {
      }
   }
}
