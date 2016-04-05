package us.ihmc.aware.controller.force;

import java.io.BufferedReader;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.io.StringReader;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.aware.config.DoubleArrayParameter;
import us.ihmc.aware.config.DoubleParameter;
import us.ihmc.aware.config.ParameterFactory;
import us.ihmc.aware.config.ParameterPacketListener;
import us.ihmc.aware.config.ParameterRegistry;
import us.ihmc.aware.controller.common.DivergentComponentOfMotionController;
import us.ihmc.aware.controller.force.taskSpaceController.*;
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
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointNameMap;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class QuadrupedVirtualModelBasedTrotController implements QuadrupedForceController
{
   private final SDFFullRobotModel fullRobotModel;
   private final DoubleYoVariable robotTimestamp;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;
   private final QuadrupedJointNameMap jointNameMap;
   private final double controlDT;
   private final double gravity;
   private final double mass;
   private final QuadrupedControllerInputProviderInterface inputProvider;

   // parameters
   private final ParameterFactory propertyFactory = new ParameterFactory(QuadrupedVirtualModelBasedTrotController.class.getSimpleName());
   private final DoubleArrayParameter bodyOrientationProportionalGainsProperty = propertyFactory.createDoubleArray("bodyOrientationProportionalGains", 5000, 5000, 5000);
   private final DoubleArrayParameter bodyOrientationDerivativeGainsProperty = propertyFactory.createDoubleArray("bodyOrientationDerivativeGains", 750, 750, 750);
   private final DoubleArrayParameter bodyOrientationIntegralGainsProperty = propertyFactory.createDoubleArray("bodyOrientationIntegralGains", 0, 0, 0);
   private final DoubleParameter bodyOrientationMaxIntegralErrorProperty = propertyFactory.createDouble("bodyOrientationMaxIntegralError", 0);
   private final DoubleArrayParameter comPositionProportionalGainsProperty = propertyFactory.createDoubleArray("comPositionProportionalGains", 0, 0, 5000);
   private final DoubleArrayParameter comPositionDerivativeGainsProperty = propertyFactory.createDoubleArray("comPositionDerivativeGains", 0, 0, 750);
   private final DoubleArrayParameter comPositionIntegralGainsProperty = propertyFactory.createDoubleArray("comPositionIntegralGains", 0, 0, 0);
   private final DoubleParameter comPositionMaxIntegralErrorProperty = propertyFactory.createDouble("comPositionMaxIntegralError", 0);
   private final DoubleArrayParameter dcmPositionProportionalGainsProperty = propertyFactory.createDoubleArray("dcmPositionProportionalGains", 1, 1, 0);
   private final DoubleArrayParameter dcmPositionDerivativeGainsProperty = propertyFactory.createDoubleArray("dcmPositionDerivativeGains", 0, 0, 0);
   private final DoubleArrayParameter dcmPositionIntegralGainsProperty = propertyFactory.createDoubleArray("dcmPositionIntegralGains", 0, 0, 0);
   private final DoubleParameter dcmPositionMaxIntegralErrorProperty = propertyFactory.createDouble("dcmPositionMaxIntegralError", 0);
   private final DoubleArrayParameter swingPositionProportionalGainsProperty = propertyFactory.createDoubleArray("swingPositionProportionalGains", 50000, 50000, 100000);
   private final DoubleArrayParameter swingPositionDerivativeGainsProperty = propertyFactory.createDoubleArray("swingPositionDerivativeGains", 500, 500, 500);
   private final DoubleArrayParameter swingPositionIntegralGainsProperty = propertyFactory.createDoubleArray("swingPositionIntegralGains", 0, 0, 0);
   private final DoubleParameter swingPositionMaxIntegralErrorProperty = propertyFactory.createDouble("swingPositionMaxIntegralError", 0);
   private final DoubleParameter swingTrajectoryGroundClearanceProperty = propertyFactory.createDouble("swingTrajectoryGroundClearance", 0.1);
   private final DoubleParameter quadSupportDurationProperty = propertyFactory.createDouble("quadSupportDuration", 1.0);
   private final DoubleParameter doubleSupportDurationProperty = propertyFactory.createDouble("doubleSupportDuration", 0.33);
   private final DoubleParameter stanceWidthNominalProperty = propertyFactory.createDouble("stanceWidthNominal", 0.35);
   private final DoubleParameter stanceLengthNominalProperty = propertyFactory.createDouble("stanceLengthNominal", 1.1);
   private final DoubleParameter noContactPressureLimitProperty = propertyFactory.createDouble("noContactPressureLimit", 75);

   // frames
   private final PoseReferenceFrame supportFrame;
   private final ReferenceFrame worldFrame;

   // support
   QuadrupedSupportPolygon supportPolygon;
   FramePoint supportCentroid;
   FrameOrientation supportOrientation;

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

   // trajectories
   private double bodyYawSetpoint;
   private final PiecewisePeriodicDcmTrajectory nominalPeriodicDcmTrajectory;
   private final QuadrantDependentList<ThreeDoFSwingFootTrajectory> swingFootTrajectory;

   // state machine
   public enum TrotState
   {
      QUAD_SUPPORT, HIND_LEFT_FRONT_RIGHT_SUPPORT, HIND_RIGHT_FRONT_LEFT_SUPPORT
   }
   public enum TrotEvent
   {
      TIMEOUT
   }
   private final StateMachine<TrotState, TrotEvent> trotStateMachine;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   public QuadrupedVirtualModelBasedTrotController(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedRobotParameters robotParameters,
         QuadrupedControllerInputProviderInterface inputProvider, QuadrupedReferenceFrames referenceFrames, QuadrupedTaskSpaceEstimator taskSpaceEstimator,
         QuadrupedTaskSpaceController taskSpaceController)
   {
      this.fullRobotModel = runtimeEnvironment.getFullRobotModel();
      this.robotTimestamp = runtimeEnvironment.getRobotTimestamp();
      this.yoGraphicsListRegistry = runtimeEnvironment.getGraphicsListRegistry();
      this.jointNameMap = robotParameters.getJointMap();
      this.controlDT = runtimeEnvironment.getControlDT();
      this.gravity = 9.81;
      this.mass = fullRobotModel.getTotalMass();
      this.inputProvider = inputProvider;
      this.taskSpaceEstimator = taskSpaceEstimator;
      this.taskSpaceController = taskSpaceController;

      // frames
      ReferenceFrame comFrame = referenceFrames.getCenterOfMassZUpFrame();
      supportFrame = new PoseReferenceFrame("SupportFrame", ReferenceFrame.getWorldFrame());
      worldFrame = ReferenceFrame.getWorldFrame();

      // support
      supportPolygon = new QuadrupedSupportPolygon();
      supportCentroid = new FramePoint();
      supportOrientation = new FrameOrientation();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         supportPolygon.setFootstep(robotQuadrant, new FramePoint());
      }

      // dcm controller
      dcmPositionEstimate = new FramePoint();
      dcmPositionSetpoint = new FramePoint();
      dcmVelocitySetpoint = new FrameVector();
      dcmPositionController = new DivergentComponentOfMotionController("dcmPosition", comFrame, controlDT, mass, gravity, inputProvider.getComPositionInput().getZ(), registry);

      // task space controllers
      taskSpaceCommands = new QuadrupedTaskSpaceCommands();
      taskSpaceSetpoints = new QuadrupedTaskSpaceSetpoints();
      taskSpaceEstimates = new QuadrupedTaskSpaceEstimates();
      taskSpaceControllerSettings = new QuadrupedTaskSpaceControllerSettings();

      // trajectories
      swingFootTrajectory = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         swingFootTrajectory.set(robotQuadrant, new ThreeDoFSwingFootTrajectory());
      }
      nominalPeriodicDcmTrajectory = new PiecewisePeriodicDcmTrajectory(1, gravity, inputProvider.getComPositionInput().getZ(), null);

      // state machine
      StateMachineBuilder<TrotState, TrotEvent> stateMachineBuilder = new StateMachineBuilder<>(TrotState.class, "TrotState", registry);
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
      taskSpaceEstimates.getComPosition().changeFrame(worldFrame);
      taskSpaceEstimates.getComVelocity().changeFrame(worldFrame);
      dcmPositionEstimate.changeFrame(worldFrame);
      dcmPositionEstimate.set(taskSpaceEstimates.getComVelocity());
      dcmPositionEstimate.scale(1.0 / dcmPositionController.getNaturalFrequency());
      dcmPositionEstimate.add(taskSpaceEstimates.getComPosition());

      // compute support frame
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         taskSpaceEstimates.getSolePosition().get(robotQuadrant).changeFrame(supportPolygon.getReferenceFrame());
         supportPolygon.setFootstep(robotQuadrant, taskSpaceEstimates.getSolePosition().get(robotQuadrant));
         taskSpaceEstimates.getSolePosition().get(robotQuadrant).changeFrame(ReferenceFrame.getWorldFrame());
      }
      double minFrontFootHeight = Math.min(taskSpaceEstimates.getSolePosition().get(RobotQuadrant.FRONT_LEFT).getZ(), taskSpaceEstimates.getSolePosition().get(RobotQuadrant.FRONT_RIGHT).getZ());
      double minHindFootHeight = Math.min(taskSpaceEstimates.getSolePosition().get(RobotQuadrant.HIND_LEFT).getZ(), taskSpaceEstimates.getSolePosition().get(RobotQuadrant.HIND_RIGHT).getZ());

      // compute support frame (centroid and nominal orientation)
      supportCentroid.changeFrame(supportPolygon.getReferenceFrame());
      supportPolygon.getCentroid(supportCentroid);
      supportCentroid.changeFrame(ReferenceFrame.getWorldFrame());
      supportCentroid.setZ((minFrontFootHeight + minHindFootHeight) / 2.0);
      supportOrientation.changeFrame(supportPolygon.getReferenceFrame());
      supportOrientation.setYawPitchRoll(supportPolygon.getNominalYaw(), 0.0, 0.0);
      supportFrame.setPoseAndUpdate(supportCentroid, supportOrientation);
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
            RotationTools.computePitch(inputProvider.getBodyOrientationInput()) + supportPolygon.getNominalPitch(),
                  RotationTools.computeRoll(inputProvider.getBodyOrientationInput()));
      taskSpaceSetpoints.getBodyAngularVelocity().setToZero();
      taskSpaceSetpoints.getComTorqueFeedforward().setToZero();

      // update joint setpoints
      taskSpaceController.compute(taskSpaceControllerSettings, taskSpaceSetpoints, taskSpaceEstimates, taskSpaceCommands);
      taskSpaceSetpoints.getBodyOrientation().changeFrame(worldFrame);
   }

   @Override public QuadrupedForceControllerEvent process()
   {
      dcmPositionController.setComHeight(inputProvider.getComPositionInput().getZ());
      updateEstimates();
      updateSetpoints();
      return null;
   }

   @Override public void onEntry()
   {
      // initialize dcm controller
      dcmPositionController.setGains(
            dcmPositionProportionalGainsProperty.get(),
            dcmPositionDerivativeGainsProperty.get(),
            dcmPositionIntegralGainsProperty.get(),
            dcmPositionMaxIntegralErrorProperty.get());
      dcmPositionController.reset();

      // initialize task space controller
      taskSpaceEstimator.compute(taskSpaceEstimates);
      taskSpaceSetpoints.initialize(taskSpaceEstimates);
      taskSpaceControllerSettings.initialize();
      taskSpaceControllerSettings.setComForceCommandWeights(1.0, 1.0, 1.0);
      taskSpaceControllerSettings.setComTorqueCommandWeights(1.0, 1.0, 1.0);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         taskSpaceControllerSettings.setSoleForceCommandWeights(robotQuadrant, 0.0, 0.0, 0.0);
         taskSpaceControllerSettings.setContactState(robotQuadrant, ContactState.IN_CONTACT);
         taskSpaceControllerSettings.setSolePositionFeedbackGainsToZero(robotQuadrant);
      }
      taskSpaceControllerSettings.setBodyOrientationFeedbackGains(
            bodyOrientationProportionalGainsProperty.get(),
            bodyOrientationDerivativeGainsProperty.get(),
            bodyOrientationIntegralGainsProperty.get(),
            bodyOrientationMaxIntegralErrorProperty.get()
      );
      taskSpaceControllerSettings.setComPositionFeedbackGains(
            comPositionProportionalGainsProperty.get(),
            comPositionDerivativeGainsProperty.get(),
            comPositionIntegralGainsProperty.get(),
            comPositionMaxIntegralErrorProperty.get()
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

   private void computeNominalCmpPositionAtSoS(RobotQuadrant hindSupportQuadrant, RobotQuadrant frontSupportQuadrant, FramePoint nominalCmpPositionAtSoS)
   {
      taskSpaceEstimates.getSolePosition(hindSupportQuadrant).changeFrame(worldFrame);
      taskSpaceEstimates.getSolePosition(frontSupportQuadrant).changeFrame(worldFrame);
      nominalCmpPositionAtSoS.setToZero(worldFrame);
      nominalCmpPositionAtSoS.add(taskSpaceEstimates.getSolePosition(hindSupportQuadrant));
      nominalCmpPositionAtSoS.add(taskSpaceEstimates.getSolePosition(frontSupportQuadrant));
      nominalCmpPositionAtSoS.scale(0.5);
   }

   private void computeNominalCmpPositionAtEoS(FramePoint nominalCmpPositionAtSoS, FramePoint nominalCmpPositionAtEoS)
   {
      taskSpaceSetpoints.getBodyOrientation().changeFrame(worldFrame);
      double bodyYaw = taskSpaceSetpoints.getBodyOrientation().getYaw() + inputProvider.getPlanarVelocityInput().getZ() * doubleSupportDurationProperty.get();
      double xStride = inputProvider.getPlanarVelocityInput().getX() * doubleSupportDurationProperty.get();
      double yStride = inputProvider.getPlanarVelocityInput().getY() * doubleSupportDurationProperty.get();
      double xOffset = Math.cos(bodyYaw) * xStride - Math.sin(bodyYaw) * yStride;
      double yOffset = Math.sin(bodyYaw) * xStride + Math.cos(bodyYaw) * yStride;
      nominalCmpPositionAtEoS.setIncludingFrame(nominalCmpPositionAtSoS);
      nominalCmpPositionAtEoS.changeFrame(worldFrame);
      nominalCmpPositionAtEoS.add(xOffset, yOffset, 0.0);
   }

   private void computeNominalPeriodicDcmTrajectory(FramePoint nominalCmpPositionAtSoS, FramePoint nominalCmpPositionAtEoS)
   {
      double timeAtEoS = doubleSupportDurationProperty.get();
      double relativeYawAtEoS = inputProvider.getPlanarVelocityInput().getZ() * timeAtEoS;
      nominalPeriodicDcmTrajectory.setComHeight(inputProvider.getComPositionInput().getZ());
      nominalPeriodicDcmTrajectory.initializeTrajectory(0.0, nominalCmpPositionAtSoS, timeAtEoS, nominalCmpPositionAtEoS, relativeYawAtEoS);
   }

   private void computeNominalDcmPositionAtSoS(FramePoint nominalCmpPositionAtSoS, FramePoint nominalCmpPositionAtEoS, FramePoint nominalDcmPositionAtSoS)
   {
      computeNominalPeriodicDcmTrajectory(nominalCmpPositionAtSoS, nominalCmpPositionAtEoS);
      nominalPeriodicDcmTrajectory.computeTrajectory(0.0);
      nominalPeriodicDcmTrajectory.getPosition(nominalDcmPositionAtSoS);
   }

   private void computeNominalDcmPositionAtEoS(FramePoint nominalCmpPositionAtSoS, FramePoint nominalCmpPositionAtEoS, FramePoint nominalDcmPositionAtEoS)
   {
      computeNominalPeriodicDcmTrajectory(nominalCmpPositionAtSoS, nominalCmpPositionAtEoS);
      double timeAtEoS = doubleSupportDurationProperty.get();
      nominalPeriodicDcmTrajectory.computeTrajectory(timeAtEoS);
      nominalPeriodicDcmTrajectory.getPosition(nominalDcmPositionAtEoS);
   }

   private void computeFootholdPosition(RobotQuadrant robotQuadrant, FramePoint cmpPosition, double bodyYaw, FramePoint footholdPosition)
   {
      double xOffset, yOffset;

      // compute foothold position based on the nominal stance and desired cmp
      taskSpaceEstimates.getSolePosition(robotQuadrant).changeFrame(worldFrame);
      double xStance = robotQuadrant.getEnd().negateIfHindEnd(stanceLengthNominalProperty.get() / 2);
      double yStance = robotQuadrant.getSide().negateIfRightSide(stanceWidthNominalProperty.get() / 2);
      xOffset = Math.cos(bodyYaw) * xStance - Math.sin(bodyYaw) * yStance;
      yOffset = Math.sin(bodyYaw) * xStance + Math.cos(bodyYaw) * yStance;
      footholdPosition.setIncludingFrame(cmpPosition);
      footholdPosition.changeFrame(worldFrame);
      footholdPosition.add(xOffset, yOffset, 0.0);

      // compute foothold height based on estimated ground slope
      taskSpaceEstimates.getSolePosition(robotQuadrant).changeFrame(worldFrame);
      double xStride = footholdPosition.getX() - taskSpaceEstimates.getSolePosition(robotQuadrant).getX();
      double yStride = footholdPosition.getY() - taskSpaceEstimates.getSolePosition(robotQuadrant).getY();
      xOffset = Math.cos(-bodyYaw) * xStride - Math.sin(-bodyYaw) * yStride;
      yOffset = Math.sin(-bodyYaw) * xStride + Math.cos(-bodyYaw) * yStride;
      footholdPosition.setZ(taskSpaceEstimates.getSolePosition(robotQuadrant).getZ());
      footholdPosition.add(0, 0, -xOffset * Math.tan(supportPolygon.getNominalPitch()));
      footholdPosition.add(0, 0, yOffset * Math.tan(supportPolygon.getNominalRoll()));
   }

   private class QuadSupportState implements StateMachineState<TrotEvent>
   {
      private double initialTime;
      private final ThreeDoFMinimumJerkTrajectory dcmTrajectory;
      private final FramePoint cmpPositionAtSoSNominal;
      private final FramePoint cmpPositionAtEoSNominal;
      private final FramePoint dcmPositionAtSoSNominal;

      public QuadSupportState()
      {
         initialTime = 0.0;
         dcmTrajectory = new ThreeDoFMinimumJerkTrajectory();
         cmpPositionAtSoSNominal = new FramePoint(worldFrame);
         cmpPositionAtEoSNominal = new FramePoint(worldFrame);
         dcmPositionAtSoSNominal = new FramePoint(worldFrame);
      }

      @Override public void onEntry()
      {
         initialTime = robotTimestamp.getDoubleValue();

         // compute desired dcm position at start of step
         computeNominalCmpPositionAtSoS(RobotQuadrant.HIND_LEFT, RobotQuadrant.FRONT_RIGHT, cmpPositionAtSoSNominal);
         computeNominalCmpPositionAtEoS(cmpPositionAtSoSNominal, cmpPositionAtEoSNominal);
         computeNominalDcmPositionAtSoS(cmpPositionAtSoSNominal, cmpPositionAtEoSNominal, dcmPositionAtSoSNominal);

         // compute desired dcm trajectory
         dcmTrajectory.initializeTrajectory(dcmPositionEstimate, dcmPositionAtSoSNominal, quadSupportDurationProperty.get());

         // initialize contact state
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            taskSpaceControllerSettings.setContactState(robotQuadrant, ContactState.IN_CONTACT);
            taskSpaceControllerSettings.setPressureUpperLimit(robotQuadrant, Double.MAX_VALUE);
         }
      }

      @Override public TrotEvent process()
      {
         double currentTime = robotTimestamp.getDoubleValue();

         // compute dcm setpoint
         dcmTrajectory.computeTrajectory(currentTime - initialTime);
         dcmTrajectory.getPosition(dcmPositionSetpoint);
         dcmTrajectory.getVelocity(dcmVelocitySetpoint);

         // trigger touch down event
         if (currentTime > initialTime + quadSupportDurationProperty.get())
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
      private final FramePoint cmpPositionAtSoSNominal;
      private final FramePoint cmpPositionAtEoSNominal;
      private final FramePoint dcmPositionAtEoSNominal;
      private final FramePoint cmpPositionAtEoS;
      private final FramePoint dcmPositionAtEoS;
      private final FramePoint footholdPosition;

      public DoubleSupportState(RobotQuadrant hindSupportQuadrant, RobotQuadrant frontSupportQuadrant)
      {
         initialTime = 0.0;
         supportQuadrants = new RobotQuadrant[] {hindSupportQuadrant, frontSupportQuadrant};
         swingQuadrants = new RobotQuadrant[] {hindSupportQuadrant.getAcrossBodyQuadrant(), frontSupportQuadrant.getAcrossBodyQuadrant()};
         dcmTrajectory = new PiecewiseForwardDcmTrajectory(1, gravity, dcmPositionController.getComHeight(), null);
         cmpPositionAtSoSNominal = new FramePoint(worldFrame);
         cmpPositionAtEoSNominal = new FramePoint(worldFrame);
         dcmPositionAtEoSNominal = new FramePoint(worldFrame);
         cmpPositionAtEoS = new FramePoint(worldFrame);
         dcmPositionAtEoS = new FramePoint(worldFrame);
         footholdPosition = new FramePoint(worldFrame);
      }

      @Override public void onEntry()
      {
         initialTime = robotTimestamp.getDoubleValue();

         // compute desired dcm position at end of step
         computeNominalCmpPositionAtSoS(supportQuadrants[0], supportQuadrants[1], cmpPositionAtSoSNominal);
         computeNominalCmpPositionAtEoS(cmpPositionAtSoSNominal, cmpPositionAtEoSNominal);
         computeNominalDcmPositionAtEoS(cmpPositionAtSoSNominal, cmpPositionAtEoSNominal, dcmPositionAtEoSNominal);

         // compute desired dcm trajectory
         dcmPositionEstimate.changeFrame(worldFrame);
         dcmTrajectory.setComHeight(dcmPositionController.getComHeight());
         dcmTrajectory.initializeTrajectory(initialTime, cmpPositionAtSoSNominal, dcmPositionEstimate);
         dcmTrajectory.computeTrajectory(initialTime + doubleSupportDurationProperty.get());
         dcmTrajectory.getPosition(dcmPositionAtEoS);

         // compute desired cmp position at end of step
         cmpPositionAtEoS.set(dcmPositionAtEoS);
         cmpPositionAtEoS.sub(dcmPositionAtEoSNominal);
         cmpPositionAtEoS.add(cmpPositionAtEoSNominal);
         cmpPositionAtEoS.setZ(cmpPositionAtEoSNominal.getZ());

         // compute desired body yaw at end of step
         taskSpaceSetpoints.getBodyOrientation().changeFrame(worldFrame);
         double bodyYawAtSoS = taskSpaceSetpoints.getBodyOrientation().getYaw();
         double bodyYawAtEoS = bodyYawAtSoS + inputProvider.getPlanarVelocityInput().getZ() * doubleSupportDurationProperty.get();

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
               swingTrajectoryGroundClearanceProperty.get(), doubleSupportDurationProperty.get());

            // initialize sole position feedback gains
            taskSpaceControllerSettings.setSolePositionFeedbackGainsToZero(supportQuadrant);
            taskSpaceControllerSettings.setSolePositionFeedbackGains(swingQuadrant,
                  swingPositionProportionalGainsProperty.get(),
                  swingPositionDerivativeGainsProperty.get(),
                  swingPositionIntegralGainsProperty.get(),
                  swingPositionMaxIntegralErrorProperty.get()
            );

            // initialize contact state
            taskSpaceControllerSettings.setContactState(swingQuadrant, ContactState.NO_CONTACT);
            taskSpaceControllerSettings.setContactState(supportQuadrant, ContactState.IN_CONTACT);
            taskSpaceControllerSettings.setPressureUpperLimit(swingQuadrant, noContactPressureLimitProperty.get());
            taskSpaceControllerSettings.setPressureUpperLimit(supportQuadrant, Double.MAX_VALUE);
         }
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
         if (currentTime > initialTime + doubleSupportDurationProperty.get())
            return TrotEvent.TIMEOUT;
         else
            return null;
      }

      @Override public void onExit()
      {
      }
   }
}
