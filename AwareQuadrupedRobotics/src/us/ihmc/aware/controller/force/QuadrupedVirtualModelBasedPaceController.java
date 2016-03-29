package us.ihmc.aware.controller.force;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.aware.controller.common.DivergentComponentOfMotionController;
import us.ihmc.aware.controller.force.taskSpaceController.*;
import us.ihmc.aware.parameters.QuadrupedRuntimeEnvironment;
import us.ihmc.aware.params.ParameterMap;
import us.ihmc.aware.params.ParameterMapRepository;
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
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class QuadrupedVirtualModelBasedPaceController implements QuadrupedForceController
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
   private final ParameterMap params;
   private final static String JOINT_DAMPING = "jointDamping";
   private final static String BODY_ORIENTATION_PROPORTIONAL_GAINS = "bodyOrientationProportionalGains";
   private final static String BODY_ORIENTATION_DERIVATIVE_GAINS = "bodyOrientationDerivativeGains";
   private final static String BODY_ORIENTATION_INTEGRAL_GAINS = "bodyOrientationIntegralGains";
   private final static String BODY_ORIENTATION_MAX_INTEGRAL_ERROR = "bodyOrientationMaxIntegralError";
   private final static String DCM_POSITION_PROPORTIONAL_GAINS = "dcmPositionProportionalGains";
   private final static String DCM_POSITION_DERIVATIVE_GAINS = "dcmPositionDerivativeGains";
   private final static String DCM_POSITION_INTEGRAL_GAINS = "dcmPositionIntegralGains";
   private final static String DCM_POSITION_MAX_INTEGRAL_ERROR = "dcmPositionMaxIntegralError";
   private final static String COM_POSITION_PROPORTIONAL_GAINS = "comPositionProportionalGains";
   private final static String COM_POSITION_DERIVATIVE_GAINS = "comPositionDerivativeGains";
   private final static String COM_POSITION_INTEGRAL_GAINS = "comPositionIntegralGains";
   private final static String COM_POSITION_MAX_INTEGRAL_ERROR = "comPositionMaxIntegralError";
   private final static String SWING_POSITION_PROPORTIONAL_GAINS = "swingPositionProportionalGains";
   private final static String SWING_POSITION_DERIVATIVE_GAINS = "swingPositionDerivativeGains";
   private final static String SWING_POSITION_INTEGRAL_GAINS = "swingPositionIntegralGains";
   private final static String SWING_POSITION_MAX_INTEGRAL_ERROR = "swingPositionMaxIntegralError";
   private final static String SWING_TRAJECTORY_GROUND_CLEARANCE = "swingTrajectoryGroundClearance";
   private final static String QUAD_SUPPORT_DURATION = "quadSupportDuration";
   private final static String DOUBLE_SUPPORT_DURATION = "doubleSupportDuration";
   private final static String STANCE_WIDTH_NOMINAL = "stanceWidthNominal";
   private final static String STANCE_LENGTH_NOMINAL = "stanceLengthNominal";
   private final static String NO_CONTACT_PRESSURE_LIMIT = "noContactPressureLimit";

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
      QUAD_SUPPORT, HIND_LEFT_FRONT_LEFT_SUPPORT, HIND_RIGHT_FRONT_RIGHT_SUPPORT
   }
   public enum TrotEvent
   {
      TIMEOUT
   }
   private final StateMachine<TrotState, TrotEvent> trotStateMachine;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   public QuadrupedVirtualModelBasedPaceController(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedRobotParameters robotParameters,
         ParameterMapRepository parameterMapRepository, QuadrupedControllerInputProviderInterface inputProvider, QuadrupedReferenceFrames referenceFrames, QuadrupedTaskSpaceEstimator taskSpaceEstimator, QuadrupedTaskSpaceController taskSpaceController)
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

      // parameters
      this.params = parameterMapRepository.get(QuadrupedVirtualModelBasedPaceController.class);
      params.setDefault(JOINT_DAMPING, 2);
      params.setDefault(BODY_ORIENTATION_PROPORTIONAL_GAINS, 5000, 5000, 5000);
      params.setDefault(BODY_ORIENTATION_DERIVATIVE_GAINS, 750, 750, 750);
      params.setDefault(BODY_ORIENTATION_INTEGRAL_GAINS, 0, 0, 0);
      params.setDefault(BODY_ORIENTATION_MAX_INTEGRAL_ERROR, 0);
      params.setDefault(COM_POSITION_PROPORTIONAL_GAINS, 0, 0, 5000);
      params.setDefault(COM_POSITION_DERIVATIVE_GAINS, 0, 0, 750);
      params.setDefault(COM_POSITION_INTEGRAL_GAINS, 0, 0, 0);
      params.setDefault(COM_POSITION_MAX_INTEGRAL_ERROR, 0);
      params.setDefault(DCM_POSITION_PROPORTIONAL_GAINS, 1, 1, 0);
      params.setDefault(DCM_POSITION_DERIVATIVE_GAINS, 0, 0, 0);
      params.setDefault(DCM_POSITION_INTEGRAL_GAINS, 0, 0, 0);
      params.setDefault(DCM_POSITION_MAX_INTEGRAL_ERROR, 0);
      params.setDefault(SWING_POSITION_PROPORTIONAL_GAINS, 50000, 50000, 100000);
      params.setDefault(SWING_POSITION_DERIVATIVE_GAINS, 500, 500, 500);
      params.setDefault(SWING_POSITION_INTEGRAL_GAINS, 0, 0, 0);
      params.setDefault(SWING_POSITION_MAX_INTEGRAL_ERROR, 0);
      params.setDefault(SWING_TRAJECTORY_GROUND_CLEARANCE, 0.1);
      params.setDefault(QUAD_SUPPORT_DURATION, 1.00);
      params.setDefault(DOUBLE_SUPPORT_DURATION, 0.4);
      params.setDefault(STANCE_WIDTH_NOMINAL, 0.25);
      params.setDefault(STANCE_LENGTH_NOMINAL, 1.1);
      params.setDefault(NO_CONTACT_PRESSURE_LIMIT, 75);

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
      nominalPeriodicDcmTrajectory = new PiecewisePeriodicDcmTrajectory(2, gravity, inputProvider.getComPositionInput().getZ(), null);

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
            params.getVolatileArray(DCM_POSITION_PROPORTIONAL_GAINS),
            params.getVolatileArray(DCM_POSITION_DERIVATIVE_GAINS),
            params.getVolatileArray(DCM_POSITION_INTEGRAL_GAINS),
            params.get(DCM_POSITION_MAX_INTEGRAL_ERROR));
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
            params.getVolatileArray(BODY_ORIENTATION_PROPORTIONAL_GAINS),
            params.getVolatileArray(BODY_ORIENTATION_DERIVATIVE_GAINS),
            params.getVolatileArray(BODY_ORIENTATION_INTEGRAL_GAINS),
            params.get(BODY_ORIENTATION_MAX_INTEGRAL_ERROR)
      );
      taskSpaceControllerSettings.setComPositionFeedbackGains(
            params.getVolatileArray(COM_POSITION_PROPORTIONAL_GAINS),
            params.getVolatileArray(COM_POSITION_DERIVATIVE_GAINS),
            params.getVolatileArray(COM_POSITION_INTEGRAL_GAINS),
            params.get(COM_POSITION_MAX_INTEGRAL_ERROR)
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

   private void computeNominalCmpPositions(RobotQuadrant hindSupportQuadrant, RobotQuadrant frontSupportQuadrant, FramePoint[] nominalCmpPositionAtSoS, FramePoint nominalCmpPositionAtEoS)
   {
      double bodyYaw, yStance, xStride, yStride, xOffset, yOffset;

      taskSpaceEstimates.getSolePosition(hindSupportQuadrant).changeFrame(worldFrame);
      taskSpaceEstimates.getSolePosition(frontSupportQuadrant).changeFrame(worldFrame);
      nominalCmpPositionAtSoS[0].setToZero(worldFrame);
      nominalCmpPositionAtSoS[0].add(taskSpaceEstimates.getSolePosition(hindSupportQuadrant));
      nominalCmpPositionAtSoS[0].add(taskSpaceEstimates.getSolePosition(frontSupportQuadrant));
      nominalCmpPositionAtSoS[0].scale(0.5);

      taskSpaceSetpoints.getBodyOrientation().changeFrame(worldFrame);
      bodyYaw = taskSpaceSetpoints.getBodyOrientation().getYaw() + inputProvider.getPlanarVelocityInput().getZ() * params.get(DOUBLE_SUPPORT_DURATION);
      yStance = hindSupportQuadrant.getSide().negateIfLeftSide(params.get(STANCE_WIDTH_NOMINAL));
      xStride = inputProvider.getPlanarVelocityInput().getX() * params.get(DOUBLE_SUPPORT_DURATION);
      yStride = inputProvider.getPlanarVelocityInput().getY() * params.get(DOUBLE_SUPPORT_DURATION) * 2;
      xOffset =-Math.sin(bodyYaw) * yStance + Math.cos(bodyYaw) * xStride - Math.sin(bodyYaw) * yStride;
      yOffset = Math.cos(bodyYaw) * yStance + Math.sin(bodyYaw) * xStride + Math.cos(bodyYaw) * yStride;
      nominalCmpPositionAtSoS[1].setIncludingFrame(nominalCmpPositionAtSoS[0]);
      nominalCmpPositionAtSoS[1].changeFrame(worldFrame);
      nominalCmpPositionAtSoS[1].add(xOffset, yOffset, 0.0);

      taskSpaceSetpoints.getBodyOrientation().changeFrame(worldFrame);
      bodyYaw = taskSpaceSetpoints.getBodyOrientation().getYaw() + inputProvider.getPlanarVelocityInput().getZ() * 2 * params.get(DOUBLE_SUPPORT_DURATION);
      yStance = hindSupportQuadrant.getSide().negateIfRightSide(params.get(STANCE_WIDTH_NOMINAL));
      xStride = inputProvider.getPlanarVelocityInput().getX() * params.get(DOUBLE_SUPPORT_DURATION);
      yStride = inputProvider.getPlanarVelocityInput().getY() * params.get(DOUBLE_SUPPORT_DURATION) * 2;
      xOffset =-Math.sin(bodyYaw) * yStance + Math.cos(bodyYaw) * xStride - Math.sin(bodyYaw) * yStride;
      yOffset = Math.cos(bodyYaw) * yStance + Math.sin(bodyYaw) * xStride + Math.cos(bodyYaw) * yStride;
      nominalCmpPositionAtEoS.setIncludingFrame(nominalCmpPositionAtSoS[1]);
      nominalCmpPositionAtEoS.changeFrame(worldFrame);
      nominalCmpPositionAtEoS.add(xOffset, yOffset, 0.0);
   }

   private void computeNominalPeriodicDcmTrajectory(FramePoint[] nominalCmpPositionAtSoS, double[] timeAtSoS, FramePoint nominalCmpPositionAtEoS, double timeAtEoS)
   {
      double relativeYawAtEoS = inputProvider.getPlanarVelocityInput().getZ() * timeAtEoS - timeAtSoS[0];
      nominalPeriodicDcmTrajectory.setComHeight(inputProvider.getComPositionInput().getZ());
      nominalPeriodicDcmTrajectory.initializeTrajectory(2, timeAtSoS, nominalCmpPositionAtSoS, timeAtEoS, nominalCmpPositionAtEoS, relativeYawAtEoS);
   }

   private void computeNominalDcmPositionAtSoS(int step, FramePoint[] nominalCmpPositionAtSoS, double[] timeAtSoS, FramePoint nominalCmpPositionAtEoS, double timeAtEoS, FramePoint nominalDcmPositionAtSoS)
   {
      computeNominalPeriodicDcmTrajectory(nominalCmpPositionAtSoS, timeAtSoS, nominalCmpPositionAtEoS, timeAtEoS);
      nominalPeriodicDcmTrajectory.computeTrajectory(timeAtSoS[step]);
      nominalPeriodicDcmTrajectory.getPosition(nominalDcmPositionAtSoS);
   }

   private void computeNominalDcmPositionAtEoS(FramePoint[] nominalCmpPositionAtSoS, double[] timeAtSoS, FramePoint nominalCmpPositionAtEoS, double timeAtEoS, FramePoint nominalDcmPositionAtEoS)
   {
      computeNominalPeriodicDcmTrajectory(nominalCmpPositionAtSoS, timeAtSoS, nominalCmpPositionAtEoS, timeAtEoS);
      nominalPeriodicDcmTrajectory.computeTrajectory(timeAtEoS);
      nominalPeriodicDcmTrajectory.getPosition(nominalDcmPositionAtEoS);
   }

   private void computeFootholdPosition(RobotQuadrant robotQuadrant, FramePoint cmpPosition, double bodyYaw, FramePoint footholdPosition)
   {
      double xOffset, yOffset;

      // compute foothold position based on the nominal stance and desired cmp
      taskSpaceEstimates.getSolePosition(robotQuadrant).changeFrame(worldFrame);
      double xStance = robotQuadrant.getEnd().negateIfHindEnd(params.get(STANCE_LENGTH_NOMINAL) / 2);
      double yStance = 0.0;
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
      private final double[] timeAtSoS;
      private double timeAtEoS;
      private final FramePoint[] cmpPositionAtSoSNominal;
      private final FramePoint cmpPositionAtEoSNominal;
      private final FramePoint dcmPositionAtSoSNominal;

      public QuadSupportState()
      {
         initialTime = 0.0;
         dcmTrajectory = new ThreeDoFMinimumJerkTrajectory();
         timeAtSoS = new double[] {0.0, 0.0};
         timeAtEoS = 0.0;
         cmpPositionAtSoSNominal = new FramePoint[] {new FramePoint(), new FramePoint()};
         cmpPositionAtEoSNominal = new FramePoint();
         dcmPositionAtSoSNominal = new FramePoint();
      }

      @Override public void onEntry()
      {
         initialTime = robotTimestamp.getDoubleValue();

         // compute desired dcm position at start of step
         timeAtSoS[0] = 0.0;
         timeAtSoS[1] = params.get(DOUBLE_SUPPORT_DURATION);
         timeAtEoS = 2 * params.get(DOUBLE_SUPPORT_DURATION);
         computeNominalCmpPositions(RobotQuadrant.HIND_LEFT, RobotQuadrant.FRONT_LEFT, cmpPositionAtSoSNominal, cmpPositionAtEoSNominal);
         computeNominalDcmPositionAtSoS(0, cmpPositionAtSoSNominal, timeAtSoS, cmpPositionAtEoSNominal, timeAtEoS, dcmPositionAtSoSNominal);

         // compute desired dcm trajectory
         dcmTrajectory.initializeTrajectory(dcmPositionEstimate, dcmPositionAtSoSNominal, params.get(QUAD_SUPPORT_DURATION));

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
         if (currentTime > initialTime + params.get(QUAD_SUPPORT_DURATION))
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
      private final double[] timeAtSoS;
      private double timeAtEoS;
      private final FramePoint[] cmpPositionAtSoSNominal;
      private final FramePoint cmpPositionAtEoSNominal;
      private final FramePoint dcmPositionAtSoSNominal;
      private final FramePoint cmpPositionAtEoS;
      private final FramePoint dcmPositionAtSoS;
      private final FramePoint footholdPosition;

      public DoubleSupportState(RobotQuadrant hindSupportQuadrant, RobotQuadrant frontSupportQuadrant)
      {
         initialTime = 0.0;
         supportQuadrants = new RobotQuadrant[] {hindSupportQuadrant, frontSupportQuadrant};
         swingQuadrants = new RobotQuadrant[] {hindSupportQuadrant.getAcrossBodyQuadrant(), frontSupportQuadrant.getAcrossBodyQuadrant()};
         dcmTrajectory = new PiecewiseForwardDcmTrajectory(1, gravity, dcmPositionController.getComHeight(), null);
         timeAtSoS = new double[] {0.0, 0.0};
         timeAtEoS = 0.0;
         cmpPositionAtSoSNominal = new FramePoint[] {new FramePoint(), new FramePoint()};
         cmpPositionAtEoSNominal = new FramePoint();
         dcmPositionAtSoSNominal = new FramePoint();
         cmpPositionAtEoS = new FramePoint();
         dcmPositionAtSoS = new FramePoint();
         footholdPosition = new FramePoint();
      }

      @Override public void onEntry()
      {
         initialTime = robotTimestamp.getDoubleValue();

         // compute desired dcm position at end of step
         timeAtSoS[0] = 0.0;
         timeAtSoS[1] = params.get(DOUBLE_SUPPORT_DURATION);
         timeAtEoS = 2 * params.get(DOUBLE_SUPPORT_DURATION);
         computeNominalCmpPositions(supportQuadrants[0], supportQuadrants[1], cmpPositionAtSoSNominal, cmpPositionAtEoSNominal);
         computeNominalDcmPositionAtSoS(1, cmpPositionAtSoSNominal, timeAtSoS, cmpPositionAtEoSNominal, timeAtEoS, dcmPositionAtSoSNominal);

         // compute desired dcm trajectory
         dcmPositionEstimate.changeFrame(worldFrame);
         dcmTrajectory.setComHeight(dcmPositionController.getComHeight());
         dcmTrajectory.initializeTrajectory(initialTime, cmpPositionAtSoSNominal[0], dcmPositionEstimate);
         dcmTrajectory.computeTrajectory(initialTime + params.get(DOUBLE_SUPPORT_DURATION));
         dcmTrajectory.getPosition(dcmPositionAtSoS);

         // compute desired cmp position at end of step
         cmpPositionAtEoS.set(dcmPositionAtSoS);
         cmpPositionAtEoS.sub(dcmPositionAtSoSNominal);
         cmpPositionAtEoS.add(cmpPositionAtSoSNominal[1]);
         cmpPositionAtEoS.setZ(cmpPositionAtSoSNominal[1].getZ());

         // compute desired body yaw at end of step
         taskSpaceSetpoints.getBodyOrientation().changeFrame(worldFrame);
         double bodyYawAtSoS = taskSpaceSetpoints.getBodyOrientation().getYaw();
         double bodyYawAtEoS = bodyYawAtSoS + inputProvider.getPlanarVelocityInput().getZ() * params.get(DOUBLE_SUPPORT_DURATION);

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
               params.get(SWING_TRAJECTORY_GROUND_CLEARANCE), params.get(DOUBLE_SUPPORT_DURATION));

            // initialize sole position feedback gains
            taskSpaceControllerSettings.setSolePositionFeedbackGainsToZero(supportQuadrant);
            taskSpaceControllerSettings.setSolePositionFeedbackGains(swingQuadrant,
                  params.getVolatileArray(SWING_POSITION_PROPORTIONAL_GAINS),
                  params.getVolatileArray(SWING_POSITION_DERIVATIVE_GAINS),
                  params.getVolatileArray(SWING_POSITION_INTEGRAL_GAINS),
                  params.get(SWING_POSITION_MAX_INTEGRAL_ERROR)
            );

            // initialize contact state
            taskSpaceControllerSettings.setContactState(swingQuadrant, ContactState.NO_CONTACT);
            taskSpaceControllerSettings.setContactState(supportQuadrant, ContactState.IN_CONTACT);
            taskSpaceControllerSettings.setPressureUpperLimit(swingQuadrant, params.get(NO_CONTACT_PRESSURE_LIMIT));
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
         if (currentTime > initialTime + params.get(DOUBLE_SUPPORT_DURATION))
            return TrotEvent.TIMEOUT;
         else
            return null;
      }

      @Override public void onExit()
      {
      }
   }
}
