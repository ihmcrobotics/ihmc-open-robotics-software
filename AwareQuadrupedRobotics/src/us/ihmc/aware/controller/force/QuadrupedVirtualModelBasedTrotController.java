package us.ihmc.aware.controller.force;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.aware.controller.common.DivergentComponentOfMotionController;
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
import us.ihmc.aware.vmc.*;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointName;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointNameMap;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.quadrupedRobotics.virtualModelController.QuadrupedJointLimits;
import us.ihmc.robotics.controllers.AxisAngleOrientationController;
import us.ihmc.robotics.controllers.EuclideanPositionController;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.ArtifactList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactPolygon;

import java.awt.*;

public class QuadrupedVirtualModelBasedTrotController implements QuadrupedForceController
{
   private final SDFFullRobotModel fullRobotModel;
   private final DoubleYoVariable robotTimestamp;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;
   private final QuadrupedJointNameMap jointNameMap;
   private final double controlDT;
   private final double gravity;
   private final double mass;

   // parameters
   private final ParameterMap params;
   private final static String JOINT_DAMPING = "jointDamping";
   private final static String BODY_ORIENTATION_PROPORTIONAL_GAINS = "bodyOrientationProportionalGains";
   private final static String BODY_ORIENTATION_DERIVATIVE_GAINS = "bodyOrientationDerivativeGains";
   private final static String BODY_ORIENTATION_INTEGRAL_GAINS = "bodyOrientationIntegralGains";
   private final static String BODY_ORIENTATION_MAX_INTEGRAL_ERROR = "bodyOrientationMaxIntegralError";
   private final static String SWING_POSITION_PROPORTIONAL_GAINS = "swingPositionProportionalGains";
   private final static String SWING_POSITION_DERIVATIVE_GAINS = "swingPositionDerivativeGains";
   private final static String SWING_POSITION_INTEGRAL_GAINS = "swingPositionIntegralGains";
   private final static String SWING_POSITION_MAX_INTEGRAL_ERROR = "swingPositionMaxIntegralError";
   private final static String SWING_POSITION_GRAVITY_FEEDFORWARD_FORCE = "swingPositionGravityFeedforwardForce";
   private final static String SWING_TRAJECTORY_GROUND_CLEARANCE = "swingTrajectoryGroundClearance";
   private final static String DCM_PROPORTIONAL_GAINS = "dcmProportionalGains";
   private final static String DCM_INTEGRAL_GAINS = "dcmIntegralGains";
   private final static String DCM_MAX_INTEGRAL_ERROR = "dcmMaxIntegralError";
   private final static String COM_HEIGHT_PROPORTIONAL_GAIN = "comHeightProportionalGain";
   private final static String COM_HEIGHT_DERIVATIVE_GAIN = "comHeightDerivativeGain";
   private final static String COM_HEIGHT_INTEGRAL_GAIN = "comHeightIntegralGain";
   private final static String COM_HEIGHT_MAX_INTEGRAL_ERROR = "comHeightMaxIntegralError";
   private final static String COM_HEIGHT_GRAVITY_FEEDFORWARD_CONSTANT = "comHeightGravityFeedforwardConstant";
   private final static String COM_HEIGHT_NOMINAL = "comHeightNominal";
   private final static String QUAD_SUPPORT_DURATION = "quadSupportDuration";
   private final static String DOUBLE_SUPPORT_DURATION = "doubleSupportDuration";
   private final static String STANCE_WIDTH_NOMINAL = "stanceWidthNominal";
   private final static String STANCE_LENGTH_NOMINAL = "stanceLengthNominal";

   // utilities
   private final QuadrupedJointLimits jointLimits;
   private final QuadrupedContactForceLimits contactForceLimits;
   private final QuadrupedReferenceFrames referenceFrames;
   private final ReferenceFrame worldFrame;
   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame comFrame;
   private final PoseReferenceFrame supportFrame;
   private final QuadrantDependentList<ReferenceFrame> soleFrame;
   private final QuadrantDependentList<RigidBody> footRigidBody;
   private final OneDoFJoint[] oneDoFJoints;
   private final CenterOfMassJacobian comJacobian;
   private final TwistCalculator twistCalculator;
   private final QuadrantDependentList<ThreeDoFSwingFootTrajectory> swingFootTrajectory;
   private final QuadrantDependentList<ContactState> contactState;

   // controllers
   private final QuadrupedVirtualModelController virtualModelController;
   private final QuadrupedVirtualModelControllerSettings virtualModelControllerSettings;
   private final QuadrupedContactForceOptimization contactForceOptimization;
   private final QuadrupedContactForceOptimizationSettings contactForceOptimizationSettings;
   private final PIDController comHeightController;
   private final AxisAngleOrientationController bodyOrientationController;
   private final DivergentComponentOfMotionController dcmPositionController;
   private final QuadrantDependentList<EuclideanPositionController> swingPositionController;
   private final PiecewisePeriodicDcmTrajectory nominalPeriodicDcmTrajectory;

   // state machines
   public enum TrotState
   {
      QUAD_SUPPORT, HIND_LEFT_FRONT_RIGHT_SUPPORT, HIND_RIGHT_FRONT_LEFT_SUPPORT
   }

   public enum TrotEvent
   {
      TIMEOUT
   }

   private final StateMachine<TrotState, TrotEvent> trotStateMachine;

   // provider inputs
   private double bodyYawRateInput;
   private double bodyYawRateIntegral;
   private final FrameVector bodyVelocityInput;
   private final FrameOrientation bodyOrientationInput;
   private double comHeightInput;

   // setpoints
   private final QuadrantDependentList<FramePoint> solePositionSetpoint;
   private final QuadrantDependentList<FrameVector> soleLinearVelocitySetpoint;
   private final QuadrantDependentList<FrameVector> soleForceFeedforwardSetpoint;
   private final QuadrantDependentList<FrameVector> soleForceSetpoint;
   private final FrameOrientation bodyOrientationSetpoint;
   private final FrameVector bodyAngularVelocitySetpoint;
   private final FrameVector bodyTorqueFeedforwardSetpoint;
   private final FrameVector bodyTorqueSetpoint;
   private final FramePoint dcmPositionSetpoint;
   private final FrameVector dcmVelocitySetpoint;
   private final FramePoint icpPositionSetpoint;
   private final FrameVector icpVelocitySetpoint;
   private final FramePoint cmpPositionSetpoint;
   private final FramePoint vrpPositionSetpoint;
   private final FrameVector comForceSetpoint;
   private double comHeightSetpoint;

   // estimates
   private final QuadrantDependentList<FrameOrientation> soleOrientationEstimate;
   private final QuadrantDependentList<FramePoint> solePositionEstimate;
   private final QuadrantDependentList<FrameVector> soleAngularVelocityEstimate;
   private final QuadrantDependentList<FrameVector> soleLinearVelocityEstimate;
   private final QuadrupedSupportPolygon supportPolygonEstimate;
   private final FramePoint supportCentroidEstimate;
   private final FrameOrientation supportOrientationEstimate;
   private final FrameOrientation bodyOrientationEstimate;
   private final FramePoint bodyPositionEstimate;
   private final FrameVector bodyLinearVelocityEstimate;
   private final FrameVector bodyAngularVelocityEstimate;
   private final FramePoint comPositionEstimate;
   private final FrameVector comVelocityEstimate;
   private final FramePoint dcmPositionEstimate;
   private final FramePoint icpPositionEstimate;
   private double comHeightEstimate;

   // YoVariables
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoFrameVector yoBodyVelocityInput;
   private final DoubleYoVariable yoBodyYawRateInput;
   private final YoFrameOrientation yoBodyOrientationInput;
   private final DoubleYoVariable yoComHeightInput;
   private final QuadrantDependentList<YoFramePoint> yoSolePositionSetpoint;
   private final QuadrantDependentList<YoFrameVector> yoSoleLinearVelocitySetpoint;
   private final QuadrantDependentList<YoFrameVector> yoSoleForceFeedforwardSetpoint;
   private final QuadrantDependentList<YoFrameVector> yoSoleForceSetpoint;
   private final YoFrameOrientation yoBodyOrientationSetpoint;
   private final YoFrameVector yoBodyAngularVelocitySetpoint;
   private final YoFrameVector yoBodyTorqueFeedforwardSetpoint;
   private final YoFrameVector yoBodyTorqueSetpoint;
   private final YoFramePoint yoIcpPositionSetpoint;
   private final YoFramePoint yoCmpPositionSetpoint;
   private final YoFramePoint yoVrpPositionSetpoint;
   private final YoFrameVector yoComForceSetpoint;
   private final DoubleYoVariable yoComHeightSetpoint;
   private final QuadrantDependentList<YoFrameOrientation> yoSoleOrientationEstimate;
   private final QuadrantDependentList<YoFramePoint> yoSolePositionEstimate;
   private final QuadrantDependentList<YoFrameVector> yoSoleAngularVelocityEstimate;
   private final QuadrantDependentList<YoFrameVector> yoSoleLinearVelocityEstimate;
   private final YoFrameConvexPolygon2d yoSupportPolygonEstimate;
   private final YoFramePoint yoSupportCentroidEstimate;
   private final YoFrameOrientation yoSupportOrientationEstimate;
   private final YoFrameOrientation yoBodyOrientationEstimate;
   private final YoFramePoint yoBodyPositionEstimate;
   private final YoFrameVector yoBodyAngularVelocityEstimate;
   private final YoFrameVector yoBodyLinearVelocityEstimate;
   private final YoFramePoint yoComPositionEstimate;
   private final YoFrameVector yoComVelocityEstimate;
   private final YoFramePoint yoIcpPositionEstimate;
   private final DoubleYoVariable yoComHeightEstimate;

   // YoGraphics
   private final YoGraphicsList yoGraphicsList;
   private final ArtifactList artifactList;

   // temporary
   private final Twist twistStorage = new Twist();

   public QuadrupedVirtualModelBasedTrotController(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedRobotParameters robotParameters,
         ParameterMapRepository parameterMapRepository)
   {
      this.fullRobotModel = runtimeEnvironment.getFullRobotModel();
      this.robotTimestamp = runtimeEnvironment.getRobotTimestamp();
      this.yoGraphicsListRegistry = runtimeEnvironment.getGraphicsListRegistry();
      this.jointNameMap = robotParameters.getJointMap();
      this.controlDT = runtimeEnvironment.getControlDT();
      this.gravity = 9.81;
      this.mass = fullRobotModel.getTotalMass();

      // parameters
      this.params = parameterMapRepository.get(QuadrupedVirtualModelBasedTrotController.class);
      params.setDefault(JOINT_DAMPING, 1);
      params.setDefault(BODY_ORIENTATION_PROPORTIONAL_GAINS, 5000, 5000, 5000);
      params.setDefault(BODY_ORIENTATION_DERIVATIVE_GAINS, 750, 750, 750);
      params.setDefault(BODY_ORIENTATION_INTEGRAL_GAINS, 0, 0, 0);
      params.setDefault(BODY_ORIENTATION_MAX_INTEGRAL_ERROR, 0);
      params.setDefault(SWING_POSITION_PROPORTIONAL_GAINS, 100000, 100000, 100000);
      params.setDefault(SWING_POSITION_DERIVATIVE_GAINS, 500, 500, 500);
      params.setDefault(SWING_POSITION_INTEGRAL_GAINS, 0, 0, 0);
      params.setDefault(SWING_POSITION_MAX_INTEGRAL_ERROR, 0);
      params.setDefault(SWING_POSITION_GRAVITY_FEEDFORWARD_FORCE, 0);
      params.setDefault(SWING_TRAJECTORY_GROUND_CLEARANCE, 0.05);
      params.setDefault(DCM_PROPORTIONAL_GAINS, 2, 2, 0);
      params.setDefault(DCM_INTEGRAL_GAINS, 0, 0, 0);
      params.setDefault(DCM_MAX_INTEGRAL_ERROR, 0);
      params.setDefault(COM_HEIGHT_PROPORTIONAL_GAIN, 5000);
      params.setDefault(COM_HEIGHT_DERIVATIVE_GAIN, 750);
      params.setDefault(COM_HEIGHT_INTEGRAL_GAIN, 0);
      params.setDefault(COM_HEIGHT_MAX_INTEGRAL_ERROR, 0);
      params.setDefault(COM_HEIGHT_GRAVITY_FEEDFORWARD_CONSTANT, 0.95);
      params.setDefault(COM_HEIGHT_NOMINAL, 0.60);
      params.setDefault(QUAD_SUPPORT_DURATION, 1.00);
      params.setDefault(DOUBLE_SUPPORT_DURATION, 0.35);
      params.setDefault(STANCE_WIDTH_NOMINAL, 0.5);
      params.setDefault(STANCE_LENGTH_NOMINAL, 1.1);

      // utilities
      jointLimits = new QuadrupedJointLimits(robotParameters.getQuadrupedJointLimits());
      contactForceLimits = new QuadrupedContactForceLimits();
      referenceFrames = new QuadrupedReferenceFrames(fullRobotModel, jointNameMap, robotParameters.getPhysicalProperties());
      comFrame = referenceFrames.getCenterOfMassZUpFrame();
      bodyFrame = referenceFrames.getBodyFrame();
      worldFrame = referenceFrames.getWorldFrame();
      supportFrame = new PoseReferenceFrame("SupportFrame", referenceFrames.getWorldFrame());
      soleFrame = referenceFrames.getFootReferenceFrames();
      footRigidBody = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String jointBeforeFootName = robotParameters.getJointMap().getJointBeforeFootName(robotQuadrant);
         OneDoFJoint jointBeforeFoot = fullRobotModel.getOneDoFJointByName(jointBeforeFootName);
         footRigidBody.set(robotQuadrant, jointBeforeFoot.getSuccessor());
      }
      oneDoFJoints = fullRobotModel.getOneDoFJoints();
      comJacobian = new CenterOfMassJacobian(fullRobotModel.getElevator());
      twistCalculator = new TwistCalculator(worldFrame, fullRobotModel.getElevator());
      swingFootTrajectory = new QuadrantDependentList<>();
      contactState = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         swingFootTrajectory.set(robotQuadrant, new ThreeDoFSwingFootTrajectory());
         contactState.set(robotQuadrant, ContactState.IN_CONTACT);
      }
      nominalPeriodicDcmTrajectory = new PiecewisePeriodicDcmTrajectory(1, gravity, params.get(COM_HEIGHT_NOMINAL), null);

      // controllers
      virtualModelController = new QuadrupedVirtualModelController(fullRobotModel, referenceFrames, jointNameMap, registry, yoGraphicsListRegistry);
      virtualModelControllerSettings = new QuadrupedVirtualModelControllerSettings();
      contactForceOptimization = new QuadrupedContactForceOptimization(referenceFrames, registry);
      contactForceOptimizationSettings = new QuadrupedContactForceOptimizationSettings();
      comHeightController = new PIDController("bodyHeight", registry);
      bodyOrientationController = new AxisAngleOrientationController("bodyOrientation", bodyFrame, controlDT, registry);
      dcmPositionController = new DivergentComponentOfMotionController("dcm", comFrame, controlDT, mass, gravity, params.get(COM_HEIGHT_NOMINAL), registry);
      swingPositionController = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression() + "SwingState";
         swingPositionController.set(robotQuadrant, new EuclideanPositionController(prefix, soleFrame.get(robotQuadrant), controlDT, registry));
      }

      // state machines
      StateMachineBuilder<TrotState, TrotEvent> stateMachineBuilder = new StateMachineBuilder<>(TrotState.class, "TrotState", registry);
      stateMachineBuilder.addState(TrotState.QUAD_SUPPORT, new QuadSupportState());
      stateMachineBuilder.addState(TrotState.HIND_LEFT_FRONT_RIGHT_SUPPORT, new DoubleSupportState(RobotQuadrant.HIND_LEFT, RobotQuadrant.FRONT_RIGHT));
      stateMachineBuilder.addState(TrotState.HIND_RIGHT_FRONT_LEFT_SUPPORT, new DoubleSupportState(RobotQuadrant.HIND_RIGHT, RobotQuadrant.FRONT_LEFT));
      stateMachineBuilder.addTransition(TrotEvent.TIMEOUT, TrotState.QUAD_SUPPORT, TrotState.HIND_LEFT_FRONT_RIGHT_SUPPORT);
      stateMachineBuilder.addTransition(TrotEvent.TIMEOUT, TrotState.HIND_LEFT_FRONT_RIGHT_SUPPORT, TrotState.HIND_RIGHT_FRONT_LEFT_SUPPORT);
      stateMachineBuilder.addTransition(TrotEvent.TIMEOUT, TrotState.HIND_RIGHT_FRONT_LEFT_SUPPORT, TrotState.HIND_LEFT_FRONT_RIGHT_SUPPORT);
      trotStateMachine = stateMachineBuilder.build(TrotState.QUAD_SUPPORT);

      // provider inputs
      bodyYawRateInput = 0.0;
      bodyYawRateIntegral = 0.0;
      bodyVelocityInput = new FrameVector(worldFrame);
      bodyOrientationInput = new FrameOrientation(worldFrame);
      comHeightInput = params.get(COM_HEIGHT_NOMINAL);

      // setpoints
      solePositionSetpoint = new QuadrantDependentList<>();
      soleLinearVelocitySetpoint = new QuadrantDependentList<>();
      soleForceFeedforwardSetpoint = new QuadrantDependentList<>();
      soleForceSetpoint = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePositionSetpoint.set(robotQuadrant, new FramePoint(worldFrame));
         soleLinearVelocitySetpoint.set(robotQuadrant, new FrameVector(worldFrame));
         soleForceFeedforwardSetpoint.set(robotQuadrant, new FrameVector(worldFrame));
         soleForceSetpoint.set(robotQuadrant, new FrameVector(worldFrame));
      }
      bodyOrientationSetpoint = new FrameOrientation(worldFrame);
      bodyAngularVelocitySetpoint = new FrameVector(worldFrame);
      bodyTorqueFeedforwardSetpoint = new FrameVector(worldFrame);
      bodyTorqueSetpoint = new FrameVector(worldFrame);
      dcmPositionSetpoint = new FramePoint(worldFrame);
      dcmVelocitySetpoint = new FrameVector(worldFrame);
      icpPositionSetpoint = new FramePoint(worldFrame);
      icpVelocitySetpoint = new FrameVector(worldFrame);
      cmpPositionSetpoint = new FramePoint(worldFrame);
      vrpPositionSetpoint = new FramePoint(worldFrame);
      comForceSetpoint = new FrameVector(worldFrame);
      comHeightSetpoint = params.get(COM_HEIGHT_NOMINAL);

      // estimates
      soleOrientationEstimate = new QuadrantDependentList<>();
      solePositionEstimate = new QuadrantDependentList<>();
      soleAngularVelocityEstimate = new QuadrantDependentList<>();
      soleLinearVelocityEstimate = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         soleOrientationEstimate.set(robotQuadrant, new FrameOrientation(worldFrame));
         solePositionEstimate.set(robotQuadrant, new FramePoint(worldFrame));
         soleAngularVelocityEstimate.set(robotQuadrant, new FrameVector(worldFrame));
         soleLinearVelocityEstimate.set(robotQuadrant, new FrameVector(worldFrame));
      }
      supportPolygonEstimate = new QuadrupedSupportPolygon();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         supportPolygonEstimate.setFootstep(robotQuadrant, new FramePoint(worldFrame));
      }
      supportCentroidEstimate = new FramePoint(worldFrame);
      supportOrientationEstimate = new FrameOrientation(worldFrame);
      bodyOrientationEstimate = new FrameOrientation(worldFrame);
      bodyPositionEstimate = new FramePoint(worldFrame);
      bodyAngularVelocityEstimate = new FrameVector(worldFrame);
      bodyLinearVelocityEstimate = new FrameVector(worldFrame);
      comPositionEstimate = new FramePoint(worldFrame);
      comVelocityEstimate = new FrameVector(worldFrame);
      dcmPositionEstimate = new FramePoint(worldFrame);
      icpPositionEstimate = new FramePoint(worldFrame);
      comHeightEstimate = params.get(COM_HEIGHT_NOMINAL);

      // YoVariables
      yoBodyYawRateInput = new DoubleYoVariable("bodyYawRateInput", registry);
      yoBodyVelocityInput = new YoFrameVector("bodyVelocityInput", comFrame, registry);
      yoBodyOrientationInput = new YoFrameOrientation("bodyOrientationInput", worldFrame, registry);
      yoComHeightInput = new DoubleYoVariable("comHeightInput", registry);
      yoSolePositionSetpoint = new QuadrantDependentList<>();
      yoSoleLinearVelocitySetpoint = new QuadrantDependentList<>();
      yoSoleForceFeedforwardSetpoint = new QuadrantDependentList<>();
      yoSoleForceSetpoint = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression();
         yoSolePositionSetpoint.set(robotQuadrant, new YoFramePoint(prefix + "SolePositionSetpoint", worldFrame, registry));
         yoSoleLinearVelocitySetpoint.set(robotQuadrant, new YoFrameVector(prefix + "SoleLinearVelocitySetpoint", worldFrame, registry));
         yoSoleForceFeedforwardSetpoint.set(robotQuadrant, new YoFrameVector(prefix + "SoleForceFeedforwardSetpoint", worldFrame, registry));
         yoSoleForceSetpoint.set(robotQuadrant, new YoFrameVector(prefix + "SoleForceSetpoint", worldFrame, registry));
      }
      yoBodyOrientationSetpoint = new YoFrameOrientation("bodyOrientationSetpoint", worldFrame, registry);
      yoBodyAngularVelocitySetpoint = new YoFrameVector("bodyAngularVelocitySetpoint", worldFrame, registry);
      yoBodyTorqueFeedforwardSetpoint = new YoFrameVector("bodyTorqueFeedforwardSetpoint", worldFrame, registry);
      yoBodyTorqueSetpoint = new YoFrameVector("bodyTorqueSetpoint", worldFrame, registry);
      yoIcpPositionSetpoint = new YoFramePoint("icpPositionSetpoint", worldFrame, registry);
      yoCmpPositionSetpoint = new YoFramePoint("cmpPositionSetpoint", worldFrame, registry);
      yoVrpPositionSetpoint = new YoFramePoint("vrpPositionSetpoint", worldFrame, registry);
      yoComForceSetpoint = new YoFrameVector("comForceSetpoint", worldFrame, registry);
      yoComHeightSetpoint = new DoubleYoVariable("comHeightSetpoint", registry);
      yoSoleOrientationEstimate = new QuadrantDependentList<>();
      yoSolePositionEstimate = new QuadrantDependentList<>();
      yoSoleAngularVelocityEstimate = new QuadrantDependentList<>();
      yoSoleLinearVelocityEstimate = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression();
         yoSoleOrientationEstimate.set(robotQuadrant, new YoFrameOrientation(prefix + "SoleOrientationEstimate", worldFrame, registry));
         yoSolePositionEstimate.set(robotQuadrant, new YoFramePoint(prefix + "SolePositionEstimate", worldFrame, registry));
         yoSoleAngularVelocityEstimate.set(robotQuadrant, new YoFrameVector(prefix + "SoleAngularVelocityEstimate", worldFrame, registry));
         yoSoleLinearVelocityEstimate.set(robotQuadrant, new YoFrameVector(prefix + "SoleLinearVelocityEstimate", worldFrame, registry));
      }
      yoSupportPolygonEstimate = new YoFrameConvexPolygon2d("supportPolygon", "", worldFrame, 4, registry);
      yoSupportCentroidEstimate = new YoFramePoint("supportCentroidEstimate", worldFrame, registry);
      yoSupportOrientationEstimate = new YoFrameOrientation("supportOrientationEstimate", worldFrame, registry);
      yoBodyOrientationEstimate = new YoFrameOrientation("bodyOrientationEstimate", worldFrame, registry);
      yoBodyPositionEstimate = new YoFramePoint("bodyPositionEstimate", worldFrame, registry);
      yoBodyAngularVelocityEstimate = new YoFrameVector("bodyAngularVelocityEstimate", worldFrame, registry);
      yoBodyLinearVelocityEstimate = new YoFrameVector("bodyLinearVelocityEstimate", worldFrame, registry);
      yoComPositionEstimate = new YoFramePoint("comPositionEstimate", worldFrame, registry);
      yoComVelocityEstimate = new YoFrameVector("comVelocityEstimate", worldFrame, registry);
      yoIcpPositionEstimate = new YoFramePoint("icpPositionEstimate", worldFrame, registry);
      yoComHeightEstimate = new DoubleYoVariable("comHeightEstimate", registry);

      // YoGraphics
      yoGraphicsList = new YoGraphicsList(getClass().getSimpleName() + "Graphics");
      artifactList = new ArtifactList(getClass().getSimpleName() + "Artifacts");
      registerGraphics();

      runtimeEnvironment.getParentRegistry().addChild(registry);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public YoGraphicsList getYoGraphicsList()
   {
      return yoGraphicsList;
   }

   public ArtifactList getArtifactList()
   {
      return artifactList;
   }

   private void registerGraphics()
   {
      String prefix = getClass().getSimpleName();
      YoGraphicPosition yoComPositionEstimateViz = new YoGraphicPosition(prefix + "pcomPositionEstimate", yoComPositionEstimate, 0.025, YoAppearance.Black(),
            GraphicType.BALL_WITH_CROSS);
      YoGraphicPosition yoIcpPositionEstimateViz = new YoGraphicPosition(prefix + "icpPositionEstimate", yoIcpPositionEstimate, 0.025, YoAppearance.Magenta());
      YoGraphicPosition yoIcpPositionSetpointViz = new YoGraphicPosition(prefix + "icpPositionSetpoint", yoIcpPositionSetpoint, 0.025, YoAppearance.Blue());
      YoGraphicPosition yoCmpPositionSetpointViz = new YoGraphicPosition(prefix + "cmpPositionSetpoint", yoCmpPositionSetpoint, 0.025, YoAppearance.Chartreuse());
      yoGraphicsList.add(yoComPositionEstimateViz);
      yoGraphicsList.add(yoIcpPositionEstimateViz);
      yoGraphicsList.add(yoIcpPositionSetpointViz);
      yoGraphicsList.add(yoCmpPositionSetpointViz);
      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);

      YoArtifactPolygon yoSupportPolygonArtifact = new YoArtifactPolygon(prefix + "supportPolygon", yoSupportPolygonEstimate, Color.BLACK, false);
      artifactList.add(yoComPositionEstimateViz.createArtifact());
      artifactList.add(yoIcpPositionEstimateViz.createArtifact());
      artifactList.add(yoIcpPositionSetpointViz.createArtifact());
      artifactList.add(yoCmpPositionSetpointViz.createArtifact());
      artifactList.add(yoSupportPolygonArtifact);
      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   private void updateEstimates()
   {
      // update frames and twists
      referenceFrames.updateFrames();
      twistCalculator.compute();
      comJacobian.compute();

      // compute sole poses and twists
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         twistCalculator.getTwistOfBody(twistStorage, footRigidBody.get(robotQuadrant));
         twistStorage.changeFrame(soleFrame.get(robotQuadrant));
         twistStorage.getAngularPart(soleAngularVelocityEstimate.get(robotQuadrant));
         twistStorage.getLinearPart(soleLinearVelocityEstimate.get(robotQuadrant));
         soleOrientationEstimate.get(robotQuadrant).setToZero(soleFrame.get(robotQuadrant));
         solePositionEstimate.get(robotQuadrant).setToZero(soleFrame.get(robotQuadrant));
      }

      // compute support polygon
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePositionEstimate.get(robotQuadrant).changeFrame(supportPolygonEstimate.getReferenceFrame());
         supportPolygonEstimate.setFootstep(robotQuadrant, solePositionEstimate.get(robotQuadrant));
      }

      // compute support frame (centroid and nominal orientation)
      supportCentroidEstimate.changeFrame(supportPolygonEstimate.getReferenceFrame());
      supportOrientationEstimate.changeFrame(supportPolygonEstimate.getReferenceFrame());
      supportPolygonEstimate.getCentroid2d(supportCentroidEstimate);
      supportOrientationEstimate.setYawPitchRoll(supportPolygonEstimate.getNominalYaw(), 0, 0);
      supportFrame.setPoseAndUpdate(supportCentroidEstimate, supportOrientationEstimate);

      // compute body pose and twist
      twistCalculator.getTwistOfBody(twistStorage, fullRobotModel.getPelvis());
      twistStorage.changeFrame(bodyFrame);
      twistStorage.getAngularPart(bodyAngularVelocityEstimate);
      twistStorage.getLinearPart(bodyLinearVelocityEstimate);
      bodyOrientationEstimate.setToZero(bodyFrame);
      bodyPositionEstimate.setToZero(bodyFrame);

      // compute center of mass position and velocity
      comPositionEstimate.setToZero(comFrame);
      comJacobian.getCenterOfMassVelocity(comVelocityEstimate);

      // compute divergent component of motion and instantaneous capture point
      comPositionEstimate.changeFrame(worldFrame);
      comVelocityEstimate.changeFrame(worldFrame);
      dcmPositionEstimate.changeFrame(worldFrame);
      double omega = dcmPositionController.getNaturalFrequency();
      dcmPositionEstimate.setX(comPositionEstimate.getX() + comVelocityEstimate.getX() / omega);
      dcmPositionEstimate.setY(comPositionEstimate.getY() + comVelocityEstimate.getY() / omega);
      dcmPositionEstimate.setZ(comPositionEstimate.getZ() + comVelocityEstimate.getZ() / omega);
      icpPositionEstimate.setIncludingFrame(dcmPositionEstimate);
      icpPositionEstimate.add(0, 0, -dcmPositionController.getComHeight());

      // compute center of mass height
      comPositionEstimate.changeFrame(worldFrame);
      supportCentroidEstimate.changeFrame(worldFrame);
      comHeightEstimate = comPositionEstimate.getZ() - supportPolygonEstimate.getLowestFootstepZHeight();
   }

   private void updateSetpoints()
   {
      double currentTime = robotTimestamp.getDoubleValue();

      // compute com forces and swing foot forces
      trotStateMachine.process();

      // compute body torques
      bodyYawRateIntegral += bodyYawRateInput * controlDT;
      bodyOrientationSetpoint.changeFrame(bodyOrientationInput.getReferenceFrame());
      bodyOrientationSetpoint.setYawPitchRoll(bodyYawRateIntegral, bodyOrientationInput.getPitch(), bodyOrientationInput.getRoll());
      bodyOrientationSetpoint.changeFrame(bodyFrame);
      bodyAngularVelocitySetpoint.setToZero(bodyFrame);
      bodyAngularVelocityEstimate.changeFrame(bodyFrame);
      bodyTorqueFeedforwardSetpoint.setToZero(bodyFrame);
      bodyTorqueSetpoint.changeFrame(bodyFrame);
      bodyOrientationController
            .compute(bodyTorqueSetpoint, bodyOrientationSetpoint, bodyAngularVelocitySetpoint, bodyAngularVelocityEstimate, bodyTorqueFeedforwardSetpoint);

      // compute optimal contact forces
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactForceOptimization.setContactState(robotQuadrant, contactState.get(robotQuadrant));
      }
      contactForceOptimization.setComForceCommand(comForceSetpoint);
      contactForceOptimization.setComTorqueCommand(bodyTorqueSetpoint);
      contactForceOptimization.solve(contactForceLimits, contactForceOptimizationSettings);

      // compute leg joint torques using virtual model control
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (contactState.get(robotQuadrant) == ContactState.IN_CONTACT)
         {
            contactForceOptimization.getContactForceSolution(robotQuadrant, soleForceSetpoint.get(robotQuadrant));
            virtualModelController.setSoleContactForce(robotQuadrant, soleForceSetpoint.get(robotQuadrant));
            virtualModelController.setSoleContactForceVisible(robotQuadrant, true);
            virtualModelController.setSoleVirtualForceVisible(robotQuadrant, false);
         }
         else
         {
            virtualModelController.setSoleVirtualForce(robotQuadrant, soleForceSetpoint.get(robotQuadrant));
            virtualModelController.setSoleContactForceVisible(robotQuadrant, false);
            virtualModelController.setSoleVirtualForceVisible(robotQuadrant, true);
         }
      }
      virtualModelController.compute(jointLimits, virtualModelControllerSettings);
   }

   private void readYoVariables()
   {
      bodyYawRateInput = yoBodyYawRateInput.getDoubleValue();
      yoBodyVelocityInput.getFrameTupleIncludingFrame(bodyVelocityInput);
      yoBodyOrientationInput.getFrameOrientationIncludingFrame(bodyOrientationInput);
      comHeightInput = yoComHeightInput.getDoubleValue();
   }

   private void writeYoVariables()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         yoSolePositionSetpoint.get(robotQuadrant).setAndMatchFrame(solePositionSetpoint.get(robotQuadrant));
         yoSoleLinearVelocitySetpoint.get(robotQuadrant).setAndMatchFrame(soleLinearVelocitySetpoint.get(robotQuadrant));
         yoSoleForceFeedforwardSetpoint.get(robotQuadrant).setAndMatchFrame(soleForceFeedforwardSetpoint.get(robotQuadrant));
         yoSoleForceSetpoint.get(robotQuadrant).setAndMatchFrame(soleForceSetpoint.get(robotQuadrant));
      }
      yoBodyOrientationSetpoint.setAndMatchFrame(bodyOrientationSetpoint);
      yoBodyAngularVelocitySetpoint.setAndMatchFrame(bodyAngularVelocitySetpoint);
      yoBodyTorqueFeedforwardSetpoint.setAndMatchFrame(bodyTorqueFeedforwardSetpoint);
      yoBodyTorqueSetpoint.setAndMatchFrame(bodyTorqueSetpoint);
      yoIcpPositionSetpoint.setAndMatchFrame(icpPositionSetpoint);
      yoCmpPositionSetpoint.setAndMatchFrame(cmpPositionSetpoint);
      yoVrpPositionSetpoint.setAndMatchFrame(vrpPositionSetpoint);
      yoComForceSetpoint.setAndMatchFrame(comForceSetpoint);
      yoComHeightSetpoint.set(comHeightSetpoint);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         yoSoleOrientationEstimate.get(robotQuadrant).setAndMatchFrame(soleOrientationEstimate.get(robotQuadrant));
         yoSolePositionEstimate.get(robotQuadrant).setAndMatchFrame(solePositionEstimate.get(robotQuadrant));
         yoSoleAngularVelocityEstimate.get(robotQuadrant).setAndMatchFrame(soleAngularVelocityEstimate.get(robotQuadrant));
         yoSoleLinearVelocityEstimate.get(robotQuadrant).setAndMatchFrame(soleLinearVelocityEstimate.get(robotQuadrant));
      }
      supportPolygonEstimate.packYoFrameConvexPolygon2d(yoSupportPolygonEstimate);
      yoSupportCentroidEstimate.setAndMatchFrame(supportCentroidEstimate);
      yoSupportOrientationEstimate.setAndMatchFrame(supportOrientationEstimate);
      yoBodyOrientationEstimate.setAndMatchFrame(bodyOrientationEstimate);
      yoBodyAngularVelocityEstimate.setAndMatchFrame(bodyAngularVelocityEstimate);
      yoBodyLinearVelocityEstimate.setAndMatchFrame(bodyLinearVelocityEstimate);
      yoComPositionEstimate.setAndMatchFrame(comPositionEstimate);
      yoComVelocityEstimate.setAndMatchFrame(comVelocityEstimate);
      yoIcpPositionEstimate.setAndMatchFrame(icpPositionEstimate);
      yoComHeightEstimate.set(comHeightEstimate);
   }

   @Override public QuadrupedForceControllerEvent process()
   {
      readYoVariables();
      updateEstimates();
      updateSetpoints();
      writeYoVariables();
      return null;
   }

   @Override public void onEntry()
   {
      // show graphics
      yoGraphicsListRegistry.hideYoGraphics();
      yoGraphicsListRegistry.hideArtifacts();
      yoGraphicsList.setVisible(true);
      artifactList.setVisible(true);
      virtualModelController.setVisible(false);

      // initialize desired values (provider inputs)
      yoBodyYawRateInput.set(0.0);
      yoBodyVelocityInput.set(0.0, 0.0, 0.0);
      yoBodyOrientationInput.setYawPitchRoll(0.0, 0.0, 0.0);
      yoComHeightInput.set(params.get(COM_HEIGHT_NOMINAL));

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         for (int i = 0; i < jointNameMap.getLegJointNames().length; i++)
         {
            // initialize leg joint mode to force control
            LegJointName legJointName = jointNameMap.getLegJointNames()[i];
            String jointName = jointNameMap.getLegJointName(robotQuadrant, legJointName);
            OneDoFJoint joint = fullRobotModel.getOneDoFJointByName(jointName);
            joint.setUnderPositionControl(false);
         }
      }

      // initialize controllers and state machines
      virtualModelController.reset();
      for (OneDoFJoint joint : oneDoFJoints)
      {
         QuadrupedJointName jointName = jointNameMap.getJointNameForSDFName(joint.getName());
         virtualModelControllerSettings.setJointDamping(jointName, params.get(JOINT_DAMPING));
      }
      contactForceOptimization.reset();
      contactForceOptimizationSettings.setDefaults();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactForceOptimization.setContactState(robotQuadrant, ContactState.IN_CONTACT);
         contactForceOptimizationSettings.setContactForceCommandWeights(robotQuadrant, 0.0, 0.0, 0.0);
      }
      contactForceOptimizationSettings.setComForceCommandWeights(1.0, 1.0, 1.0);
      contactForceOptimizationSettings.setComTorqueCommandWeights(1.0, 1.0, 1.0);
      bodyOrientationController.reset();
      bodyOrientationController.setProportionalGains(params.getVolatileArray(BODY_ORIENTATION_PROPORTIONAL_GAINS));
      bodyOrientationController.setIntegralGains(params.getVolatileArray(BODY_ORIENTATION_INTEGRAL_GAINS), params.get(BODY_ORIENTATION_MAX_INTEGRAL_ERROR));
      bodyOrientationController.setDerivativeGains(params.getVolatileArray(BODY_ORIENTATION_DERIVATIVE_GAINS));
      comHeightController.resetIntegrator();
      comHeightController.setProportionalGain(params.get(COM_HEIGHT_PROPORTIONAL_GAIN));
      comHeightController.setIntegralGain(params.get(COM_HEIGHT_INTEGRAL_GAIN));
      comHeightController.setMaxIntegralError(params.get(COM_HEIGHT_MAX_INTEGRAL_ERROR));
      comHeightController.setDerivativeGain(params.get(COM_HEIGHT_DERIVATIVE_GAIN));
      dcmPositionController.reset();
      dcmPositionController.setProportionalGains(params.getVolatileArray(DCM_PROPORTIONAL_GAINS));
      dcmPositionController.setIntegralGains(params.getVolatileArray(DCM_INTEGRAL_GAINS), params.get(DCM_MAX_INTEGRAL_ERROR));
      trotStateMachine.reset();

      // initialize setpoints
      updateEstimates();
      dcmPositionSetpoint.setIncludingFrame(dcmPositionEstimate);
      dcmVelocitySetpoint.setToZero();
      bodyYawRateIntegral = bodyOrientationEstimate.getYaw();
   }

   @Override public void onExit()
   {
      // hide graphics
      yoGraphicsList.setVisible(false);
      artifactList.setVisible(false);
      virtualModelController.setVisible(false);
   }

   private void computeNominalCmpPositionAtSoS(RobotQuadrant hindSupportQuadrant, RobotQuadrant frontSupportQuadrant, FramePoint nominalCmpPositionAtSoS)
   {
      solePositionEstimate.get(hindSupportQuadrant).changeFrame(worldFrame);
      solePositionEstimate.get(frontSupportQuadrant).changeFrame(worldFrame);
      nominalCmpPositionAtSoS.setToZero(worldFrame);
      nominalCmpPositionAtSoS.add(solePositionEstimate.get(hindSupportQuadrant));
      nominalCmpPositionAtSoS.add(solePositionEstimate.get(frontSupportQuadrant));
      nominalCmpPositionAtSoS.scale(0.5);
   }

   private void computeNominalCmpPositionAtEoS(FramePoint nominalCmpPositionAtSoS, FramePoint nominalCmpPositionAtEoS)
   {
      double aStride = bodyYawRateIntegral + bodyYawRateInput * params.get(DOUBLE_SUPPORT_DURATION);
      double xStride = bodyVelocityInput.getX() * params.get(DOUBLE_SUPPORT_DURATION);
      double yStride = bodyVelocityInput.getY() * params.get(DOUBLE_SUPPORT_DURATION);
      double xOffset = Math.cos(aStride) * xStride - Math.sin(aStride) * yStride;
      double yOffset = Math.sin(aStride) * xStride + Math.cos(aStride) * yStride;
      nominalCmpPositionAtEoS.setIncludingFrame(nominalCmpPositionAtSoS);
      nominalCmpPositionAtEoS.changeFrame(worldFrame);
      nominalCmpPositionAtEoS.add(xOffset, yOffset, 0.0);
   }

   private void computeNominalPeriodicDcmTrajectory(FramePoint nominalCmpPositionAtSoS, FramePoint nominalCmpPositionAtEoS)
   {
      double timeAtEoS = params.get(DOUBLE_SUPPORT_DURATION);
      double relativeYawAtEoS = bodyYawRateInput * timeAtEoS;
      nominalPeriodicDcmTrajectory.setComHeight(Math.max(comHeightInput, 0.001));
      nominalPeriodicDcmTrajectory.initializeTrajectory(0.0, nominalCmpPositionAtSoS, timeAtEoS, nominalCmpPositionAtEoS, relativeYawAtEoS);
   }

   private void computeNominalDcmPositionAtSoS(FramePoint nominalCmpPositionAtSoS, FramePoint nominalCmpPositionAtEoS, FramePoint nominalDcmPositionAtSoS)
   {
      computeNominalPeriodicDcmTrajectory(nominalCmpPositionAtSoS, nominalCmpPositionAtEoS);
      nominalPeriodicDcmTrajectory.computeTrajectory(0.0);
      nominalPeriodicDcmTrajectory.getPosition(nominalDcmPositionAtSoS);
   }

   private void computeNominalDcmPositionAtEoS(FramePoint nominalCmpAtSoS, FramePoint nominalCmpAtEoS, FramePoint nominalDcmPositionAtEoS)
   {
      computeNominalPeriodicDcmTrajectory(nominalCmpAtSoS, nominalCmpAtEoS);
      double timeAtEoS = params.get(DOUBLE_SUPPORT_DURATION);
      nominalPeriodicDcmTrajectory.computeTrajectory(timeAtEoS);
      nominalPeriodicDcmTrajectory.getPosition(nominalDcmPositionAtEoS);
   }

   private void computeFootholdPosition(RobotQuadrant robotQuadrant, FramePoint cmpPosition, double bodyYaw, FramePoint footholdPosition)
   {
      double xStance = robotQuadrant.getEnd().negateIfHindEnd(params.get(STANCE_LENGTH_NOMINAL) / 2);
      double yStance = robotQuadrant.getSide().negateIfRightSide(params.get(STANCE_WIDTH_NOMINAL) / 2);
      double xOffset = Math.cos(bodyYaw) * xStance - Math.sin(bodyYaw) * yStance;
      double yOffset = Math.sin(bodyYaw) * xStance + Math.cos(bodyYaw) * yStance;
      footholdPosition.setIncludingFrame(cmpPosition);
      footholdPosition.changeFrame(worldFrame);
      footholdPosition.add(xOffset, yOffset, 0.0);
      footholdPosition.setZ(solePositionEstimate.get(robotQuadrant).getZ());
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
         dcmTrajectory.initializeTrajectory(dcmPositionEstimate, dcmPositionAtSoSNominal, params.get(QUAD_SUPPORT_DURATION));

         // initialize contact state
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            contactState.set(robotQuadrant, ContactState.IN_CONTACT);
         }
      }

      @Override public TrotEvent process()
      {
         double currentTime = robotTimestamp.getDoubleValue();

         // compute dcm setpoint
         dcmTrajectory.computeTrajectory(currentTime - initialTime);
         dcmTrajectory.getPosition(dcmPositionSetpoint);
         dcmTrajectory.getVelocity(dcmVelocitySetpoint);
         icpPositionSetpoint.setIncludingFrame(dcmPositionSetpoint);
         icpPositionSetpoint.sub(0, 0, dcmPositionController.getComHeight());
         icpVelocitySetpoint.setIncludingFrame(dcmVelocitySetpoint);

         // compute horizontal forces to track desired dcm trajectory
         dcmPositionController.setComHeight(Math.max(comHeightSetpoint, 0.001));
         dcmPositionController.compute(comForceSetpoint, vrpPositionSetpoint, cmpPositionSetpoint, dcmPositionSetpoint, dcmVelocitySetpoint, dcmPositionEstimate);

         // compute vertical com force
         comHeightSetpoint = comHeightInput;
         double comForceZ = params.get(COM_HEIGHT_GRAVITY_FEEDFORWARD_CONSTANT) * mass * gravity + comHeightController.compute(comHeightEstimate, comHeightSetpoint, comVelocityEstimate.getZ(), 0, controlDT);
         comForceSetpoint.changeFrame(worldFrame);
         comForceSetpoint.setZ(comForceZ);

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
         dcmTrajectory = new PiecewiseForwardDcmTrajectory(1, gravity, params.get(COM_HEIGHT_NOMINAL), null);
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
         dcmTrajectory.initializeTrajectory(initialTime, cmpPositionAtSoSNominal, dcmPositionEstimate);
         dcmTrajectory.computeTrajectory(initialTime + params.get(DOUBLE_SUPPORT_DURATION));
         dcmTrajectory.getPosition(dcmPositionAtEoS);

         // compute desired cmp position at end of step
         cmpPositionAtEoS.setIncludingFrame(dcmPositionAtEoS);
         cmpPositionAtEoS.sub(dcmPositionAtEoSNominal);
         cmpPositionAtEoS.add(cmpPositionAtEoSNominal);
         cmpPositionAtEoS.setZ(cmpPositionAtSoSNominal.getZ());

         // compute desired body yaw at end of step
         double bodyYawAtEoS = bodyYawRateIntegral + bodyYawRateInput * params.get(DOUBLE_SUPPORT_DURATION);

         for (int i = 0; i < 2; i++)
         {
            RobotQuadrant swingQuadrant = swingQuadrants[i];
            RobotQuadrant supportQuadrant = supportQuadrants[i];

            // compute foothold position to track the periodic dcm trajectory using deadbeat control
            computeFootholdPosition(swingQuadrant, cmpPositionAtEoS, bodyYawAtEoS, footholdPosition);

            // initialize swing foot trajectory
            FramePoint solePosition = solePositionEstimate.get(swingQuadrant);
            solePosition.changeFrame(footholdPosition.getReferenceFrame());
            swingFootTrajectory.get(swingQuadrant).initializeTrajectory(solePosition, footholdPosition,
               params.get(SWING_TRAJECTORY_GROUND_CLEARANCE), params.get(DOUBLE_SUPPORT_DURATION));
            swingPositionController.get(swingQuadrant).reset();
            swingPositionController.get(swingQuadrant).setProportionalGains(params.getVolatileArray(SWING_POSITION_PROPORTIONAL_GAINS));
            swingPositionController.get(swingQuadrant).setIntegralGains(params.getVolatileArray(SWING_POSITION_INTEGRAL_GAINS), params.get(SWING_POSITION_MAX_INTEGRAL_ERROR));
            swingPositionController.get(swingQuadrant).setDerivativeGains(params.getVolatileArray(SWING_POSITION_DERIVATIVE_GAINS));

            // initialize contact state
            contactState.set(swingQuadrant, ContactState.NO_CONTACT);
            contactState.set(supportQuadrant, ContactState.IN_CONTACT);
         }
      }

      @Override public TrotEvent process()
      {
         double currentTime = robotTimestamp.getDoubleValue();

         // compute dcm setpoint
         dcmTrajectory.computeTrajectory(currentTime);
         dcmTrajectory.getPosition(dcmPositionSetpoint);
         dcmTrajectory.getVelocity(dcmVelocitySetpoint);
         icpPositionSetpoint.setIncludingFrame(dcmPositionSetpoint);
         icpPositionSetpoint.sub(0, 0, dcmPositionController.getComHeight());
         icpVelocitySetpoint.setIncludingFrame(dcmVelocitySetpoint);

         // compute horizontal forces to track desired divergent component of motion
         dcmPositionController.setComHeight(Math.max(comHeightSetpoint, 0.001));
         dcmPositionController.compute(comForceSetpoint, vrpPositionSetpoint, cmpPositionSetpoint, dcmPositionSetpoint, dcmVelocitySetpoint, dcmPositionEstimate);

         // compute vertical com force
         comHeightSetpoint = comHeightInput;
         double comForceZ = params.get(COM_HEIGHT_GRAVITY_FEEDFORWARD_CONSTANT) * mass * gravity + comHeightController.compute(comHeightEstimate, comHeightSetpoint, comVelocityEstimate.getZ(), 0, controlDT);
         comForceSetpoint.changeFrame(worldFrame);
         comForceSetpoint.setZ(comForceZ);

         for (int i = 0; i < 2; i++)
         {
            RobotQuadrant swingQuadrant = swingQuadrants[i];

            // compute nominal sole position setpoint
            swingFootTrajectory.get(swingQuadrant).computeTrajectory(currentTime - initialTime);
            swingFootTrajectory.get(swingQuadrant).getPosition(solePositionSetpoint.get(swingQuadrant));

            // shift the swing foot trajectory in the direction of the dcm tracking error
            dcmPositionEstimate.changeFrame(solePositionSetpoint.get(swingQuadrant).getReferenceFrame());
            dcmPositionSetpoint.changeFrame(solePositionSetpoint.get(swingQuadrant).getReferenceFrame());
            solePositionSetpoint.get(swingQuadrant).add(dcmPositionEstimate.getX(), dcmPositionEstimate.getY(), 0.0);
            solePositionSetpoint.get(swingQuadrant).sub(dcmPositionSetpoint.getX(), dcmPositionSetpoint.getY(), 0.0);

            // compute sole force setpoint
            soleForceSetpoint.get(swingQuadrant).setToZero();
            soleForceSetpoint.get(swingQuadrant).changeFrame(soleFrame.get(swingQuadrant));
            solePositionSetpoint.get(swingQuadrant).changeFrame(soleFrame.get(swingQuadrant));
            soleLinearVelocitySetpoint.get(swingQuadrant).setToZero(soleFrame.get(swingQuadrant));
            soleLinearVelocityEstimate.get(swingQuadrant).changeFrame(soleFrame.get(swingQuadrant));
            soleForceFeedforwardSetpoint.get(swingQuadrant).changeFrame(worldFrame);
            soleForceFeedforwardSetpoint.get(swingQuadrant).set(0, 0, params.get(SWING_POSITION_GRAVITY_FEEDFORWARD_FORCE));
            soleForceFeedforwardSetpoint.get(swingQuadrant).changeFrame(soleFrame.get(swingQuadrant));
            swingPositionController.get(swingQuadrant)
                  .compute(soleForceSetpoint.get(swingQuadrant), solePositionSetpoint.get(swingQuadrant), soleLinearVelocitySetpoint.get(swingQuadrant),
                        soleLinearVelocityEstimate.get(swingQuadrant), soleForceFeedforwardSetpoint.get(swingQuadrant));
         }

         // trigger touch down event
         if (currentTime > initialTime + params.get(DOUBLE_SUPPORT_DURATION))
            return TrotEvent.TIMEOUT;
         else
            return null;
      }

      @Override public void onExit()
      {
         // change setpoints to world frame when not in use
         for (int i = 0; i < 2; i++)
         {
            RobotQuadrant swingQuadrant = swingQuadrants[i];
            soleForceSetpoint.get(swingQuadrant).changeFrame(worldFrame);
            solePositionSetpoint.get(swingQuadrant).changeFrame(worldFrame);
            soleLinearVelocitySetpoint.get(swingQuadrant).changeFrame(worldFrame);
            soleForceFeedforwardSetpoint.get(swingQuadrant).changeFrame(worldFrame);
         }
      }
   }
}
