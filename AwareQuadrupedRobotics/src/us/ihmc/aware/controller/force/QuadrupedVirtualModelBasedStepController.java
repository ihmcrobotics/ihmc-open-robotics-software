package us.ihmc.aware.controller.force;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.aware.controller.common.*;
import us.ihmc.aware.params.ParameterMap;
import us.ihmc.aware.params.ParameterMapRepository;
import us.ihmc.aware.planning.SingleStepDCMTrajectory;
import us.ihmc.aware.util.QuadrupedTimedStep;
import us.ihmc.aware.planning.ThreeDoFSwingFootTrajectory;
import us.ihmc.aware.state.StateMachine;
import us.ihmc.aware.state.StateMachineBuilder;
import us.ihmc.aware.state.StateMachineState;
import us.ihmc.aware.util.ContactState;
import us.ihmc.aware.util.TimeInterval;
import us.ihmc.aware.vmc.*;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.aware.parameters.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointNameMap;
import us.ihmc.quadrupedRobotics.virtualModelController.QuadrupedJointLimits;
import us.ihmc.aware.util.PreallocatedQueue;
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

public class QuadrupedVirtualModelBasedStepController implements QuadrupedForceController
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
   private final CenterOfMassJacobian comJacobian;
   private final TwistCalculator twistCalculator;
   private final QuadrantDependentList<ThreeDoFSwingFootTrajectory> swingFootTrajectory;
   private final SingleStepDCMTrajectory dcmTrajectory;

   // controllers
   private final QuadrupedVirtualModelController virtualModelController;
   private final QuadrupedVirtualModelControllerSettings virtualModelControllerSettings;
   private final QuadrupedContactForceOptimization contactForceOptimization;
   private final QuadrupedContactForceOptimizationSettings contactForceOptimizationSettings;
   private final PIDController comHeightController;
   private final AxisAngleOrientationController bodyOrientationController;
   private final DivergentComponentOfMotionController dcmPositionController;
   private final QuadrantDependentList<EuclideanPositionController> swingPositionController;

   // state machines
   public enum FootState
   {
      SUPPORT, TRANSFER, SWING
   }
   public enum FootEvent
   {
      TRANSFER, LIFT_OFF, TOUCH_DOWN
   }
   private final QuadrantDependentList<StateMachine<FootState, FootEvent>> footStateMachine;

   // provider inputs
   private static int STEP_QUEUE_CAPACITY = 30;
   private final PreallocatedQueue<QuadrupedTimedStep> stepQueue;
   private final QuadrantDependentList<QuadrupedTimedStep> stepCache;
   private final FrameOrientation bodyOrientationDesired;
   private double comHeightDesired;

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
   private final YoFrameOrientation yoBodyOrientationDesired;
   private final DoubleYoVariable yoComHeightDesired;
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

   public QuadrupedVirtualModelBasedStepController(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedRobotParameters robotParameters, ParameterMapRepository parameterMapRepository)
   {
      this.fullRobotModel = runtimeEnvironment.getFullRobotModel();
      this.robotTimestamp = runtimeEnvironment.getRobotTimestamp();
      this.yoGraphicsListRegistry = runtimeEnvironment.getGraphicsListRegistry();
      this.jointNameMap = robotParameters.getJointMap();
      this.controlDT = runtimeEnvironment.getControlDT();
      this.gravity = 9.81;
      this.mass = fullRobotModel.getTotalMass();

      // parameters
      this.params = parameterMapRepository.get(QuadrupedVirtualModelBasedStepController.class);
      params.setDefault(BODY_ORIENTATION_PROPORTIONAL_GAINS, 5000, 5000, 2500);
      params.setDefault(BODY_ORIENTATION_DERIVATIVE_GAINS, 750, 750, 500);
      params.setDefault(BODY_ORIENTATION_INTEGRAL_GAINS, 0, 0, 0);
      params.setDefault(BODY_ORIENTATION_MAX_INTEGRAL_ERROR, 0);
      params.setDefault(SWING_POSITION_PROPORTIONAL_GAINS, 4000, 4000, 4000);
      params.setDefault(SWING_POSITION_DERIVATIVE_GAINS, 200, 200, 200);
      params.setDefault(SWING_POSITION_INTEGRAL_GAINS, 0, 0, 0);
      params.setDefault(SWING_POSITION_MAX_INTEGRAL_ERROR, 0);
      params.setDefault(SWING_POSITION_GRAVITY_FEEDFORWARD_FORCE, 10);
      params.setDefault(SWING_TRAJECTORY_GROUND_CLEARANCE, 0.1);
      params.setDefault(DCM_PROPORTIONAL_GAINS, 2, 2, 0);
      params.setDefault(DCM_INTEGRAL_GAINS, 0, 0, 0);
      params.setDefault(DCM_MAX_INTEGRAL_ERROR, 0);
      params.setDefault(COM_HEIGHT_PROPORTIONAL_GAIN, 5000);
      params.setDefault(COM_HEIGHT_DERIVATIVE_GAIN, 750);
      params.setDefault(COM_HEIGHT_INTEGRAL_GAIN, 0);
      params.setDefault(COM_HEIGHT_MAX_INTEGRAL_ERROR, 0);
      params.setDefault(COM_HEIGHT_GRAVITY_FEEDFORWARD_CONSTANT, 0.95);
      params.setDefault(COM_HEIGHT_NOMINAL, 0.55);

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
      comJacobian = new CenterOfMassJacobian(fullRobotModel.getElevator());
      twistCalculator = new TwistCalculator(worldFrame, fullRobotModel.getElevator());
      swingFootTrajectory = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         swingFootTrajectory.set(robotQuadrant, new ThreeDoFSwingFootTrajectory());
      }
      dcmTrajectory = new SingleStepDCMTrajectory(gravity, params.get(COM_HEIGHT_NOMINAL), registry);

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
      footStateMachine = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression();
         StateMachineBuilder<FootState, FootEvent> stateMachineBuilder = new StateMachineBuilder<>(FootState.class, prefix + "FootState", registry);
         stateMachineBuilder.addState(FootState.SUPPORT, new SupportState(robotQuadrant));
         stateMachineBuilder.addState(FootState.TRANSFER, new TransferState(robotQuadrant));
         stateMachineBuilder.addState(FootState.SWING, new SwingState(robotQuadrant));
         stateMachineBuilder.addTransition(FootEvent.TRANSFER, FootState.SUPPORT, FootState.TRANSFER);
         stateMachineBuilder.addTransition(FootEvent.LIFT_OFF, FootState.TRANSFER, FootState.SWING);
         stateMachineBuilder.addTransition(FootEvent.TOUCH_DOWN, FootState.SWING, FootState.SUPPORT);
         footStateMachine.set(robotQuadrant, stateMachineBuilder.build(FootState.SUPPORT));
      }

      // provider inputs
      stepQueue = new PreallocatedQueue<>(QuadrupedTimedStep.class, STEP_QUEUE_CAPACITY);
      stepCache = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         stepCache.set(robotQuadrant, new QuadrupedTimedStep(robotQuadrant));
      }
      bodyOrientationDesired = new FrameOrientation(worldFrame);
      comHeightDesired = params.get(COM_HEIGHT_NOMINAL);

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
      yoBodyOrientationDesired = new YoFrameOrientation("bodyOrientationDesired", supportFrame, registry);
      yoComHeightDesired = new DoubleYoVariable("comHeightDesired", registry);
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

   public boolean addStep(QuadrupedTimedStep quadrupedTimedStep)
   {
      if ((quadrupedTimedStep.getTimeInterval().getStartTime() > robotTimestamp.getDoubleValue()) && stepQueue.enqueue())
      {
         stepQueue.getTail().set(quadrupedTimedStep);
         return true;
      }
      return false;
   }

   public void removeSteps()
   {
      while(stepQueue.dequeue())
      {
      }
   }

   public int getStepQueueSize()
   {
      return stepQueue.size();
   }


   private void registerGraphics()
   {
      String prefix = getClass().getSimpleName();
      YoGraphicPosition yoComPositionEstimateViz = new YoGraphicPosition(prefix + "pcomPositionEstimate", yoComPositionEstimate, 0.025, YoAppearance.Black(), GraphicType.BALL_WITH_CROSS);
      YoGraphicPosition yoIcpPositionEstimateViz = new YoGraphicPosition(prefix + "icpPositionEstimate", yoIcpPositionEstimate, 0.025, YoAppearance.Chartreuse());
      YoGraphicPosition yoIcpPositionSetpointViz = new YoGraphicPosition(prefix + "icpPositionSetpoint", yoIcpPositionSetpoint, 0.025, YoAppearance.Blue());
      YoGraphicPosition yoCmpPositionSetpointViz = new YoGraphicPosition(prefix + "cmpPositionSetpoint", yoCmpPositionSetpoint, 0.025, YoAppearance.Magenta());
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

   private void handleStepEvents()
   {
      if (stepQueue.size() > 0)
      {
         boolean inTransfer = false;
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            if (footStateMachine.get(robotQuadrant).getState() == FootState.TRANSFER)
               inTransfer = true;
         }
         QuadrupedTimedStep step = stepQueue.getHead();
         RobotQuadrant robotQuadrant = step.getRobotQuadrant();
         double currentTime = robotTimestamp.getDoubleValue();
         double transferTime = step.getTimeInterval().getStartTime() - currentTime;
         if (!inTransfer && transferTime < 1.5)
         {
            stepCache.get(robotQuadrant).set(step);
            stepQueue.dequeue();
            footStateMachine.get(robotQuadrant).trigger(FootEvent.TRANSFER);
            dcmTrajectory.setComHeight(dcmPositionController.getComHeight());
            dcmTrajectory.setStepPameters(currentTime, dcmPositionSetpoint, dcmVelocitySetpoint, supportPolygonEstimate, step);
         }
      }
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
         twistCalculator.packTwistOfBody(twistStorage, footRigidBody.get(robotQuadrant));
         twistStorage.changeFrame(soleFrame.get(robotQuadrant));
         twistStorage.packAngularPart(soleAngularVelocityEstimate.get(robotQuadrant));
         twistStorage.packLinearPart(soleLinearVelocityEstimate.get(robotQuadrant));
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
      twistCalculator.packTwistOfBody(twistStorage, fullRobotModel.getPelvis());
      twistStorage.changeFrame(bodyFrame);
      twistStorage.packAngularPart(bodyAngularVelocityEstimate);
      twistStorage.packLinearPart(bodyLinearVelocityEstimate);
      bodyOrientationEstimate.setToZero(bodyFrame);
      bodyPositionEstimate.setToZero(bodyFrame);

      // compute center of mass position and velocity
      comPositionEstimate.setToZero(comFrame);
      comJacobian.packCenterOfMassVelocity(comVelocityEstimate);

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
      comHeightEstimate = comPositionEstimate.getZ() - supportPolygonEstimate.getLowestFootStepZHeight();
   }

   private void updateSetpoints()
   {
      double currentTime = robotTimestamp.getDoubleValue();

      // compute capture point natural frequency
      double comHeight = Math.max(comHeightSetpoint, params.get(COM_HEIGHT_NOMINAL) / 5);
      dcmPositionController.setComHeight(comHeight);

      // compute body torque setpoints to track desired body orientation
      bodyTorqueSetpoint.changeFrame(bodyFrame);
      bodyOrientationSetpoint.setIncludingFrame(bodyOrientationDesired);
      bodyOrientationSetpoint.changeFrame(bodyFrame);
      bodyAngularVelocitySetpoint.setToZero(bodyFrame);
      bodyAngularVelocityEstimate.changeFrame(bodyFrame);
      bodyTorqueFeedforwardSetpoint.setToZero(bodyFrame);
      bodyOrientationController.compute(bodyTorqueSetpoint, bodyOrientationSetpoint, bodyAngularVelocitySetpoint, bodyAngularVelocityEstimate, bodyTorqueFeedforwardSetpoint);

      // compute horizontal forces to track desired instantaneous capture point
      if (checkInQuadSupport())
      {
         icpPositionSetpoint.setIncludingFrame(supportCentroidEstimate);
         icpVelocitySetpoint.setToZero(supportFrame);
         dcmPositionSetpoint.setIncludingFrame(icpPositionSetpoint);
         dcmPositionSetpoint.add(0, 0, dcmPositionController.getComHeight());
         dcmVelocitySetpoint.setIncludingFrame(icpVelocitySetpoint);
      }
      else
      {
         // compute dynamic dcm trajectory if robot taking a step
         dcmTrajectory.computeTrajectory(currentTime);
         dcmTrajectory.getPosition(dcmPositionSetpoint);
         dcmTrajectory.getVelocity(dcmVelocitySetpoint);
         icpPositionSetpoint.setIncludingFrame(dcmPositionSetpoint);
         icpPositionSetpoint.sub(0, 0, dcmPositionController.getComHeight());
         icpVelocitySetpoint.setIncludingFrame(dcmVelocitySetpoint);
      }
      dcmPositionController.compute(comForceSetpoint, vrpPositionSetpoint, cmpPositionSetpoint, dcmPositionSetpoint, dcmVelocitySetpoint, dcmPositionEstimate);

      // compute vertical force to track desired center of mass height
      comHeightSetpoint = comHeightDesired;
      double comForceZ = params.get(COM_HEIGHT_GRAVITY_FEEDFORWARD_CONSTANT) * mass * gravity + comHeightController.compute(comHeightEstimate, comHeightSetpoint, comVelocityEstimate.getZ(), 0, controlDT);
      comForceSetpoint.changeFrame(worldFrame);
      comForceSetpoint.setZ(comForceZ);

      // compute optimal contact forces
      contactForceOptimization.setComForceCommand(comForceSetpoint);
      contactForceOptimization.setComTorqueCommand(bodyTorqueSetpoint);
      contactForceOptimization.solve(contactForceLimits, contactForceOptimizationSettings);

      // compute virtual foot forces
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         footStateMachine.get(robotQuadrant).process();
      }

      // compute leg joint torques using virtual model control
      virtualModelController.compute(jointLimits, virtualModelControllerSettings);
   }

   private void readYoVariables()
   {
      yoBodyOrientationDesired.getFrameOrientationIncludingFrame(bodyOrientationDesired);
      comHeightDesired = yoComHeightDesired.getDoubleValue();
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

   private boolean checkInQuadSupport()
   {
      boolean inQuadSupport = true;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (footStateMachine.get(robotQuadrant).getState() != FootState.SUPPORT)
            inQuadSupport = false;
      }
      return inQuadSupport;
   }

   @Override public QuadrupedForceControllerEvent process()
   {
      readYoVariables();
      handleStepEvents();
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
      yoBodyOrientationDesired.setYawPitchRoll(0.0, 0.0, 0.0);
      yoComHeightDesired.set(params.get(COM_HEIGHT_NOMINAL));

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

      // initialize setpoints
      updateEstimates();
      dcmPositionSetpoint.setIncludingFrame(dcmPositionEstimate);
      dcmVelocitySetpoint.setToZero();

      // initialize controllers and state machines
      virtualModelController.reset();
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
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         footStateMachine.get(robotQuadrant).reset();
      }

      // FIXME: provide an external interface to test steps
      double stanceWidth = 0.25;
      double stanceLength = 1.2;
      double stepDuration = 0.50;
      double stepTimeShift = 1.00;
      double strideLength = 0.35;

      double currentTime = robotTimestamp.getDoubleValue();
      TimeInterval timeInterval = new TimeInterval(0, stepDuration).shiftInterval(currentTime);
      FramePoint hindRightGoalPosition = new FramePoint(supportCentroidEstimate);
      FramePoint frontRightGoalPosition = new FramePoint(supportCentroidEstimate);
      FramePoint hindLeftGoalPosition = new FramePoint(supportCentroidEstimate);
      FramePoint frontLeftGoalPosition = new FramePoint(supportCentroidEstimate);
      hindRightGoalPosition.changeFrame(comFrame);
      frontRightGoalPosition.changeFrame(comFrame);
      hindLeftGoalPosition.changeFrame(comFrame);
      frontLeftGoalPosition.changeFrame(comFrame);

      hindRightGoalPosition.add(-stanceLength/2, -stanceWidth/2, 0);
      frontRightGoalPosition.add(stanceLength/2, -stanceWidth/2, 0);
      hindLeftGoalPosition.add(-stanceLength/2 + strideLength/2, stanceWidth/2, 0);
      frontLeftGoalPosition.add(stanceLength/2 + strideLength/2, stanceWidth/2, 0);
      addStep(new QuadrupedTimedStep(RobotQuadrant.HIND_RIGHT, hindRightGoalPosition, timeInterval.shiftInterval(stepTimeShift)));
      addStep(new QuadrupedTimedStep(RobotQuadrant.FRONT_RIGHT, frontRightGoalPosition, timeInterval.shiftInterval(stepTimeShift)));
      addStep(new QuadrupedTimedStep(RobotQuadrant.HIND_LEFT, hindLeftGoalPosition, timeInterval.shiftInterval(stepTimeShift)));
      addStep(new QuadrupedTimedStep(RobotQuadrant.FRONT_LEFT, frontLeftGoalPosition, timeInterval.shiftInterval(stepTimeShift)));

      for (int i = 0; i < 3; i++)
      {
         hindRightGoalPosition.add(strideLength, 0, 0);
         frontRightGoalPosition.add(strideLength, 0, 0);
         hindLeftGoalPosition.add(strideLength, 0, 0);
         frontLeftGoalPosition.add(strideLength, 0, 0);
         addStep(new QuadrupedTimedStep(RobotQuadrant.HIND_RIGHT, hindRightGoalPosition, timeInterval.shiftInterval(stepTimeShift)));
         addStep(new QuadrupedTimedStep(RobotQuadrant.FRONT_RIGHT, frontRightGoalPosition, timeInterval.shiftInterval(stepTimeShift)));
         addStep(new QuadrupedTimedStep(RobotQuadrant.HIND_LEFT, hindLeftGoalPosition, timeInterval.shiftInterval(stepTimeShift)));
         addStep(new QuadrupedTimedStep(RobotQuadrant.FRONT_LEFT, frontLeftGoalPosition, timeInterval.shiftInterval(stepTimeShift)));
      }
   }

   @Override public void onExit()
   {
      // hide graphics
      yoGraphicsList.setVisible(false);
      artifactList.setVisible(false);
      virtualModelController.setVisible(false);
   }

   private class SupportState implements StateMachineState<FootEvent>
   {
      private final RobotQuadrant robotQuadrant;

      public SupportState(RobotQuadrant robotQuadrant)
      {
         this.robotQuadrant = robotQuadrant;
      }

      @Override public void onEntry()
      {
         // initialize contact state
         contactForceOptimization.setContactState(robotQuadrant, ContactState.IN_CONTACT);
         virtualModelController.setSoleContactForceVisible(robotQuadrant, true);
         virtualModelController.setSoleVirtualForceVisible(robotQuadrant, false);
      }

      @Override public FootEvent process()
      {
         // compute sole force setpoint
         contactForceOptimization.getContactForceSolution(robotQuadrant, soleForceSetpoint.get(robotQuadrant));
         virtualModelController.setSoleContactForce(robotQuadrant, soleForceSetpoint.get(robotQuadrant));
         return null;
      }

      @Override public void onExit()
      {
      }
   }

   private class TransferState implements StateMachineState<FootEvent>
   {
      private final RobotQuadrant robotQuadrant;

      public TransferState(RobotQuadrant robotQuadrant)
      {
         this.robotQuadrant = robotQuadrant;
      }

      @Override public void onEntry()
      {
      }

      @Override public FootEvent process()
      {
         double currentTime = robotTimestamp.getDoubleValue();
         double liftOffTime = stepCache.get(robotQuadrant).getTimeInterval().getStartTime();

         // compute sole force setpoint
         contactForceOptimization.getContactForceSolution(robotQuadrant, soleForceSetpoint.get(robotQuadrant));
         virtualModelController.setSoleContactForce(robotQuadrant, soleForceSetpoint.get(robotQuadrant));

         // trigger lift off event
         if (currentTime > liftOffTime)
            return FootEvent.LIFT_OFF;
         else
            return null;
      }

      @Override public void onExit()
      {
      }
   }

   private class SwingState implements StateMachineState<FootEvent>
   {
      private final RobotQuadrant robotQuadrant;

      public SwingState(RobotQuadrant robotQuadrant)
      {
         this.robotQuadrant = robotQuadrant;
      }

      @Override public void onEntry()
      {

         // initialize swing foot controller
         TimeInterval timeInterval = stepCache.get(robotQuadrant).getTimeInterval();
         FramePoint goalPosition = stepCache.get(robotQuadrant).getGoalPosition();
         FramePoint solePosition = solePositionEstimate.get(robotQuadrant);
         solePosition.changeFrame(goalPosition.getReferenceFrame());
         swingFootTrajectory.get(robotQuadrant).setMoveParameters(
               solePosition, goalPosition, params.get(SWING_TRAJECTORY_GROUND_CLEARANCE), timeInterval.getDuration());
         swingPositionController.get(robotQuadrant).reset();
         swingPositionController.get(robotQuadrant).setProportionalGains(params.getVolatileArray(SWING_POSITION_PROPORTIONAL_GAINS));
         swingPositionController.get(robotQuadrant).setIntegralGains(params.getVolatileArray(SWING_POSITION_INTEGRAL_GAINS), params.get(SWING_POSITION_MAX_INTEGRAL_ERROR));
         swingPositionController.get(robotQuadrant).setDerivativeGains(params.getVolatileArray(SWING_POSITION_DERIVATIVE_GAINS));

         // initialize contact state
         contactForceOptimization.setContactState(robotQuadrant, ContactState.NO_CONTACT);
         virtualModelController.setSoleVirtualForceVisible(robotQuadrant, true);
         virtualModelController.setSoleContactForceVisible(robotQuadrant, false);
      }

      @Override public FootEvent process()
      {
         double currentTime = robotTimestamp.getDoubleValue();
         double liftOffTime = stepCache.get(robotQuadrant).getTimeInterval().getStartTime();
         double touchDownTime = stepCache.get(robotQuadrant).getTimeInterval().getEndTime();

         // compute swing trajectory
         swingFootTrajectory.get(robotQuadrant).computeTrajectory(currentTime - liftOffTime);
         swingFootTrajectory.get(robotQuadrant).getPosition(solePositionSetpoint.get(robotQuadrant));

         // compute sole force setpoint
         soleForceSetpoint.get(robotQuadrant).setToZero();
         soleForceSetpoint.get(robotQuadrant).changeFrame(soleFrame.get(robotQuadrant));
         solePositionSetpoint.get(robotQuadrant).changeFrame(soleFrame.get(robotQuadrant));
         soleLinearVelocitySetpoint.get(robotQuadrant).setToZero(soleFrame.get(robotQuadrant));
         soleLinearVelocityEstimate.get(robotQuadrant).changeFrame(soleFrame.get(robotQuadrant));
         soleForceFeedforwardSetpoint.get(robotQuadrant).changeFrame(worldFrame);
         soleForceFeedforwardSetpoint.get(robotQuadrant).set(0, 0, params.get(SWING_POSITION_GRAVITY_FEEDFORWARD_FORCE));
         soleForceFeedforwardSetpoint.get(robotQuadrant).changeFrame(soleFrame.get(robotQuadrant));
         swingPositionController.get(robotQuadrant)
               .compute(soleForceSetpoint.get(robotQuadrant), solePositionSetpoint.get(robotQuadrant), soleLinearVelocitySetpoint.get(robotQuadrant), soleLinearVelocityEstimate.get(robotQuadrant), soleForceFeedforwardSetpoint.get(robotQuadrant));
         virtualModelController.setSoleVirtualForce(robotQuadrant, soleForceSetpoint.get(robotQuadrant));

         // trigger touch down event
         if (currentTime > touchDownTime)
            return FootEvent.TOUCH_DOWN;
         else
            return null;
      }

      @Override public void onExit()
      {
         // change setpoints to world frame when not in use
         soleForceSetpoint.get(robotQuadrant).changeFrame(worldFrame);
         solePositionSetpoint.get(robotQuadrant).changeFrame(worldFrame);
         soleLinearVelocitySetpoint.get(robotQuadrant).changeFrame(worldFrame);
         soleForceFeedforwardSetpoint.get(robotQuadrant).changeFrame(worldFrame);
      }
   }


}
