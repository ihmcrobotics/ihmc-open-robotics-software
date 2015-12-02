package us.ihmc.quadrupedRobotics.controller;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.SdfLoader.OutputWriter;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointNameMap;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedVMCStandParameters;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.quadrupedRobotics.util.HeterogeneousMemoryPool;
import us.ihmc.quadrupedRobotics.virtualModelController.QuadrupedVirtualModelController;
import us.ihmc.robotics.controllers.AxisAngleOrientationController;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.PointJacobian;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.stateMachines.State;
import us.ihmc.sensorProcessing.simulatedSensors.SDFPerfectSimulatedSensorReader;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class QuadrupedVMCStandController extends State<QuadrupedControllerState>
{
   // Parameters
   private final SDFFullRobotModel fullRobotModel;
   private final DoubleYoVariable robotTimestamp;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;
   private final YoVariableRegistry registry;
   private final QuadrupedVMCStandParameters parameters;
   private final QuadrupedJointNameMap jointNameMap;
   private final double dt;
   private final double mass;
   private final double gravity;
   private final double startTime;

   // Utilities
   private final QuadrupedReferenceFrames referenceFrames;
   private final ReferenceFrame worldFrame;
   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame comFrame;
   private QuadrupedSupportPolygon supportPolygon;
   private CenterOfMassJacobian comJacobian;
   private TwistCalculator twistCalculator;
   private AxisAngleOrientationController bodyOrientationController;
   private PIDController comHeightPIDController;
   private PIDController icpForwardPIDController;
   private PIDController icpLateralPIDController;
   private QuadrupedVirtualModelController virtualModelController;

   // Setpoints
   private QuadrantDependentList<YoFrameVector> yoFootForceSetpoint;
   private YoFrameOrientation yoBodyOrientationSetpoint;
   private YoFrameVector yoBodyAngularVelocitySetpoint;
   private YoFrameVector yoBodyTorqueFeedforwardSetpoint;
   private YoFrameVector yoBodyTorqueSetpoint;
   private DoubleYoVariable yoComHeightSetpoint;
   private DoubleYoVariable yoIcpOmegaSetpoint;
   private YoFramePoint yoIcpPositionSetpoint;
   private YoFramePoint yoCmpPositionSetpoint;
   private YoFrameVector yoComForceSetpoint;

   // Estimates
   private QuadrantDependentList<YoFramePoint> yoFootPositionEstimate;
   private YoFramePoint yoSupportCentroidEstimate;
   private YoFrameOrientation yoBodyOrientationEstimate;
   private YoFramePoint yoBodyPositionEstimate;
   private YoFrameVector yoBodyAngularVelocityEstimate;
   private YoFramePoint yoComPositionEstimate;
   private YoFrameVector yoComVelocityEstimate;
   private DoubleYoVariable yoComHeightEstimate;
   private YoFramePoint yoIcpPositionEstimate;

   private HeterogeneousMemoryPool pool = new HeterogeneousMemoryPool();

   public QuadrupedVMCStandController(double dt, QuadrupedRobotParameters robotParameters, SDFFullRobotModel fullRobotModel, DoubleYoVariable robotTimestamp,
         YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(QuadrupedControllerState.VMC_STAND);

      // Parameters
      this.fullRobotModel = fullRobotModel;
      this.robotTimestamp = robotTimestamp;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;
      this.registry = new YoVariableRegistry(getClass().getSimpleName());
      this.parameters = robotParameters.getQuadrupedVMCStandParameters();
      this.jointNameMap = robotParameters.getJointMap();
      this.dt = dt;
      this.mass = fullRobotModel.getTotalMass();
      this.gravity = 9.81;
      this.startTime = robotTimestamp.getDoubleValue();

      // Utilities
      referenceFrames = new QuadrupedReferenceFrames(fullRobotModel, robotParameters.getJointMap(), robotParameters.getPhysicalProperties());
      comFrame = referenceFrames.getCenterOfMassZUpFrame();
      bodyFrame = referenceFrames.getBodyFrame();
      worldFrame = QuadrupedReferenceFrames.getWorldFrame();
      supportPolygon = new QuadrupedSupportPolygon();
      comJacobian = new CenterOfMassJacobian(fullRobotModel.getElevator());
      twistCalculator = new TwistCalculator(worldFrame, fullRobotModel.getElevator());
      bodyOrientationController = new AxisAngleOrientationController("bodyOrientation", bodyFrame, dt, registry);
      bodyOrientationController.setProportionalGains(parameters.getBodyOrientationProportionalGains());
      bodyOrientationController.setIntegralGains(parameters.getBodyOrientationIntegralGains(), parameters.getBodyOrientationMaxIntegralError());
      bodyOrientationController.setDerivativeGains(parameters.getBodyOrientationDerivativeGains());
      comHeightPIDController = new PIDController("bodyHeight", registry);
      comHeightPIDController.setProportionalGain(parameters.getComHeightPropotionalGain());
      comHeightPIDController.setIntegralGain(parameters.getComHeightIntegralGain());
      comHeightPIDController.setMaxIntegralError(parameters.getComHeightMaxIntegralError());
      comHeightPIDController.setDerivativeGain(parameters.getComHeightDerivativeGain());
      icpForwardPIDController = new PIDController("icpForward", registry);
      icpForwardPIDController.setProportionalGain(parameters.getIcpForwardProportionalGain());
      icpForwardPIDController.setIntegralGain(parameters.getIcpForwardIntegralGain());
      icpForwardPIDController.setMaxIntegralError(parameters.getIcpForwardMaxIntegralError());
      icpForwardPIDController.setDerivativeGain(parameters.getIcpForwardDerivativeGain());
      icpLateralPIDController = new PIDController("icpLateral", registry);
      icpLateralPIDController.setProportionalGain(parameters.getIcpForwardProportionalGain());
      icpLateralPIDController.setIntegralGain(parameters.getIcpForwardIntegralGain());
      icpLateralPIDController.setMaxIntegralError(parameters.getIcpForwardMaxIntegralError());
      icpLateralPIDController.setDerivativeGain(parameters.getIcpForwardDerivativeGain());
      virtualModelController = new QuadrupedVirtualModelController(fullRobotModel, referenceFrames, jointNameMap);

      // Setpoints
      yoFootForceSetpoint = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression();
         yoFootForceSetpoint.put(robotQuadrant, new YoFrameVector(prefix + "FootForceSetpoint", worldFrame, registry));
      }
      yoBodyOrientationSetpoint = new YoFrameOrientation("bodyOrientationSetpoint", worldFrame, registry);
      yoBodyAngularVelocitySetpoint = new YoFrameVector("bodyAngularVelocitySetpoint", worldFrame, registry);
      yoBodyTorqueFeedforwardSetpoint = new YoFrameVector("bodyTorqueFeedforwardSetpoint", worldFrame, registry);
      yoBodyTorqueSetpoint = new YoFrameVector("bodyTorqueSetpoint", worldFrame, registry);
      yoComHeightSetpoint = new DoubleYoVariable("comHeightSetpoint", registry);
      yoIcpOmegaSetpoint = new DoubleYoVariable("icpOmegaSetpoint", registry);
      yoIcpPositionSetpoint = new YoFramePoint("icpPositionSetpoint", worldFrame, registry);
      yoCmpPositionSetpoint = new YoFramePoint("cmpPositionSetpoint", worldFrame, registry);
      yoComForceSetpoint = new YoFrameVector("comForceSetpoint", worldFrame, registry);

      // Estimates
      yoFootPositionEstimate = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression();
         yoFootPositionEstimate.put(robotQuadrant, new YoFramePoint(prefix + "FootPositionEstimate", worldFrame, registry));
      }
      yoSupportCentroidEstimate = new YoFramePoint("supportCentroidEstimate", worldFrame, registry);
      yoBodyOrientationEstimate = new YoFrameOrientation("bodyOrientationEstimate", worldFrame, registry);
      yoBodyPositionEstimate = new YoFramePoint("bodyPositionEstimate", worldFrame, registry);
      yoBodyAngularVelocityEstimate = new YoFrameVector("bodyAngularVelocityEstimate", worldFrame, registry);
      yoComPositionEstimate = new YoFramePoint("comPositionEstimate", worldFrame, registry);
      yoComVelocityEstimate = new YoFrameVector("comVelocityEstimate", worldFrame, registry);
      yoComHeightEstimate = new DoubleYoVariable("comHeightEstimate", registry);
      yoIcpPositionEstimate = new YoFramePoint("icpPositionEstimate", worldFrame, registry);

      parentRegistry.addChild(registry);
   }

   private void updateEstimates()
   {
      // update solvers
      referenceFrames.updateFrames();
      twistCalculator.compute();
      comJacobian.compute();

      // compute foot positions
      FramePoint footPosition = pool.lease(FramePoint.class);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         ReferenceFrame footFrame = referenceFrames.getFootFrame(robotQuadrant);
         footPosition.setToZero(footFrame);
         footPosition.changeFrame(worldFrame);
         yoFootPositionEstimate.get(robotQuadrant).set(footPosition);
      }

      // compute support polygon centroid
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         supportPolygon.setFootstep(robotQuadrant, yoFootPositionEstimate.get(robotQuadrant).getFrameTuple());
      }
      FramePoint supportCentroid = pool.lease(FramePoint.class);
      supportPolygon.getCentroid(supportCentroid);
      yoSupportCentroidEstimate.set(supportCentroid);

      // compute body orientation
      FrameOrientation bodyOrientation = pool.lease(FrameOrientation.class);
      bodyOrientation.setToZero(bodyFrame);
      bodyOrientation.changeFrame(worldFrame);
      yoBodyOrientationEstimate.set(bodyOrientation);

      // compute body position
      FramePoint bodyPosition = pool.lease(FramePoint.class);
      bodyPosition.setToZero(bodyFrame);
      bodyPosition.changeFrame(worldFrame);
      yoBodyPositionEstimate.set(bodyPosition);

      // compute body angular velocity
      Twist bodyTwist = pool.lease(Twist.class);
      twistCalculator.packTwistOfBody(bodyTwist, fullRobotModel.getPelvis());

      FrameVector bodyAngularVelocity = pool.lease(FrameVector.class);
      bodyTwist.packAngularPart(bodyAngularVelocity);
      bodyAngularVelocity.changeFrame(worldFrame);
      yoBodyAngularVelocityEstimate.set(bodyAngularVelocity);

      // compute center of mass position
      FramePoint comPosition = pool.lease(FramePoint.class);
      comPosition.setToZero(comFrame);
      comPosition.changeFrame(worldFrame);
      yoComPositionEstimate.set(comPosition);

      // compute center of mass velocity
      FrameVector comVelocity = pool.lease(FrameVector.class);
      comJacobian.packCenterOfMassVelocity(comVelocity);
      comVelocity.changeFrame(worldFrame);
      yoComVelocityEstimate.set(comVelocity);

      // compute center of mass height
      yoComHeightEstimate.set(yoComPositionEstimate.getZ() - yoSupportCentroidEstimate.getZ());

      // compute instantaneous capture point
      double omega = yoIcpOmegaSetpoint.getDoubleValue();
      yoIcpPositionEstimate.setX(yoComPositionEstimate.getX() + yoComVelocityEstimate.getX() / omega);
      yoIcpPositionEstimate.setY(yoComPositionEstimate.getY() + yoComVelocityEstimate.getY() / omega);
      yoIcpPositionEstimate.setZ(yoSupportCentroidEstimate.getZ());
   }

   private void updateSetpoints()
   {
      // compute capture point natural frequency
      yoIcpOmegaSetpoint.set(Math.sqrt(gravity / yoComHeightSetpoint.getDoubleValue()));

      // compute body torque setpoints to track desired body orientation
      FrameOrientation bodyOrientationSetpoint = pool.lease(FrameOrientation.class);
      yoBodyOrientationSetpoint.getFrameOrientationIncludingFrame(bodyOrientationSetpoint);
      bodyOrientationSetpoint.changeFrame(bodyFrame);

      FrameVector bodyAngularVelocitySetpoint = pool.lease(FrameVector.class);
      yoBodyAngularVelocitySetpoint.getFrameTupleIncludingFrame(bodyAngularVelocitySetpoint);
      bodyAngularVelocitySetpoint.changeFrame(bodyFrame);

      FrameVector bodyAngularVelocityEstimate = pool.lease(FrameVector.class);
      yoBodyAngularVelocityEstimate.getFrameTupleIncludingFrame(bodyAngularVelocityEstimate);
      bodyAngularVelocityEstimate.changeFrame(bodyFrame);

      FrameVector bodyTorqueFeedforwardSetpoint = pool.lease(FrameVector.class);
      yoBodyTorqueFeedforwardSetpoint.getFrameTupleIncludingFrame(bodyTorqueFeedforwardSetpoint);
      bodyTorqueFeedforwardSetpoint.changeFrame(bodyFrame);

      FrameVector bodyTorqueSetpoint = pool.lease(FrameVector.class);
      bodyOrientationController.compute(bodyTorqueSetpoint, bodyOrientationSetpoint, bodyAngularVelocitySetpoint, bodyAngularVelocityEstimate,
            bodyTorqueFeedforwardSetpoint);
      bodyTorqueSetpoint.changeFrame(worldFrame);
      yoBodyTorqueSetpoint.set(bodyTorqueSetpoint);

      // compute body force setpoints to track desired capture point and center of mass height
      FrameVector comForceSetpoint = pool.lease(FrameVector.class);
      double omega = yoIcpOmegaSetpoint.getDoubleValue();
      double cmpX = yoIcpPositionEstimate.getX()
            - 1 / omega * icpForwardPIDController.compute(yoIcpPositionEstimate.getX(), yoIcpPositionSetpoint.getX(), 0, 0, dt);
      double cmpY = yoIcpPositionEstimate.getY()
            - 1 / omega * icpLateralPIDController.compute(yoIcpPositionEstimate.getY(), yoIcpPositionSetpoint.getY(), 0, 0, dt);
      double cmpZ = yoIcpPositionSetpoint.getZ();
      double fX = mass * Math.pow(omega, 2) * (yoComPositionEstimate.getX() - cmpX);
      double fY = mass * Math.pow(omega, 2) * (yoComPositionEstimate.getY() - cmpY);
      double fZ = parameters.getComHeightGravityFeedforwardConstant() * mass * gravity
            + comHeightPIDController.compute(yoComHeightEstimate.getDoubleValue(), yoComHeightSetpoint.getDoubleValue(), yoComVelocityEstimate.getZ(), 0, dt);
      yoCmpPositionSetpoint.set(cmpX, cmpY, cmpZ);
      yoComForceSetpoint.set(fX, fY, fZ);
      yoComForceSetpoint.getFrameTupleIncludingFrame(comForceSetpoint);

      // compute joint torques using virtual model control
      virtualModelController.setDesiredComForce(comForceSetpoint);
      virtualModelController.setDesiredBodyTorque(bodyTorqueSetpoint);
      virtualModelController.compute();
   }

   @Override
   public void doAction()
   {
      pool.evict();

      // TODO Auto-generated method stub
      updateEstimates();
      updateSetpoints();
   }

   @Override
   public void doTransitionIntoAction()
   {
      // initialize estimates
      updateEstimates();

      // initialize setpoints
      yoBodyOrientationSetpoint.set(yoBodyOrientationEstimate);
      yoBodyAngularVelocitySetpoint.setToZero();
      yoBodyTorqueFeedforwardSetpoint.setToZero();
      yoBodyTorqueSetpoint.setToZero();
      yoComHeightSetpoint.set(parameters.getComHeightSetpoint());
      yoIcpOmegaSetpoint.set(Math.sqrt(gravity / yoComHeightSetpoint.getDoubleValue()));
      yoIcpPositionSetpoint.setX(yoSupportCentroidEstimate.getX());
      yoIcpPositionSetpoint.setY(yoSupportCentroidEstimate.getY());
      yoIcpPositionSetpoint.setZ(yoSupportCentroidEstimate.getZ());
      yoCmpPositionSetpoint.set(yoIcpPositionSetpoint);
      yoComForceSetpoint.setToZero();

      // initialize leg joint mode to force control
      for(RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         for (LegJointName legJointName : jointNameMap.getLegJointNames())
         {
            String jointName = jointNameMap.getLegJointName(robotQuadrant, legJointName);
            OneDoFJoint joint = fullRobotModel.getOneDoFJointByName(jointName);
            joint.setUnderPositionControl(false);
         }
      }
   }

   @Override
   public void doTransitionOutOfAction()
   {
      // TODO Auto-generated method stub

   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }
}
