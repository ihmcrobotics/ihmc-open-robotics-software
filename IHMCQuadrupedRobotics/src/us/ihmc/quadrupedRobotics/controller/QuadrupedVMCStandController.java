package us.ihmc.quadrupedRobotics.controller;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointLimits;
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
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.stateMachines.State;
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
   private final QuadrupedJointLimits jointLimits;
   private final double dt;
   private final double mass;
   private final double gravity;

   // Utilities
   private final QuadrupedReferenceFrames referenceFrames;
   private final ReferenceFrame worldFrame;
   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame comFrame;
   private PoseReferenceFrame supportFrame;
   private QuadrupedSupportPolygon supportPolygon;
   private CenterOfMassJacobian comJacobian;
   private TwistCalculator twistCalculator;
   private AxisAngleOrientationController bodyOrientationController;
   private PIDController comHeightPIDController;
   private PIDController icpForwardPIDController;
   private PIDController icpLateralPIDController;
   private QuadrupedVirtualModelController virtualModelController;

   // Setpoints
   private QuadrantDependentList<YoFrameVector> yoSoleForceSetpoint;
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
   private QuadrantDependentList<YoFramePoint> yoSolePositionEstimate;
   private YoFramePoint yoSupportCentroidEstimate;
   private YoFrameOrientation yoSupportOrientationEstimate;
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
      this.jointLimits = robotParameters.getJointLimits();
      this.dt = dt;
      this.mass = fullRobotModel.getTotalMass();
      this.gravity = 9.81;

      // Utilities
      referenceFrames = new QuadrupedReferenceFrames(fullRobotModel, robotParameters.getJointMap(), robotParameters.getPhysicalProperties());
      comFrame = referenceFrames.getCenterOfMassZUpFrame();
      bodyFrame = referenceFrames.getBodyFrame();
      worldFrame = QuadrupedReferenceFrames.getWorldFrame();
      supportFrame = new PoseReferenceFrame("SupportFrame", QuadrupedReferenceFrames.getWorldFrame());
      supportPolygon = new QuadrupedSupportPolygon();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         supportPolygon.setFootstep(robotQuadrant, new FramePoint(worldFrame));
      }
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
      yoSoleForceSetpoint = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression();
         yoSoleForceSetpoint.put(robotQuadrant, new YoFrameVector(prefix + "SoleForceSetpoint", worldFrame, registry));
      }
      yoBodyOrientationSetpoint = new YoFrameOrientation("bodyOrientationSetpoint", supportFrame, registry);
      yoBodyAngularVelocitySetpoint = new YoFrameVector("bodyAngularVelocitySetpoint", supportFrame, registry);
      yoBodyTorqueFeedforwardSetpoint = new YoFrameVector("bodyTorqueFeedforwardSetpoint", supportFrame, registry);
      yoBodyTorqueSetpoint = new YoFrameVector("bodyTorqueSetpoint", worldFrame, registry);
      yoComHeightSetpoint = new DoubleYoVariable("comHeightSetpoint", registry);
      yoIcpOmegaSetpoint = new DoubleYoVariable("icpOmegaSetpoint", registry);
      yoIcpPositionSetpoint = new YoFramePoint("icpPositionSetpoint", worldFrame, registry);
      yoCmpPositionSetpoint = new YoFramePoint("cmpPositionSetpoint", worldFrame, registry);
      yoComForceSetpoint = new YoFrameVector("comForceSetpoint", worldFrame, registry);

      // Estimates
      yoSolePositionEstimate = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression();
         yoSolePositionEstimate.put(robotQuadrant, new YoFramePoint(prefix + "SolePositionEstimate", worldFrame, registry));
      }
      yoSupportCentroidEstimate = new YoFramePoint("supportCentroidEstimate", worldFrame, registry);
      yoSupportOrientationEstimate = new YoFrameOrientation("supportOrientationEstimate", worldFrame, registry);
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
      // update frames and twists
      referenceFrames.updateFrames();
      twistCalculator.compute();
      comJacobian.compute();

      // compute sole positions
      FramePoint solePosition = pool.lease(FramePoint.class);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         ReferenceFrame soleFrame = referenceFrames.getFootFrame(robotQuadrant);
         solePosition.setToZero(soleFrame);
         solePosition.changeFrame(supportPolygon.getReferenceFrame());
         yoSolePositionEstimate.get(robotQuadrant).set(solePosition);
      }

      // compute support polygon
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         supportPolygon.setFootstep(robotQuadrant, yoSolePositionEstimate.get(robotQuadrant).getFrameTuple());
      }

      // compute support frame (centroid and nominal orientation)
      FramePoint supportCentroid = pool.lease(FramePoint.class);
      FrameOrientation supportOrientation = pool.lease(FrameOrientation.class);
      supportPolygon.getCentroid(supportCentroid);
      supportOrientation.changeFrame(supportPolygon.getReferenceFrame());
      supportOrientation.setYawPitchRoll(supportPolygon.getNominalYaw(), 0, 0);
      supportFrame.setPoseAndUpdate(supportCentroid, supportOrientation);
      yoSupportCentroidEstimate.set(supportCentroid);
      yoSupportOrientationEstimate.set(supportOrientation);

      // compute body orientation
      FrameOrientation bodyOrientation = pool.lease(FrameOrientation.class);
      bodyOrientation.setToZero(bodyFrame);
      bodyOrientation.changeFrame(yoBodyOrientationEstimate.getReferenceFrame());
      yoBodyOrientationEstimate.set(bodyOrientation);

      // compute body position
      FramePoint bodyPosition = pool.lease(FramePoint.class);
      bodyPosition.setToZero(bodyFrame);
      bodyPosition.changeFrame(yoBodyPositionEstimate.getReferenceFrame());
      yoBodyPositionEstimate.set(bodyPosition);

      // compute body angular velocity
      Twist bodyTwist = pool.lease(Twist.class);
      twistCalculator.packTwistOfBody(bodyTwist, fullRobotModel.getPelvis());
      FrameVector bodyAngularVelocity = pool.lease(FrameVector.class);
      bodyTwist.packAngularPart(bodyAngularVelocity);
      bodyAngularVelocity.changeFrame(yoBodyAngularVelocityEstimate.getReferenceFrame());
      yoBodyAngularVelocityEstimate.set(bodyAngularVelocity);

      // compute center of mass position
      FramePoint comPosition = pool.lease(FramePoint.class);
      comPosition.setToZero(comFrame);
      comPosition.changeFrame(yoComPositionEstimate.getReferenceFrame());
      yoComPositionEstimate.set(comPosition);

      // compute center of mass velocity
      FrameVector comVelocity = pool.lease(FrameVector.class);
      comJacobian.packCenterOfMassVelocity(comVelocity);
      comVelocity.changeFrame(yoComVelocityEstimate.getReferenceFrame());
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
      double comHeight = Math.max(yoComHeightSetpoint.getDoubleValue(), parameters.getComHeightSetpoint() / 5);
      yoIcpOmegaSetpoint.set(Math.sqrt(gravity / comHeight));

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
      bodyTorqueSetpoint.changeFrame(yoBodyTorqueSetpoint.getReferenceFrame());
      yoBodyTorqueSetpoint.set(bodyTorqueSetpoint);

      // compute centroidal force setpoints to track desired capture point and center of mass height
      FrameVector comForceSetpoint = pool.lease(FrameVector.class);
      FramePoint icpPositionSetpoint = pool.lease(FramePoint.class);
      FramePoint icpPositionEstimate = pool.lease(FramePoint.class);
      FramePoint cmpPositionSetpoint = pool.lease(FramePoint.class);
      icpPositionSetpoint.setIncludingFrame(yoSupportCentroidEstimate.getFrameTuple());
      icpPositionEstimate.setIncludingFrame(yoIcpPositionEstimate.getFrameTuple());
      icpPositionSetpoint.changeFrame(comFrame);
      icpPositionEstimate.changeFrame(comFrame);
      cmpPositionSetpoint.changeFrame(comFrame);
      double omega = yoIcpOmegaSetpoint.getDoubleValue();
      double cmpX = icpPositionEstimate.getX()
            - 1 / omega * icpForwardPIDController.compute(icpPositionEstimate.getX(), icpPositionSetpoint.getX(), 0, 0, dt);
      double cmpY = icpPositionEstimate.getY()
            - 1 / omega * icpLateralPIDController.compute(icpPositionEstimate.getY(), icpPositionSetpoint.getY(), 0, 0, dt);
      double cmpZ = icpPositionSetpoint.getZ();
      double fX = mass * Math.pow(omega, 2) * -cmpX;
      double fY = mass * Math.pow(omega, 2) * -cmpY;
      double fZ = parameters.getComHeightGravityFeedforwardConstant() * mass * gravity
            + comHeightPIDController.compute(yoComHeightEstimate.getDoubleValue(), yoComHeightSetpoint.getDoubleValue(), yoComVelocityEstimate.getZ(), 0, dt);
      cmpPositionSetpoint.set(cmpX, cmpY, cmpZ);
      comForceSetpoint.set(fX, fY, fZ);
      icpPositionSetpoint.changeFrame(yoIcpPositionSetpoint.getReferenceFrame());
      cmpPositionSetpoint.changeFrame(yoCmpPositionSetpoint.getReferenceFrame());
      comForceSetpoint.changeFrame(yoComForceSetpoint.getReferenceFrame());
      yoIcpPositionSetpoint.set(icpPositionSetpoint);
      yoCmpPositionSetpoint.set(cmpPositionSetpoint);
      yoComForceSetpoint.set(comForceSetpoint);

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
      yoBodyOrientationSetpoint.setYawPitchRoll(0, 0, 0);
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

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         for (LegJointName legJointName : jointNameMap.getLegJointNames())
         {
            String jointName = jointNameMap.getLegJointName(robotQuadrant, legJointName);

            // initialize leg joint mode to force control
            OneDoFJoint joint = fullRobotModel.getOneDoFJointByName(jointName);
            joint.setUnderPositionControl(false);

            // initialize controller joint limits
            double positionLowerLimit = jointLimits.getJointSoftPositionLowerLimit(jointName);
            double positionUpperLimit = jointLimits.getJointSoftPositionUpperLimit(jointName);
            double effortLimit = jointLimits.getJointEffortLimit(jointName);
            virtualModelController.setJointEffortLimits(jointName, -effortLimit, effortLimit);
            virtualModelController.setJointPositionLimits(jointName, positionLowerLimit, positionUpperLimit);
            virtualModelController.setJointPositionLimitStiffness(jointName, parameters.getJointPositionLimitStiffness());
            virtualModelController.setJointPositionLimitDamping(jointName, parameters.getJointPositionLimitDamping());
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
