package us.ihmc.quadrupedRobotics.controller;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.SdfLoader.OutputWriter;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointNameMap;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedVMCStandParameters;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.quadrupedRobotics.util.HeterogeneousMemoryPool;
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
   private final SDFFullRobotModel sdfFullRobotModel;
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

   public QuadrupedVMCStandController(double dt, QuadrupedRobotParameters robotParameters, SDFFullRobotModel sdfFullRobotModel, DoubleYoVariable robotTimestamp,
         YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(QuadrupedControllerState.VMC_STAND);
      
      // Parameters
      this.sdfFullRobotModel = sdfFullRobotModel;
      this.robotTimestamp = robotTimestamp;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;
      this.registry = new YoVariableRegistry(getClass().getSimpleName());
      this.parameters = robotParameters.getQuadrupedVMCStandParameters();
      this.jointNameMap = robotParameters.getJointMap();
      this.dt = dt;
      this.mass = sdfFullRobotModel.getTotalMass();
      this.gravity = 9.81;
      this.startTime = robotTimestamp.getDoubleValue();
      
      // Utilities
      referenceFrames = new QuadrupedReferenceFrames(sdfFullRobotModel, robotParameters.getJointMap(), robotParameters.getPhysicalProperties());
      comFrame = referenceFrames.getCenterOfMassFrame();
      bodyFrame = referenceFrames.getBodyFrame();
      worldFrame = QuadrupedReferenceFrames.getWorldFrame();
      supportPolygon = new QuadrupedSupportPolygon();
      comJacobian = new CenterOfMassJacobian(sdfFullRobotModel.getElevator());
      twistCalculator = new TwistCalculator(worldFrame, sdfFullRobotModel.getElevator());
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
         yoFootPositionEstimate.get(robotQuadrant).setAndMatchFrame(footPosition);
      }

      // compute support polygon centroid
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         supportPolygon.setFootstep(robotQuadrant, yoFootPositionEstimate.get(robotQuadrant).getFrameTuple());
      }
      FramePoint supportCentroid = pool.lease(FramePoint.class);
      supportPolygon.getCentroid(supportCentroid);
      yoSupportCentroidEstimate.setAndMatchFrame(supportCentroid);

      // compute body orientation
      FrameOrientation bodyOrientation = pool.lease(FrameOrientation.class);
      bodyOrientation.setToZero(bodyFrame);
      bodyOrientation.changeFrame(worldFrame);
      yoBodyOrientationEstimate.setAndMatchFrame(bodyOrientation);

      // compute body position
      FramePoint bodyPosition = pool.lease(FramePoint.class);
      bodyPosition.setToZero(bodyFrame);
      bodyPosition.changeFrame(worldFrame);
      yoBodyPositionEstimate.setAndMatchFrame(bodyPosition);

      // compute body angular velocity
      Twist bodyTwist = pool.lease(Twist.class);
      twistCalculator.packTwistOfBody(bodyTwist, sdfFullRobotModel.getPelvis());

      FrameVector bodyAngularVelocity = pool.lease(FrameVector.class);
      bodyTwist.packAngularPart(bodyAngularVelocity);
      yoBodyAngularVelocityEstimate.setAndMatchFrame(bodyAngularVelocity);

      // compute center of mass position
      FramePoint comPosition = pool.lease(FramePoint.class);
      comPosition.setToZero(comFrame);
      comPosition.changeFrame(worldFrame);
      yoComPositionEstimate.setAndMatchFrame(comPosition);

      // compute center of mass velocity
      FrameVector comVelocity = pool.lease(FrameVector.class);
      comJacobian.packCenterOfMassVelocity(comVelocity);
      comVelocity.changeFrame(worldFrame);
      yoComVelocityEstimate.setAndMatchFrame(comVelocity);

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
      bodyOrientationController.compute(bodyTorqueSetpoint, bodyOrientationSetpoint, bodyAngularVelocitySetpoint, bodyAngularVelocityEstimate, bodyTorqueFeedforwardSetpoint);
      bodyTorqueSetpoint.changeFrame(worldFrame);
      yoBodyTorqueSetpoint.setAndMatchFrame(bodyTorqueSetpoint);

      // compute body force setpoints to track desired capture point and center of mass height
      double omega = yoIcpOmegaSetpoint.getDoubleValue();
      double cmpX = yoIcpPositionEstimate.getX() - 1 / omega * icpForwardPIDController.compute(yoIcpPositionEstimate.getX(), yoIcpPositionSetpoint.getX(), 0, 0, dt);
      double cmpY = yoIcpPositionEstimate.getY() - 1 / omega * icpLateralPIDController.compute(yoIcpPositionEstimate.getY(), yoIcpPositionSetpoint.getY(), 0, 0, dt);
      double cmpZ = yoIcpPositionSetpoint.getZ();
      double fX = mass * Math.pow(omega, 2) * (yoComPositionEstimate.getX() - cmpX);
      double fY = mass * Math.pow(omega, 2) * (yoComPositionEstimate.getY() - cmpY);
      double fZ = 0.75 * mass * gravity
            + comHeightPIDController.compute(yoComHeightEstimate.getDoubleValue(), yoComHeightSetpoint.getDoubleValue(), yoComVelocityEstimate.getZ(), 0, dt);
      yoCmpPositionSetpoint.set(cmpX, cmpY, cmpZ);
      yoComForceSetpoint.set(fX, fY, fZ);

      // compute ground reaction forces using least squares optimization
      distributeForcesFourLegs(yoComForceSetpoint.getX(), yoComForceSetpoint.getY(), yoComForceSetpoint.getZ(), yoBodyTorqueSetpoint.getX(), yoBodyTorqueSetpoint.getY(),
            yoBodyTorqueSetpoint.getZ());

      // compute joint torques using virtual model control
      computeLegTorques();
   }

   public void distributeForcesFourLegs(double fX, double fY, double fZ, double nX, double nY, double nZ)
   {

      // compute constraint matrix (mapping from foot forces to body wrench)
      DenseMatrix64F forceConstraintsMatrix = new DenseMatrix64F(6, 12);
      forceConstraintsMatrix.zero();
      int columnOffset = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         // compute foot offset relative to body in world coordinates
         FramePoint footOffset = pool.lease(FramePoint.class);
         footOffset.setIncludingFrame(yoFootPositionEstimate.get(robotQuadrant).getFrameTuple());
         footOffset.sub(yoBodyPositionEstimate.getFrameTuple());

         // populate constraint matrix coefficients
         forceConstraintsMatrix.set(0, 0 + columnOffset, 1.0); // fX row
         forceConstraintsMatrix.set(1, 1 + columnOffset, 1.0); // fY row
         forceConstraintsMatrix.set(2, 2 + columnOffset, 1.0); // fZ row
         forceConstraintsMatrix.set(3, 1 + columnOffset, -footOffset.getZ()); // nX row
         forceConstraintsMatrix.set(3, 2 + columnOffset, footOffset.getY());
         forceConstraintsMatrix.set(4, 0 + columnOffset, footOffset.getZ()); // nY row
         forceConstraintsMatrix.set(4, 2 + columnOffset, -footOffset.getX());
         forceConstraintsMatrix.set(5, 0 + columnOffset, -footOffset.getY()); // nZ row
         forceConstraintsMatrix.set(5, 1 + columnOffset, footOffset.getX());
         columnOffset += 3;
      }

      // compute constraint vector (desired body wrench)
      DenseMatrix64F totalForcesOnTheBody = new DenseMatrix64F(6, 1);
      totalForcesOnTheBody.set(0, 0, fX);
      totalForcesOnTheBody.set(1, 0, fY);
      totalForcesOnTheBody.set(2, 0, fZ);
      totalForcesOnTheBody.set(3, 0, nX);
      totalForcesOnTheBody.set(4, 0, nY);
      totalForcesOnTheBody.set(5, 0, nZ);

      // solve for optimal foot forces in world coordinates using least squares
      DenseMatrix64F forceConstraintsMatrixInverse = new DenseMatrix64F(12, 6);
      try
      {
         CommonOps.pinv(forceConstraintsMatrix, forceConstraintsMatrixInverse);
      }
      catch (Exception e)
      {
         System.out.println("forceConstraintsMatrix = " + forceConstraintsMatrix);
         return;
      }
      DenseMatrix64F distributedForcesOnTheLegs = new DenseMatrix64F(12, 1);
      CommonOps.mult(forceConstraintsMatrixInverse, totalForcesOnTheBody, distributedForcesOnTheLegs);
      int rowOffset = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         yoFootForceSetpoint.get(robotQuadrant).setX(distributedForcesOnTheLegs.get(0 + rowOffset, 0));
         yoFootForceSetpoint.get(robotQuadrant).setY(distributedForcesOnTheLegs.get(1 + rowOffset, 0));
         yoFootForceSetpoint.get(robotQuadrant).setZ(distributedForcesOnTheLegs.get(2 + rowOffset, 0));
         rowOffset += 3;
      }

      //	    System.out.println("forceConstraintsMatrix = " + forceConstraintsMatrix);
      //	    System.out.println("totalForcesOnTheBody = " + totalForcesOnTheBody);
      //	    System.out.println("distributedForcesOnTheLegs = " + distributedForcesOnTheLegs);
   }

   private void computeLegTorques()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         // compute foot force in body coordinates
         FrameVector footForce = pool.lease(FrameVector.class);
         yoFootForceSetpoint.get(robotQuadrant).getFrameTupleIncludingFrame(footForce);
         footForce.changeFrame(bodyFrame);

         // compute foot position in body coordinates
         FramePoint footPosition = pool.lease(FramePoint.class);
         yoFootPositionEstimate.get(robotQuadrant).getFrameTupleIncludingFrame(footPosition);
         footPosition.changeFrame(bodyFrame);

         // get joint array for the leg chain
         OneDoFJoint kneeJoint = sdfFullRobotModel.getOneDoFJointByName(jointNameMap.getJointBeforeFootName(robotQuadrant));
         RigidBody body = sdfFullRobotModel.getRootJoint().getSuccessor();
         RigidBody shin = kneeJoint.getSuccessor();
         OneDoFJoint[] legJoints = ScrewTools.filterJoints(ScrewTools.createJointPath(body, shin), OneDoFJoint.class);

         // compute foot Jacobian in body coordinates
         GeometricJacobian shinJacobian = new GeometricJacobian(legJoints, bodyFrame);
         shinJacobian.compute();
         PointJacobian footJacobian = new PointJacobian();
         footJacobian.set(shinJacobian, footPosition);
         footJacobian.compute();

         // solve for leg torques using Jacobian transpose
         DenseMatrix64F jacobian = footJacobian.getJacobianMatrix();
         DenseMatrix64F forces = new DenseMatrix64F(3, 1);
         forces.set(0, 0, -footForce.getX());
         forces.set(1, 0, -footForce.getY());
         forces.set(2, 0, -footForce.getZ());
         DenseMatrix64F torques = new DenseMatrix64F(1, jacobian.getNumCols());
         CommonOps.multTransA(forces, jacobian, torques);
         CommonOps.transpose(torques);

         // update joint torque setpoints in full robot models
         int row = 0;
         for (OneDoFJoint joint : legJoints)
         {
            joint.setTau(torques.get(row++, 0));
         }

         // hack to get knees to bend without stand_prep state
         if (robotTimestamp.getDoubleValue() - startTime < 0.1)
         {
            for (OneDoFJoint joint : legJoints)
            {
               joint.setTau(0);
            }
            kneeJoint.setTau(robotQuadrant.getEnd().negateIfFrontEnd(5));
         }

         //		    System.out.println("forces = " + forces);
         //		    System.out.println("jacobian = " + jacobian);
         //		    System.out.println("torques = " + torques);
      }

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

      for (OneDoFJoint oneDofJoint : sdfFullRobotModel.getOneDoFJoints())
      {
         oneDofJoint.setUnderPositionControl(false);
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
