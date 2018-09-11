package us.ihmc.stateEstimation.ekf;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.screwTheory.GeometricJacobianCalculator;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class LinearAccelerationSensor extends Sensor
{
   private static final int measurementSize = 3;

   private final BiasState biasState;

   private final GeometricJacobianCalculator robotJacobian = new GeometricJacobianCalculator();
   private final List<String> oneDofJointNames = new ArrayList<>();

   private final ReferenceFrame measurementFrame;
   private final FrameVector3D measurement = new FrameVector3D();

   private final double dt;
   private final double sqrtHz;

   private boolean hasBeenCalled = false;

   private final DoubleProvider variance;

   // Temporary variables for computations:
   private final DenseMatrix64F tempRobotState = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F jacobianMatrix = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F jacobianAngularPart = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F jacobianLinearPart = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F qd = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F qdd = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F jointAccelerationTerm = new DenseMatrix64F(Twist.SIZE, 1);
   private final FrameVector3DBasics linearJointTerm = new FrameVector3D();
   private final DenseMatrix64F convectiveTerm = new DenseMatrix64F(Twist.SIZE, 1);
   private final FrameVector3DBasics linearConvectiveTerm = new FrameVector3D();
   private final Twist sensorTwist = new Twist();
   private final FrameVector3DBasics sensorAngularVelocity = new FrameVector3D();
   private final FrameVector3DBasics sensorLinearVelocity = new FrameVector3D();
   private final FrameVector3DBasics centrifugalTerm = new FrameVector3D();
   private final FrameVector3DBasics gravityTerm = new FrameVector3D();
   private final DenseMatrix64F linearJointTermLinearization = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F previousJacobianMatrixLinearPart = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F jacobianDotLinearPart = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F convectiveTermLinearization = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F crossProductLinearization = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F centrifugalTermLinearization = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F gravityTermLinearization = new DenseMatrix64F(0, 0);
   private final Matrix3D gravityPart = new Matrix3D();
   private final RigidBodyTransform rootToMeasurement = new RigidBodyTransform();
   private final RigidBodyTransform rootTransform = new RigidBodyTransform();
   private final Vector3D Aqd = new Vector3D();
   private final Vector3D Lqd = new Vector3D();
   private final Matrix3D Aqdx_matrix = new Matrix3D();
   private final Matrix3D Lqdx_matrix = new Matrix3D();
   private final DenseMatrix64F Aqdx = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F Lqdx = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F tempResult = new DenseMatrix64F(3, 1);
   private final DenseMatrix64F AqdxL = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F LqdxA = new DenseMatrix64F(0, 0);

   public LinearAccelerationSensor(String sensorName, double dt, RigidBody body, ReferenceFrame measurementFrame, boolean estimateBias,
                                   YoVariableRegistry registry)
   {
      this.dt = dt;
      this.sqrtHz = 1.0 / Math.sqrt(dt);
      this.measurementFrame = measurementFrame;

      robotJacobian.setKinematicChain(ScrewTools.getRootBody(body), body);
      robotJacobian.setJacobianFrame(measurementFrame);
      List<OneDoFJoint> oneDofJoints = ScrewTools.filterJoints(robotJacobian.getJointsFromBaseToEndEffector(), OneDoFJoint.class);
      oneDofJoints.stream().forEach(joint -> oneDofJointNames.add(joint.getName()));
      variance = new DoubleParameter(sensorName + "Variance", registry, 1.0);

      if (estimateBias)
      {
         biasState = new BiasState(sensorName, dt, registry);
      }
      else
      {
         biasState = null;
      }

      int degreesOfFreedom = robotJacobian.getNumberOfDegreesOfFreedom();
      jacobianAngularPart.reshape(3, degreesOfFreedom);
      jacobianLinearPart.reshape(3, degreesOfFreedom);
      jacobianDotLinearPart.reshape(3, degreesOfFreedom);
      crossProductLinearization.reshape(3, degreesOfFreedom);
      AqdxL.reshape(3, degreesOfFreedom);
      LqdxA.reshape(3, degreesOfFreedom);
   }

   @Override
   public State getSensorState()
   {
      return biasState == null ? super.getSensorState() : biasState;
   }

   @Override
   public void getSensorJacobian(DenseMatrix64F jacobianToPack)
   {
      if (biasState == null)
      {
         super.getSensorJacobian(jacobianToPack);
      }
      else
      {
         jacobianToPack.reshape(biasState.getSize(), biasState.getSize());
         CommonOps.setIdentity(jacobianToPack);
      }
   }

   @Override
   public int getMeasurementSize()
   {
      return measurementSize;
   }

   /**
    * The measured acceleration is
    * <p>
    * {@code z = Jd * qd + J * qdd + omega x v + Rimu * g}<br>
    * {@code z ~= H * x}
    * </p>
    */
   @Override
   public void getRobotJacobianAndResidual(DenseMatrix64F jacobianToPack, DenseMatrix64F residualToPack, RobotState robotState)
   {
      robotState.getStateVector(tempRobotState);
      robotJacobian.computeJacobianMatrix();
      robotJacobian.computeConvectiveTerm();
      robotJacobian.getJacobianMatrix(jacobianMatrix);
      CommonOps.extract(jacobianMatrix, 0, 3, 0, jacobianMatrix.getNumCols(), jacobianAngularPart, 0, 0);
      CommonOps.extract(jacobianMatrix, 3, 6, 0, jacobianMatrix.getNumCols(), jacobianLinearPart, 0, 0);

      // Compute the residual (non-linear)
      // J * qdd
      FilterTools.packQdd(qdd, oneDofJointNames, tempRobotState, robotState);
      CommonOps.mult(jacobianMatrix, qdd, jointAccelerationTerm);
      linearJointTerm.setIncludingFrame(measurementFrame, 3, jointAccelerationTerm);

      // Jd * qd
      robotJacobian.getConvectiveTerm(convectiveTerm);
      linearConvectiveTerm.setIncludingFrame(measurementFrame, 3, convectiveTerm);

      // w x v
      robotJacobian.getEndEffector().getBodyFixedFrame().getTwistOfFrame(sensorTwist);
      sensorTwist.changeFrame(measurementFrame);
      centrifugalTerm.setToZero(measurementFrame);
      sensorTwist.getAngularPart(sensorAngularVelocity);
      sensorTwist.getLinearPart(sensorLinearVelocity);
      centrifugalTerm.cross(sensorAngularVelocity, sensorLinearVelocity);

      // R * g
      gravityTerm.setIncludingFrame(ReferenceFrame.getWorldFrame(), 0.0, 0.0, robotState.getGravity());
      gravityTerm.changeFrame(measurementFrame);

      // Compute the residual by substracting all terms from the measurement:
      residualToPack.reshape(measurementSize, 1);
      residualToPack.set(0, measurement.getX() - linearJointTerm.getX() - linearConvectiveTerm.getX() - centrifugalTerm.getX() - gravityTerm.getX());
      residualToPack.set(1, measurement.getY() - linearJointTerm.getY() - linearConvectiveTerm.getY() - centrifugalTerm.getY() - gravityTerm.getY());
      residualToPack.set(2, measurement.getZ() - linearJointTerm.getZ() - linearConvectiveTerm.getZ() - centrifugalTerm.getZ() - gravityTerm.getZ());

      if (biasState != null)
      {
         residualToPack.set(0, residualToPack.get(0) - biasState.getBias(0));
         residualToPack.set(1, residualToPack.get(1) - biasState.getBias(1));
         residualToPack.set(2, residualToPack.get(2) - biasState.getBias(2));
      }

      // Now for assembling the linearized measurement model:
      // J * qdd
      FilterTools.insertForAcceleration(linearJointTermLinearization, oneDofJointNames, jacobianLinearPart, robotState);

      // Jd * qd (numerical)
      if (!hasBeenCalled)
      {
         jacobianDotLinearPart.zero();
         hasBeenCalled = true;
      }
      else
      {
         CommonOps.subtract(jacobianLinearPart, previousJacobianMatrixLinearPart, jacobianDotLinearPart);
         CommonOps.scale(1.0 / dt, jacobianDotLinearPart);
      }
      FilterTools.insertForVelocity(convectiveTermLinearization, oneDofJointNames, jacobianDotLinearPart, robotState);
      previousJacobianMatrixLinearPart.set(jacobianLinearPart);

      // w x v
      FilterTools.packQd(qd, oneDofJointNames, tempRobotState, robotState);
      linearizeCrossProduct(jacobianAngularPart, jacobianLinearPart, qd, crossProductLinearization);
      FilterTools.insertForVelocity(centrifugalTermLinearization, oneDofJointNames, crossProductLinearization, robotState);

      // R * g (used only with floating joints) (skip the joint angles - only correct the base orientation)
      gravityTermLinearization.reshape(measurementSize, robotState.getSize());
      gravityTermLinearization.zero();
      if (robotState.isFloating())
      {
         ReferenceFrame rootFrame = robotJacobian.getJointsFromBaseToEndEffector().get(0).getFrameAfterJoint();
         ReferenceFrame baseFrame = robotJacobian.getJointsFromBaseToEndEffector().get(0).getFrameBeforeJoint();
         rootFrame.getTransformToDesiredFrame(rootToMeasurement, measurementFrame);
         baseFrame.getTransformToDesiredFrame(rootTransform, rootFrame);

         gravityPart.setToTildeForm(gravityTerm);
         gravityPart.multiply(rootToMeasurement.getRotationMatrix());
         gravityPart.multiply(rootTransform.getRotationMatrix());
         gravityPart.get(0, robotState.findOrientationIndex(), gravityTermLinearization);
      }

      // Add all linearizations together:
      jacobianToPack.set(linearJointTermLinearization);
      CommonOps.add(jacobianToPack, convectiveTermLinearization, jacobianToPack);
      CommonOps.add(jacobianToPack, centrifugalTermLinearization, jacobianToPack);
      CommonOps.add(jacobianToPack, gravityTermLinearization, jacobianToPack);
   }

   @Override
   public void getRMatrix(DenseMatrix64F matrixToPack)
   {
      matrixToPack.reshape(measurementSize, measurementSize);
      CommonOps.setIdentity(matrixToPack);
      CommonOps.scale(variance.getValue() * sqrtHz, matrixToPack);
   }

   public void setMeasurement(Vector3DReadOnly measurement)
   {
      this.measurement.setIncludingFrame(robotJacobian.getJacobianFrame(), measurement);
   }

   /**
    * This linearizes the cross product {@code f(qd)=[A*qd]x[L*qd]} around {@code qd0}. This allows a first
    * order approximation of:
    * <br>{@code f(qd1) = f(qd0) + J * [qd1 - qd0]}</br>
    * This approximation will be accurate for small values of {@code dqd = [qd1 - qd0]}.
    *
    * @param A matrix in the above equation
    * @param L matrix in the above equation
    * @param qd0 the point to linearize about
    * @return {@code J} is the Jacobian of the above cross product w.r.t. {@code qd}
    */
   public void linearizeCrossProduct(DenseMatrix64F A, DenseMatrix64F L, DenseMatrix64F qd0, DenseMatrix64F matrixToPack)
   {
      CommonOps.mult(A, qd0, tempResult);
      Aqd.set(tempResult);
      CommonOps.mult(L, qd0, tempResult);
      Lqd.set(tempResult);

      Aqdx_matrix.setToTildeForm(Aqd);
      Lqdx_matrix.setToTildeForm(Lqd);

      Aqdx_matrix.get(Aqdx);
      Lqdx_matrix.get(Lqdx);

      CommonOps.mult(Aqdx, L, AqdxL);
      CommonOps.mult(Lqdx, A, LqdxA);
      CommonOps.subtract(AqdxL, LqdxA, matrixToPack);
   }

   public void reset()
   {
      if (biasState != null)
      {
         biasState.reset();
      }
   }
}
