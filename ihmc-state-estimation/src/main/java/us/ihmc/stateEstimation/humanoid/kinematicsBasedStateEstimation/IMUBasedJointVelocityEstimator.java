/**
 * Author: Patrick Hammer
 *         2018-02-06
 */
package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Calculates velocity for a group of joints that connect two IMUs
 */
public class IMUBasedJointVelocityEstimator
{
   private final GeometricJacobian jacobian;
   private final IMUSensorReadOnly parentIMU;
   private final IMUSensorReadOnly childIMU;
   private final Map<OneDoFJoint, YoDouble> jointVelocitiesFromIMU = new LinkedHashMap<>();
   private final OneDoFJoint[] joints;
   private final FrameVector3D childAngularVelocity = new FrameVector3D();
   private final FrameVector3D parentAngularVelocity = new FrameVector3D();

   private final DenseMatrix64F jacobianAngularPart64F;
   private final DenseMatrix64F jacobianTransposed;
   private final DenseMatrix64F omega = new DenseMatrix64F(3, 1);
   private final DenseMatrix64F qd_estimated;
   private final DenseMatrix64F inverse;
   private final DenseMatrix64F tempMatrix = new DenseMatrix64F(1, 1);

   public IMUBasedJointVelocityEstimator(GeometricJacobian jacobian, IMUSensorReadOnly parentIMU, IMUSensorReadOnly childIMU, YoVariableRegistry registry)
   {
      this.parentIMU = parentIMU;
      this.childIMU = childIMU;
      this.jacobian = jacobian;

      joints = ScrewTools.filterJoints(jacobian.getJointsInOrder(), OneDoFJoint.class);

      jacobianAngularPart64F = new DenseMatrix64F(3, joints.length);
      jacobianTransposed = new DenseMatrix64F(joints.length, 3);
      qd_estimated = new DenseMatrix64F(joints.length, 1);
      inverse = new DenseMatrix64F(joints.length, joints.length);

      for (OneDoFJoint joint : joints)
      {
         jointVelocitiesFromIMU.put(joint, new YoDouble("qd_" + joint.getName() + "_IMUBased", registry));
      }
   }

   public IMUBasedJointVelocityEstimator(IMUSensorReadOnly parentIMU, IMUSensorReadOnly childIMU, YoVariableRegistry registry)
   {
      this(new GeometricJacobian(parentIMU.getMeasurementLink(), childIMU.getMeasurementLink(), childIMU.getMeasurementLink().getBodyFixedFrame()), parentIMU,
           childIMU, registry);
   }

   /**
    * Calculate the joint velocities and store results in YoVariables
    */
   public void compute()
   {
      jacobian.compute();
      // jacobian is 6xn
      CommonOps.extract(jacobian.getJacobianMatrix(), 0, 3, 0, joints.length, jacobianAngularPart64F, 0, 0);

      CommonOps.transpose(jacobianAngularPart64F, jacobianTransposed);

      // set tempMatrix = J' * J
      //       nxn       nx3  3xn    where n = joints.length
      tempMatrix.reshape(jacobianTransposed.getNumRows(), jacobianTransposed.getNumRows());
      CommonOps.mult(jacobianTransposed, jacobianAngularPart64F, tempMatrix);

      if (Math.abs(CommonOps.det(tempMatrix)) < 1e-5)
      {
         return;
      }

      // set inverse = inv(J' * J)
      CommonOps.invert(tempMatrix, inverse);

      // set tempMatrix = inv(J' * J) * J'
      tempMatrix.reshape(jacobianTransposed.getNumRows(), jacobianTransposed.getNumCols());
      CommonOps.mult(inverse, jacobianTransposed, tempMatrix);

      childAngularVelocity.setToZero(childIMU.getMeasurementFrame());
      childIMU.getAngularVelocityMeasurement(childAngularVelocity);
      childAngularVelocity.changeFrame(jacobian.getJacobianFrame());

      parentAngularVelocity.setToZero(parentIMU.getMeasurementFrame());
      parentIMU.getAngularVelocityMeasurement(parentAngularVelocity);
      parentAngularVelocity.changeFrame(jacobian.getJacobianFrame());
      childAngularVelocity.sub(parentAngularVelocity);

      childAngularVelocity.get(omega);
      CommonOps.mult(tempMatrix, omega, qd_estimated);

      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJoint joint = joints[i];
         double qd_IMU = qd_estimated.get(i, 0);
         jointVelocitiesFromIMU.get(joint).set(qd_IMU);
      }
   }

   public double getEstimatedJointVelocity(OneDoFJoint joint)
   {
      YoDouble estimatedJointVelocity = jointVelocitiesFromIMU.get(joint);
      if (estimatedJointVelocity != null)
         return estimatedJointVelocity.getDoubleValue();
      else
         return Double.NaN;
   }
}
