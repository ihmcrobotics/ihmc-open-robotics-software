/**
 * Author: Patrick Hammer
 *         2018-02-06
 */
package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.robotics.functionApproximation.DampedLeastSquaresSolver;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Calculates velocity for a group of joints that connect two IMUs
 */
public class IMUBasedJointVelocityEstimator
{
   private final GeometricJacobian jacobian;
   private final IMUSensorReadOnly parentIMU;
   private final IMUSensorReadOnly childIMU;
   private final YoDouble[] jointVelocitiesFromIMU;
   private final OneDoFJoint[] joints;
   private final FrameVector3D childAngularVelocity = new FrameVector3D();
   private final FrameVector3D parentAngularVelocity = new FrameVector3D();

   private final DenseMatrix64F jacobianAngularPart64F;
   private final DenseMatrix64F omega = new DenseMatrix64F(3, 1);
   private final DenseMatrix64F qd_estimated;
   private final DampedLeastSquaresSolver solver;

   public IMUBasedJointVelocityEstimator(GeometricJacobian jacobian, IMUSensorReadOnly parentIMU, IMUSensorReadOnly childIMU, YoVariableRegistry registry)
         throws IllegalArgumentException
   {
      this.parentIMU = parentIMU;
      this.childIMU = childIMU;
      this.jacobian = jacobian;

      joints = ScrewTools.filterJoints(jacobian.getJointsInOrder(), OneDoFJoint.class);
      if (joints.length > 3)
      {
         throw new IllegalArgumentException("Cannot solve for more than 3 DoF betwen IMUs. " + joints.length + " DoF were given");
      }
      if (joints.length != jacobian.getJointsInOrder().length)
      {
         throw new IllegalArgumentException("Can only solve for a chain of OneDoFJoints");
      }

      jointVelocitiesFromIMU = new YoDouble[joints.length];
      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJoint joint = joints[i];
         jointVelocitiesFromIMU[i] = new YoDouble("qd_" + joint.getName() + "_IMUBased", registry);
      }

      solver = new DampedLeastSquaresSolver(joints.length);
      jacobianAngularPart64F = new DenseMatrix64F(3, joints.length);
      qd_estimated = new DenseMatrix64F(joints.length, 1);
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

      childAngularVelocity.setToZero(childIMU.getMeasurementFrame());
      childAngularVelocity.set(childIMU.getAngularVelocityMeasurement());
      childAngularVelocity.changeFrame(jacobian.getJacobianFrame());
      parentAngularVelocity.setToZero(parentIMU.getMeasurementFrame());
      parentAngularVelocity.set(parentIMU.getAngularVelocityMeasurement());
      parentAngularVelocity.changeFrame(jacobian.getJacobianFrame());
      childAngularVelocity.sub(parentAngularVelocity);
      childAngularVelocity.get(omega);

      solver.setA(jacobianAngularPart64F);
      solver.solve(omega, qd_estimated);

      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJoint joint = joints[i];
         double qd_IMU = qd_estimated.get(i, 0);
         jointVelocitiesFromIMU[i].set(qd_IMU);
      }
   }

   public double getEstimatedJointVelocity(OneDoFJoint joint)
   {
      for (int i = 0; i < joints.length; i++)
      {
         if (joints[i] == joint)
         {
            return jointVelocitiesFromIMU[i].getDoubleValue();
         }
      }
      return Double.NaN;
   }

   public double getEstimatedJointVelocity(int jointIndex)
   {
      if (jointIndex < 0 || jointIndex >= joints.length)
      {
         throw new IndexOutOfBoundsException("Joint index out of bounds");
      }
      return jointVelocitiesFromIMU[jointIndex].getDoubleValue();
   }
}
