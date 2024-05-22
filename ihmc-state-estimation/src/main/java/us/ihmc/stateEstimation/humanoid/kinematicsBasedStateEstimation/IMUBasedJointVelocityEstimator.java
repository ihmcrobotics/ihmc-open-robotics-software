/**
 * Author: Patrick Hammer
 *         2018-02-06
 */
package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.mecano.algorithms.GeometricJacobianCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.functionApproximation.DampedLeastSquaresSolver;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Calculates velocity for a group of joints that connect two IMUs
 */
public class IMUBasedJointVelocityEstimator
{
   private final GeometricJacobianCalculator jacobian;
   private final IMUSensorReadOnly parentIMU;
   private final IMUSensorReadOnly childIMU;
   /**
    * Array with 1-element per DoF. This is not necessarily of same length as {@link #joints} as the
    * joints are not necessarily only 1-DoF joints.
    */
   private final YoDouble[] jointVelocitiesFromIMU;
   private final JointBasics[] joints;
   private final FrameVector3D childAngularVelocity = new FrameVector3D();
   private final FrameVector3D parentAngularVelocity = new FrameVector3D();
   private final YoFrameVector3D yoChildAngularVelocity;
   private final YoFrameVector3D yoParentAngularVelocity;

   private final DMatrixRMaj jacobianAngularPart64F;
   private final DMatrixRMaj omega = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj qd_estimated;
   private final DampedLeastSquaresSolver solver;

   private final int degreesOfFreedom;

   public IMUBasedJointVelocityEstimator(IMUSensorReadOnly parentIMU, IMUSensorReadOnly childIMU, YoRegistry registry) throws IllegalArgumentException
   {
      this.parentIMU = parentIMU;
      this.childIMU = childIMU;

      yoChildAngularVelocity = new YoFrameVector3D(childIMU.getSensorName() + "AngularVelocityInParentFrame", parentIMU.getMeasurementLink().getBodyFixedFrame(), registry);
      yoParentAngularVelocity = new YoFrameVector3D(parentIMU.getSensorName() + "AngularVelocityInParentFrame", parentIMU.getMeasurementLink().getBodyFixedFrame(), registry);

      this.jacobian = new GeometricJacobianCalculator();
      jacobian.setKinematicChain(parentIMU.getMeasurementLink(), childIMU.getMeasurementLink());
      jacobian.setJacobianFrame(childIMU.getMeasurementLink().getBodyFixedFrame());

      joints = MultiBodySystemTools.filterJoints(jacobian.getJointsFromBaseToEndEffector(), JointBasics.class).toArray(JointBasics[]::new);
      degreesOfFreedom = jacobian.getNumberOfDegreesOfFreedom();

      if (degreesOfFreedom > 3)
      {
         throw new IllegalArgumentException("Cannot solve for more than 3 DoF betwen IMUs. " + degreesOfFreedom + " DoF were given");
      }

      jointVelocitiesFromIMU = new YoDouble[degreesOfFreedom];

      int dof = 0;

      for (int jointIndex = 0; jointIndex < joints.length; jointIndex++)
      {
         JointBasics joint = joints[jointIndex];

         String jointName = joint.getName();

         if (joint.getDegreesOfFreedom() == 0)
         {
            continue;
         }
         else if (joint.getDegreesOfFreedom() == 1)
         {
            jointVelocitiesFromIMU[dof] = new YoDouble("qd_" + jointName + "_IMUBased", registry);
            dof++;
         }
         else
         {
            for (int i = 0; i < joint.getDegreesOfFreedom(); i++)
            {
               jointVelocitiesFromIMU[dof] = new YoDouble("qd_" + jointName + "_IMUBased" + dof, registry);
               dof++;
            }
         }
      }

      solver = new DampedLeastSquaresSolver(degreesOfFreedom);
      jacobianAngularPart64F = new DMatrixRMaj(3, degreesOfFreedom);
      qd_estimated = new DMatrixRMaj(degreesOfFreedom, 1);
   }

   /**
    * Calculate the joint velocities and store results in YoVariables
    */
   public void compute()
   {
      for (int i = 0; i < joints.length; i++)
      {
         joints[i].updateFrame();
      }

      jacobian.reset();
      // jacobian is 6xn
      CommonOps_DDRM.extract(jacobian.getJacobianMatrix(), 0, 3, 0, degreesOfFreedom, jacobianAngularPart64F, 0, 0);

      childAngularVelocity.setToZero(childIMU.getMeasurementFrame());
      childAngularVelocity.set(childIMU.getAngularVelocityMeasurement());
      childAngularVelocity.changeFrame(jacobian.getJacobianFrame());
      parentAngularVelocity.setToZero(parentIMU.getMeasurementFrame());
      parentAngularVelocity.set(parentIMU.getAngularVelocityMeasurement());
      parentAngularVelocity.changeFrame(jacobian.getJacobianFrame());

      yoChildAngularVelocity.setMatchingFrame(childAngularVelocity);
      yoParentAngularVelocity.setMatchingFrame(parentAngularVelocity);

      childAngularVelocity.sub(parentAngularVelocity);
      childAngularVelocity.get(omega);

      solver.setA(jacobianAngularPart64F);
      solver.solve(omega, qd_estimated);

      for (int i = 0; i < degreesOfFreedom; i++)
      {
         double qd_IMU = qd_estimated.get(i, 0);
         jointVelocitiesFromIMU[i].set(qd_IMU);
      }
   }

   public JointBasics[] getJoints()
   {
      return joints;
   }

   public JointBasics getJoint(int jointIndex)
   {
      return joints[jointIndex];
   }

   public int getDegreesOfFreedom()
   {
      return degreesOfFreedom;
   }

   public double getEstimatedVelocity(int dofIndex)
   {
      return jointVelocitiesFromIMU[dofIndex].getValue();
   }
}
