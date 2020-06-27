package us.ihmc.quadrupedRobotics.estimator.footSwitch;

import java.util.List;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.misc.UnrolledInverseFromMinor_DDRM;

import us.ihmc.commonWalkingControlModules.touchdownDetector.WrenchCalculator;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.GeometricJacobian;

public class JointTorqueBasedWrenchCalculator implements WrenchCalculator
{
   private final Wrench wrench = new Wrench();

   private final DMatrixRMaj linearPartOfJacobian = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj angularPartOfJacobian = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj linearJacobianInverse = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj angularJacobianInverse = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj linearSelectionMatrix = CommonOps_DDRM.identity(6);
   private final DMatrixRMaj angularSelectionMatrix = CommonOps_DDRM.identity(6);

   private final DMatrixRMaj footLinearForce = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj footAngularForce = new DMatrixRMaj(3, 1);

   private final DMatrixRMaj jointTorques = new DMatrixRMaj(3, 1);

   private final GeometricJacobian footJacobian;
   private final List<OneDoFJointBasics> joints;
   private final List<JointTorqueProvider> jointTorqueProviders;

   private final String prefix;

   private boolean isTorquingIntoJoint = false;

   public JointTorqueBasedWrenchCalculator(String prefix, FullQuadrupedRobotModel robotModel, RobotQuadrant robotQuadrant, ReferenceFrame soleFrame,
                                           List<JointTorqueProvider> jointTorqueProviders)
   {
      this.prefix = prefix;
      this.jointTorqueProviders = jointTorqueProviders;
      RigidBodyBasics body = robotModel.getRootBody();
      RigidBodyBasics foot = robotModel.getFoot(robotQuadrant);
      footJacobian = new GeometricJacobian(body, foot, soleFrame);

      joints = robotModel.getLegJointsList(robotQuadrant);

      wrench.setToZero(ReferenceFrame.getWorldFrame());

      // remove linear part
      MatrixTools.removeRow(angularSelectionMatrix, 3);
      MatrixTools.removeRow(angularSelectionMatrix, 3);
      MatrixTools.removeRow(angularSelectionMatrix, 3);

      // remove angular part
      MatrixTools.removeRow(linearSelectionMatrix, 0);
      MatrixTools.removeRow(linearSelectionMatrix, 0);
      MatrixTools.removeRow(linearSelectionMatrix, 0);
   }

   @Override
   public void calculate()
   {
      isTorquingIntoJoint = isTorquingIntoJointLimitInternal();

      footJacobian.compute();
      DMatrixRMaj jacobianMatrix = footJacobian.getJacobianMatrix();
      CommonOps_DDRM.mult(linearSelectionMatrix, jacobianMatrix, linearPartOfJacobian);
      CommonOps_DDRM.mult(angularSelectionMatrix, jacobianMatrix, angularPartOfJacobian);
      UnrolledInverseFromMinor_DDRM.inv3(linearPartOfJacobian, linearJacobianInverse, 1.0);
      UnrolledInverseFromMinor_DDRM.inv3(angularPartOfJacobian, angularJacobianInverse, 1.0);

      CommonOps_DDRM.multTransA(-1.0, linearJacobianInverse, jointTorques, footLinearForce);
      CommonOps_DDRM.multTransA(-1.0, angularJacobianInverse, jointTorques, footAngularForce);

      wrench.setToZero(footJacobian.getJacobianFrame());
      wrench.getLinearPart().set(footLinearForce);
      wrench.getAngularPart().set(footAngularForce);
   }

   @Override
   public WrenchReadOnly getWrench()
   {
      return wrench;
   }

   @Override
   public String getName()
   {
      return prefix + "JntTorqWrnchCalc";
   }

   private boolean isTorquingIntoJointLimitInternal()
   {
      for(int i = 0; i < jointTorqueProviders.size(); i++)
      {
         jointTorques.set(i, 0, jointTorqueProviders.get(i).getTorque());
         if (isTorquingIntoJointLimit(joints.get(i), jointTorqueProviders.get(i).getTorque()))
            return true;
      }

      return false;
   }

   private boolean isTorquingIntoJointLimit(OneDoFJointBasics joint, double torque)
   {
      double q = joint.getQ();
      double jointLimitLower = joint.getJointLimitLower();
      double jointLimitUpper = joint.getJointLimitUpper();

      if (q > jointLimitUpper)
         return Math.signum(torque) > 0.0;
      else if (q < jointLimitLower)
         return Math.signum(torque) < 0.0;
      return false;
   }

   @Override
   public boolean isTorquingIntoJointLimit()
   {
      return isTorquingIntoJoint;
   }

   public interface JointTorqueProvider
   {
      double getTorque();
   }


}
