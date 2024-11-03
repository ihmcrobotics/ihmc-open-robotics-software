package us.ihmc.robotics.screwTheory;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

/**
 * @author twan
 *         Date: 4/15/13
 */
public class ConstrainedCenterOfMassJacobianCalculator
{
   private final DynamicallyConsistentNullspaceCalculator dynamicallyConsistentNullspaceCalculator;
   private final CenterOfMassJacobian centerOfMassJacobian;
   private final DMatrixRMaj constrainedCenterOfMassJacobian = new DMatrixRMaj(1, 1);

   public ConstrainedCenterOfMassJacobianCalculator(FloatingJointBasics rootJoint)
   {
      this.dynamicallyConsistentNullspaceCalculator = new OriginalDynamicallyConsistentNullspaceCalculator(rootJoint, true);
      this.centerOfMassJacobian = new CenterOfMassJacobian(rootJoint.getSuccessor(), rootJoint.getSuccessor().getBodyFixedFrame());
   }

   public void compute()
   {
      dynamicallyConsistentNullspaceCalculator.compute();
      centerOfMassJacobian.reset();
      DMatrixRMaj centerOfMassJacobianMatrix = centerOfMassJacobian.getJacobianMatrix();
      DMatrixRMaj sNsBar = dynamicallyConsistentNullspaceCalculator.getSNsBar();

      constrainedCenterOfMassJacobian.reshape(centerOfMassJacobianMatrix.getNumRows(), sNsBar.getNumCols());
      CommonOps_DDRM.mult(centerOfMassJacobianMatrix, sNsBar, constrainedCenterOfMassJacobian);
   }

   public void reset()
   {
      dynamicallyConsistentNullspaceCalculator.reset();
   }

   public void addConstraint(RigidBodyBasics body, DMatrixRMaj selectionMatrix)
   {
      dynamicallyConsistentNullspaceCalculator.addConstraint(body, selectionMatrix);
   }

   public void addActuatedJoint(JointBasics joint)
   {
      dynamicallyConsistentNullspaceCalculator.addActuatedJoint(joint);
   }

   public DMatrixRMaj getConstrainedCenterOfMassJacobian()
   {
      return constrainedCenterOfMassJacobian;
   }

   public DMatrixRMaj getCenterOfMassJacobian()
   {
      return centerOfMassJacobian.getJacobianMatrix();
   }
}
