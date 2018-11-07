package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

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
   private final DenseMatrix64F constrainedCenterOfMassJacobian = new DenseMatrix64F(1, 1);

   public ConstrainedCenterOfMassJacobianCalculator(FloatingJointBasics rootJoint)
   {
      this.dynamicallyConsistentNullspaceCalculator = new OriginalDynamicallyConsistentNullspaceCalculator(rootJoint, true);
      this.centerOfMassJacobian = new CenterOfMassJacobian(rootJoint.getSuccessor(), rootJoint.getSuccessor().getBodyFixedFrame());
   }

   public void compute()
   {
      dynamicallyConsistentNullspaceCalculator.compute();
      centerOfMassJacobian.reset();
      DenseMatrix64F centerOfMassJacobianMatrix = centerOfMassJacobian.getJacobianMatrix();
      DenseMatrix64F sNsBar = dynamicallyConsistentNullspaceCalculator.getSNsBar();

      constrainedCenterOfMassJacobian.reshape(centerOfMassJacobianMatrix.getNumRows(), sNsBar.getNumCols());
      CommonOps.mult(centerOfMassJacobianMatrix, sNsBar, constrainedCenterOfMassJacobian);
   }

   public void reset()
   {
      dynamicallyConsistentNullspaceCalculator.reset();
   }

   public void addConstraint(RigidBodyBasics body, DenseMatrix64F selectionMatrix)
   {
      dynamicallyConsistentNullspaceCalculator.addConstraint(body, selectionMatrix);
   }

   public void addActuatedJoint(JointBasics joint)
   {
      dynamicallyConsistentNullspaceCalculator.addActuatedJoint(joint);
   }

   public DenseMatrix64F getConstrainedCenterOfMassJacobian()
   {
      return constrainedCenterOfMassJacobian;
   }

   public DenseMatrix64F getCenterOfMassJacobian()
   {
      return centerOfMassJacobian.getJacobianMatrix();
   }
}
