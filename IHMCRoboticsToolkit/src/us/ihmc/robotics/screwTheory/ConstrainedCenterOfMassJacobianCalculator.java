package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

/**
 * @author twan
 *         Date: 4/15/13
 */
public class ConstrainedCenterOfMassJacobianCalculator
{
   private final DynamicallyConsistentNullspaceCalculator dynamicallyConsistentNullspaceCalculator;
   private final CenterOfMassJacobian centerOfMassJacobian;
   private final DenseMatrix64F constrainedCenterOfMassJacobian = new DenseMatrix64F(1, 1);

   public ConstrainedCenterOfMassJacobianCalculator(SixDoFJoint rootJoint)
   {
      this.dynamicallyConsistentNullspaceCalculator = new OriginalDynamicallyConsistentNullspaceCalculator(rootJoint, true);
      this.centerOfMassJacobian = new CenterOfMassJacobian(rootJoint.getSuccessor());
   }

   public void compute()
   {
      dynamicallyConsistentNullspaceCalculator.compute();
      centerOfMassJacobian.compute();
      DenseMatrix64F centerOfMassJacobianMatrix = centerOfMassJacobian.getMatrix();
      DenseMatrix64F sNsBar = dynamicallyConsistentNullspaceCalculator.getSNsBar();

      constrainedCenterOfMassJacobian.reshape(centerOfMassJacobianMatrix.getNumRows(), sNsBar.getNumCols());
      CommonOps.mult(centerOfMassJacobianMatrix, sNsBar, constrainedCenterOfMassJacobian);
   }

   public void reset()
   {
      dynamicallyConsistentNullspaceCalculator.reset();
   }

   public void addConstraint(RigidBody body, DenseMatrix64F selectionMatrix)
   {
      dynamicallyConsistentNullspaceCalculator.addConstraint(body, selectionMatrix);
   }

   public void addActuatedJoint(InverseDynamicsJoint joint)
   {
      dynamicallyConsistentNullspaceCalculator.addActuatedJoint(joint);
   }

   public DenseMatrix64F getConstrainedCenterOfMassJacobian()
   {
      return constrainedCenterOfMassJacobian;
   }

   public DenseMatrix64F getCenterOfMassJacobian()
   {
      return centerOfMassJacobian.getMatrix();
   }
}
