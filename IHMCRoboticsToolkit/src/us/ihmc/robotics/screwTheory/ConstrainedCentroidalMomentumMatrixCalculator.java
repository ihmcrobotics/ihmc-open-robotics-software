package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * @author twan
 *         Date: 4/15/13
 */
public class ConstrainedCentroidalMomentumMatrixCalculator
{
   private final DynamicallyConsistentNullspaceCalculator dynamicallyConsistentNullspaceCalculator;
   private final CentroidalMomentumMatrix centroidalMomentumMatrix;
   private final DenseMatrix64F selectionMatrix;
   private final DenseMatrix64F temp = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F constrainedCentroidalMomentumMatrix = new DenseMatrix64F(1, 1);

   public ConstrainedCentroidalMomentumMatrixCalculator(FloatingInverseDynamicsJoint rootJoint, ReferenceFrame centerOfMassFrame,
                                                        DenseMatrix64F selectionMatrix)
   {
      this.dynamicallyConsistentNullspaceCalculator = new OriginalDynamicallyConsistentNullspaceCalculator(rootJoint,
            true);
      this.centroidalMomentumMatrix = new CentroidalMomentumMatrix(rootJoint.getSuccessor(), centerOfMassFrame);
      this.selectionMatrix = selectionMatrix;
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

   public void compute()
   {
      dynamicallyConsistentNullspaceCalculator.compute();
      centroidalMomentumMatrix.compute();
      DenseMatrix64F centroidalMomentumMatrix = this.centroidalMomentumMatrix.getMatrix();
      DenseMatrix64F sNsBar = dynamicallyConsistentNullspaceCalculator.getSNsBar();

      temp.reshape(centroidalMomentumMatrix.getNumRows(), sNsBar.getNumCols());
      CommonOps.mult(centroidalMomentumMatrix, sNsBar, temp);

      constrainedCentroidalMomentumMatrix.reshape(selectionMatrix.getNumRows(), temp.getNumCols());
      CommonOps.mult(selectionMatrix, temp, constrainedCentroidalMomentumMatrix);
   }

   public DenseMatrix64F getConstrainedCentroidalMomentumMatrix()
   {
      return constrainedCentroidalMomentumMatrix;
   }

   public DenseMatrix64F getCentroidalMomentumMatrix()
   {
      return centroidalMomentumMatrix.getMatrix();
   }
}
