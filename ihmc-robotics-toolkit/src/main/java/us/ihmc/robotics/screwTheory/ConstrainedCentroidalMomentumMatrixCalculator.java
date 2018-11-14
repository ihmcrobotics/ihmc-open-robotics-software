package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.algorithms.CentroidalMomentumCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

/**
 * @author twan
 *         Date: 4/15/13
 */
public class ConstrainedCentroidalMomentumMatrixCalculator
{
   private final DynamicallyConsistentNullspaceCalculator dynamicallyConsistentNullspaceCalculator;
   private final CentroidalMomentumCalculator centroidalMomentumCalculator;
   private final DenseMatrix64F selectionMatrix;
   private final DenseMatrix64F temp = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F constrainedCentroidalMomentumMatrix = new DenseMatrix64F(1, 1);

   public ConstrainedCentroidalMomentumMatrixCalculator(FloatingJointBasics rootJoint, ReferenceFrame centerOfMassFrame,
                                                        DenseMatrix64F selectionMatrix)
   {
      this.dynamicallyConsistentNullspaceCalculator = new OriginalDynamicallyConsistentNullspaceCalculator(rootJoint,
            true);
      this.centroidalMomentumCalculator = new CentroidalMomentumCalculator(rootJoint.getSuccessor(), centerOfMassFrame);
      this.selectionMatrix = selectionMatrix;
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

   public void compute()
   {
      dynamicallyConsistentNullspaceCalculator.compute();
      centroidalMomentumCalculator.reset();
      DenseMatrix64F centroidalMomentumMatrix = this.centroidalMomentumCalculator.getCentroidalMomentumMatrix();
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
      return centroidalMomentumCalculator.getCentroidalMomentumMatrix();
   }
}
