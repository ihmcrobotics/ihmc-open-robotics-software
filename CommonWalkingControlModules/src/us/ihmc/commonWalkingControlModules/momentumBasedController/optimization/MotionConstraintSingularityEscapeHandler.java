package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.NullspaceCalculator;
import us.ihmc.utilities.screwTheory.SpatialMotionVector;

public class MotionConstraintSingularityEscapeHandler
{
   private final DenseMatrix64F iMinusNNT = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F sJ = new DenseMatrix64F(1, 1);
   private final NullspaceCalculator nullspaceCalculator = new NullspaceCalculator(SpatialMotionVector.SIZE, true);
   private static final boolean DEBUG = false;

   public MotionConstraintSingularityEscapeHandler()
   {
   }

   public void computeConstraintBlocksForSingularityEscape(DenseMatrix64F selectionMatrix, DenseMatrix64F nullspaceMultipliers,
           DenseMatrix64F baseToEndEffectorJacobianMatrix, DenseMatrix64F jacobianMatrix, DenseMatrix64F zBlock, DenseMatrix64F nCompactBlock,
           DenseMatrix64F jBlockCompact)
   {
      sJ.reshape(selectionMatrix.getNumRows(), jacobianMatrix.getNumCols());
      CommonOps.mult(selectionMatrix, jacobianMatrix, sJ);
      nullspaceCalculator.setMatrix(sJ, nullspaceMultipliers.getNumRows());
      DenseMatrix64F nullspace = nullspaceCalculator.getNullspace();

      DenseMatrix64F iMinusNNT = computeIMinusNNT(nullspace);
      CommonOps.mult(iMinusNNT, baseToEndEffectorJacobianMatrix, jBlockCompact);

      nCompactBlock.reshape(nullspace.getNumCols(), nullspace.getNumRows());
      CommonOps.transpose(nullspace, nCompactBlock);

      zBlock.set(nullspaceMultipliers);
      
      if (DEBUG )
      {
         System.out.println("jBlockCompact = " + jBlockCompact);
         System.out.println("nCompactBlock = " + nCompactBlock);
         System.out.println("zBlock = " + zBlock);
         
         DenseMatrix64F jBlockCompactUnaltered = new DenseMatrix64F(selectionMatrix.getNumRows(), baseToEndEffectorJacobianMatrix.getNumCols());
         CommonOps.mult(selectionMatrix, baseToEndEffectorJacobianMatrix, jBlockCompact);
         System.out.println("jBlockCompactUnaltered = " + jBlockCompactUnaltered);
      }
   }

   private DenseMatrix64F computeIMinusNNT(DenseMatrix64F nullspace)
   {
      iMinusNNT.reshape(nullspace.getNumRows(), nullspace.getNumRows());
      CommonOps.multOuter(nullspace, iMinusNNT);
      CommonOps.scale(-1.0, iMinusNNT);
      MatrixTools.addDiagonal(iMinusNNT, 1.0);

      return iMinusNNT;
   }
}
