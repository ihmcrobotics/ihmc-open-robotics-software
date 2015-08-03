package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.linearAlgebra.NullspaceCalculator;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;

public class MotionConstraintSingularityEscapeHandler
{
   private final DenseMatrix64F iMinusNNT = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F sJ = new DenseMatrix64F(1, 1);
   private final NullspaceCalculator nullspaceCalculator = new NullspaceCalculator(SpatialMotionVector.SIZE, true);
   private static final boolean DEBUG = false;

   private final boolean DO_IMinusNNTStuff = false;
   
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

      if (DEBUG)
      {
         System.out.println("selectionMatrix = " + selectionMatrix);
         System.out.println("jacobianMatrix = " + jacobianMatrix);
         System.out.println("sJ = " + sJ);
         System.out.println("nullspace = " + sJ);
      }
      
      if (DO_IMinusNNTStuff)
      {
         DenseMatrix64F iMinusNNT = computeIMinusNNT(nullspace);
         CommonOps.mult(iMinusNNT, baseToEndEffectorJacobianMatrix, jBlockCompact);
      }
      else
      {
         jBlockCompact.set(baseToEndEffectorJacobianMatrix);
      }


      nCompactBlock.reshape(nullspace.getNumCols(), nullspace.getNumRows());
      CommonOps.transpose(nullspace, nCompactBlock);

      zBlock.set(nullspaceMultipliers);
      
      if (DEBUG)
      {
         System.out.println("iMinusNNT = " + iMinusNNT);
         System.out.println("jBlockCompact = " + jBlockCompact);
         System.out.println("nCompactBlock = " + nCompactBlock);
         System.out.println("zBlock = " + zBlock);
         
         DenseMatrix64F jBlockCompactUnaltered = new DenseMatrix64F(selectionMatrix.getNumRows(), baseToEndEffectorJacobianMatrix.getNumCols());
         CommonOps.mult(selectionMatrix, baseToEndEffectorJacobianMatrix, jBlockCompactUnaltered);
         System.out.println("jBlockCompactUnaltered = " + jBlockCompactUnaltered);
         
         // jBlockCompact times nCompactBlock should be zero:
         DenseMatrix64F jBlockEnd = CommonOps.extract(jBlockCompact, 0, jBlockCompact.getNumRows(), jBlockCompact.getNumCols() - nCompactBlock.getNumCols(), jBlockCompact.getNumCols());
         
         DenseMatrix64F jBlockTimesNullspaceBlock = new DenseMatrix64F(jBlockEnd.getNumRows(), nCompactBlock.getNumRows());
         CommonOps.multTransB(jBlockEnd, nCompactBlock, jBlockTimesNullspaceBlock);
         
         System.out.println("jBlockTimesNullspaceBlock (should be zero) = " + jBlockTimesNullspaceBlock);
         
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
