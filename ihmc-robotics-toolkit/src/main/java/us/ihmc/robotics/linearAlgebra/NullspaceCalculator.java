package us.ihmc.robotics.linearAlgebra;

import org.ejml.data.DenseMatrix64F;

public interface NullspaceCalculator
{
   /**
    * Perform a projection in place
    * @param matrixToProjectOntoNullspace the matrix to be projected. Modified.
    * @param matrixToComputeNullspaceOf the matrix to compute the nullspace of for the projection. Not Modified.
    */
   void projectOntoNullspace(DenseMatrix64F matrixToProjectOntoNullspace, DenseMatrix64F matrixToComputeNullspaceOf);
   /**
    * Perform a projection
    * @param matrixToProjectOntoNullspace the matrix to be projected. Modified.
    * @param matrixToComputeNullspaceOf the matrix to compute the nullspace of for the projection. Not Modified.
    */
   void projectOntoNullspace(DenseMatrix64F matrixToProjectOntoNullspace, DenseMatrix64F matrixToComputeNullspaceOf, DenseMatrix64F projectedMatrixToPack);

   void computeNullspaceProjector(DenseMatrix64F matrixToComputeNullspaceOf, DenseMatrix64F nullspaceProjectorToPack);
}
