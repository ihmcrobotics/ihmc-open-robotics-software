package us.ihmc.robotics.linearAlgebra;

import org.ejml.data.DMatrixRMaj;

public interface NullspaceCalculator
{
   /**
    * Perform a projection in place
    * @param matrixToProjectOntoNullspace the matrix to be projected. Modified.
    * @param matrixToComputeNullspaceOf the matrix to compute the nullspace of for the projection. Not Modified.
    */
   void projectOntoNullspace(DMatrixRMaj matrixToProjectOntoNullspace, DMatrixRMaj matrixToComputeNullspaceOf);
   /**
    * Perform a projection
    * @param matrixToProjectOntoNullspace the matrix to be projected. Modified.
    * @param matrixToComputeNullspaceOf the matrix to compute the nullspace of for the projection. Not Modified.
    */
   void projectOntoNullspace(DMatrixRMaj matrixToProjectOntoNullspace, DMatrixRMaj matrixToComputeNullspaceOf, DMatrixRMaj projectedMatrixToPack);

   void computeNullspaceProjector(DMatrixRMaj matrixToComputeNullspaceOf, DMatrixRMaj nullspaceProjectorToPack);
}
