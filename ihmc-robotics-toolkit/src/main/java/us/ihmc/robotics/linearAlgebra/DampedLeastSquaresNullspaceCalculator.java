package us.ihmc.robotics.linearAlgebra;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.functionApproximation.DampedLeastSquaresSolver;

public class DampedLeastSquaresNullspaceCalculator implements DampedNullspaceCalculator
{
   private final DampedLeastSquaresSolver pseudoInverseSolver;

   private final DMatrixRMaj nullspaceProjector;
   private final DMatrixRMaj tempMatrixForProjectionInPlace;

   public DampedLeastSquaresNullspaceCalculator(int matrixSize, double alpha)
   {
      pseudoInverseSolver = new DampedLeastSquaresSolver(matrixSize, alpha);
      nullspaceProjector = new DMatrixRMaj(matrixSize, matrixSize);
      tempMatrixForProjectionInPlace = new DMatrixRMaj(matrixSize, matrixSize);
   }

   @Override
   public void setPseudoInverseAlpha(double alpha)
   {
      pseudoInverseSolver.setAlpha(alpha);
   }

   /**
    * Compute the nullspace projector of the given matrix as follows:
    * <p>
    * &Nu; = I - M<sup>+</sup>M
    * </p>
    * Where M<sup>+</sup> is the Moore-Penrose pseudo-inverse of M.
    * A damped least-squares solver is used to compute M<sup>+</sup>.
    * @param matrixToComputeNullspaceOf the matrix to compute the nullspace of for the projection. Not Modified.
    * @param nullspaceProjectorToPack matrix to store the resulting nullspace matrix. Modified.
    */
   @Override
   public void computeNullspaceProjector(DMatrixRMaj matrixToComputeNullspaceOf, DMatrixRMaj nullspaceProjectorToPack)
   {
      pseudoInverseSolver.setA(matrixToComputeNullspaceOf);
      pseudoInverseSolver.solve(matrixToComputeNullspaceOf, nullspaceProjectorToPack);

      // I - J^* J
      CommonOps_DDRM.scale(-1.0, nullspaceProjectorToPack);
      MatrixTools.addDiagonal(nullspaceProjectorToPack, 1.0);
   }

   /**
    * Perform a projection in place as follows:
    * <p>
    * A = A * (I - B<sup>+</sup>B)
    * </p> 
    * @param matrixToProjectOntoNullspace the matrix to be projected, A in the equation. Modified.
    * @param matrixToComputeNullspaceOf the matrix to compute the nullspace of for the projection, B in the equation. Not Modified.
    */
   @Override
   public void projectOntoNullspace(DMatrixRMaj matrixToProjectOntoNullspace, DMatrixRMaj matrixToComputeNullspaceOf)
   {
      tempMatrixForProjectionInPlace.set(matrixToProjectOntoNullspace);
      projectOntoNullspace(tempMatrixForProjectionInPlace, matrixToComputeNullspaceOf, matrixToProjectOntoNullspace);
   }

   /**
    * Project {@code matrixToProjectOntoNullspace} onto the nullspace of {@code matrixToComputeNullspaceOf} as follows:
    * <p>
    * C = A * (I - B<sup>+</sup>B)
    * </p> 
    * @param matrixToProjectOntoNullspace the matrix to be projected, A in the equation. Not modified.
    * @param matrixToComputeNullspaceOf the matrix to compute the nullspace of for the projection, B in the equation. Not Modified.
    * @param projectedMatrixToPack matrix to store the resulting projection, C in the equation. Modified.
    */
   @Override
   public void projectOntoNullspace(DMatrixRMaj matrixToProjectOntoNullspace, DMatrixRMaj matrixToComputeNullspaceOf, DMatrixRMaj projectedMatrixToPack)
   {
      int nullSize = matrixToComputeNullspaceOf.getNumCols();
      nullspaceProjector.reshape(nullSize, nullSize);
      projectedMatrixToPack.reshape(matrixToProjectOntoNullspace.getNumRows(), nullSize);

      computeNullspaceProjector(matrixToComputeNullspaceOf, nullspaceProjector);
      CommonOps_DDRM.mult(matrixToProjectOntoNullspace, nullspaceProjector, projectedMatrixToPack);
   }
}
