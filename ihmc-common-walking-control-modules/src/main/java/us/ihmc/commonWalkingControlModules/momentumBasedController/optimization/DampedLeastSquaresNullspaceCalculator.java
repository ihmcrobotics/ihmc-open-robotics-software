package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.functionApproximation.DampedLeastSquaresSolver;

public class DampedLeastSquaresNullspaceCalculator
{
   private final DampedLeastSquaresSolver pseudoInverseSolver;

   private final DenseMatrix64F pseudoInverseMatrix;
   private final DenseMatrix64F nullspaceProjector;
   private final DenseMatrix64F tempMatrixForProjectionInPlace;

   public DampedLeastSquaresNullspaceCalculator(int matrixSize, double alpha)
   {
      pseudoInverseSolver = new DampedLeastSquaresSolver(matrixSize, alpha);
      pseudoInverseMatrix = new DenseMatrix64F(matrixSize, matrixSize);
      nullspaceProjector = new DenseMatrix64F(matrixSize, matrixSize);
      tempMatrixForProjectionInPlace = new DenseMatrix64F(matrixSize, matrixSize);
   }

   public void setPseudoInverseAlpha(double alpha)
   {
      pseudoInverseSolver.setAlpha(alpha);
   }

   /**
    * Compute the nullspace of the given matrix as follows:
    * <p>
    * &Nu; = I - M<sup>+</sup>M
    * </p>
    * Where M<sup>+</sup> is the Moore-Penrose pseudo-inverse of M.
    * A damped least-squares solver is used to compute M<sup>+</sup>.
    * @param matrixToComputeNullspaceOf the matrix to compute the nullspace of for the projection. Not Modified.
    * @param nullspaceToPack matrix to store the resulting nullspace matrix. Modified.
    */
   public void computeNullspace(DenseMatrix64F matrixToComputeNullspaceOf, DenseMatrix64F nullspaceToPack)
   {
      int nullspaceMatrixSize = matrixToComputeNullspaceOf.getNumCols();
      nullspaceToPack.reshape(nullspaceMatrixSize, nullspaceMatrixSize);

      pseudoInverseMatrix.reshape(nullspaceMatrixSize, matrixToComputeNullspaceOf.getNumRows());
      pseudoInverseSolver.setA(matrixToComputeNullspaceOf);
      pseudoInverseSolver.invert(pseudoInverseMatrix);

      // I - J^* J
      CommonOps.mult(-1.0, pseudoInverseMatrix, matrixToComputeNullspaceOf, nullspaceToPack);
      for (int i = 0; i < nullspaceMatrixSize; i++)
         nullspaceToPack.add(i, i, 1.0);
   }

   /**
    * Perform a projection in place as follows:
    * <p>
    * A = A * (I - B<sup>+</sup>B)
    * </p> 
    * @param matrixToProjectOntoNullspace the matrix to be projected, A in the equation. Modified.
    * @param matrixToComputeNullspaceOf the matrix to compute the nullspace of for the projection, B in the equation. Not Modified.
    */
   public void projectOntoNullspace(DenseMatrix64F matrixToProjectOntoNullspace, DenseMatrix64F matrixToComputeNullspaceOf)
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
   public void projectOntoNullspace(DenseMatrix64F matrixToProjectOntoNullspace, DenseMatrix64F matrixToComputeNullspaceOf, DenseMatrix64F projectedMatrixToPack)
   {
      computeNullspace(matrixToComputeNullspaceOf, nullspaceProjector);
      projectedMatrixToPack.reshape(matrixToProjectOntoNullspace.getNumRows(), matrixToProjectOntoNullspace.getNumCols());
      CommonOps.mult(matrixToProjectOntoNullspace, nullspaceProjector, projectedMatrixToPack);
   }
}
