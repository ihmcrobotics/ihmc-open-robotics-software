package us.ihmc.robotics.linearAlgebra;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.functionApproximation.DampedLeastSquaresSolver;
import us.ihmc.robotics.linearAlgebra.DampedNullspaceCalculator;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class DampedLeastSquaresNullspaceCalculator implements DampedNullspaceCalculator
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
   public void computeNullspaceProjector(DenseMatrix64F matrixToComputeNullspaceOf, DenseMatrix64F nullspaceProjectorToPack)
   {
      int nullspaceMatrixSize = matrixToComputeNullspaceOf.getNumCols();

      pseudoInverseMatrix.reshape(nullspaceMatrixSize, matrixToComputeNullspaceOf.getNumRows());
      pseudoInverseSolver.setA(matrixToComputeNullspaceOf);
      pseudoInverseSolver.invert(pseudoInverseMatrix);

      // I - J^* J
      CommonOps.mult(-1.0, pseudoInverseMatrix, matrixToComputeNullspaceOf, nullspaceProjectorToPack);
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
   @Override
   public void projectOntoNullspace(DenseMatrix64F matrixToProjectOntoNullspace, DenseMatrix64F matrixToComputeNullspaceOf, DenseMatrix64F projectedMatrixToPack)
   {
      nullspaceProjector.reshape(matrixToComputeNullspaceOf.getNumCols(), matrixToProjectOntoNullspace.getNumCols());
      computeNullspaceProjector(matrixToComputeNullspaceOf, nullspaceProjector);
      projectedMatrixToPack.reshape(matrixToProjectOntoNullspace.getNumRows(), matrixToProjectOntoNullspace.getNumCols());
      CommonOps.mult(matrixToProjectOntoNullspace, nullspaceProjector, projectedMatrixToPack);
   }
}
