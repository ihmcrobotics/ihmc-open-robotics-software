package us.ihmc.robotics.linearAlgebra;

import org.ejml.data.DMatrix1Row;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.decomposition.QRDecomposition;
import org.ejml.interfaces.linsol.LinearSolverDense;

import us.ihmc.commons.MathTools;
import us.ihmc.matrixlib.MatrixTools;

public class DampedQRNullspaceCalculator implements DampedNullspaceCalculator
{
   private final QRDecomposition<DMatrixRMaj> decomposer;
   private final LinearSolverDense<DMatrixRMaj> linearSolver;

   private final DMatrixRMaj nullspace;
   private final DMatrixRMaj Q;
   private final DMatrixRMaj R1;

   private final DMatrixRMaj nullspaceProjector;
   private final DMatrixRMaj tempMatrixForProjectionInPlace;

   private double alpha = 0.0;

   public DampedQRNullspaceCalculator(int matrixSize, double alpha)
   {
      this.alpha = alpha;
      MathTools.checkIntervalContains(matrixSize, 1, Integer.MAX_VALUE);

      nullspaceProjector = new DMatrixRMaj(matrixSize, matrixSize);
      tempMatrixForProjectionInPlace = new DMatrixRMaj(matrixSize, matrixSize);

      linearSolver = LinearSolverFactory_DDRM.symmPosDef(matrixSize);

      decomposer = DecompositionFactory_DDRM.qr(matrixSize, matrixSize);
      nullspace = new DMatrixRMaj(matrixSize, matrixSize);
      Q = new DMatrixRMaj(matrixSize, matrixSize);
      R1 = new DMatrixRMaj(matrixSize, matrixSize);
   }

   @Override
   public void setPseudoInverseAlpha(double alpha)
   {
      this.alpha = alpha;
   }


   /**
    * Perform a projection in place as follows:
    * <p>
    * N = nullspace(B)
    * </p>
    * <p>
    * A = A * NN<sup>T</sup>)
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
    * N = nullspace(B)
    * </p>
    * <p>
    * C = A * NN<sup>T</sup>)
    * </p>
    * @param matrixToProjectOntoNullspace the matrix to be projected, A in the equation. Not modified.
    * @param matrixToComputeNullspaceOf the matrix to compute the nullspace of for the projection, B in the equation. Not Modified.
    * @param projectedMatrixToPack matrix to store the resulting projection, C in the equation. Modified.
    */
   @Override
   public void projectOntoNullspace(DMatrixRMaj matrixToProjectOntoNullspace, DMatrixRMaj matrixToComputeNullspaceOf, DMatrixRMaj projectedMatrixToPack)
   {
      computeNullspaceProjector(matrixToComputeNullspaceOf, nullspaceProjector);
      CommonOps_DDRM.mult(matrixToProjectOntoNullspace, nullspaceProjector, projectedMatrixToPack);
   }

   /**
    * Compute the nullspace projector of the given matrix as follows:
    * <p>
    * &Nu; = NN<sup>T</sup>
    * </p>
    * Where N<sup>+</sup> is the nullspace of {@param matrixToComputeNullspaceOf}.
    * A svd decomposition solver is used to compute N.
    * @param matrixToComputeNullspaceOf the matrix to compute the nullspace of for the projection. Not Modified.
    * @param nullspaceProjectorToPack matrix to store the resulting nullspace matrix. Modified.
    */
   @Override
   public void computeNullspaceProjector(DMatrixRMaj matrixToComputeNullspaceOf, DMatrixRMaj nullspaceProjectorToPack)
   {
      int nullity = Math.max(matrixToComputeNullspaceOf.getNumCols() - matrixToComputeNullspaceOf.getNumRows(), 0);
      nullspaceProjectorToPack.reshape(matrixToComputeNullspaceOf.getNumCols(), matrixToComputeNullspaceOf.getNumCols());

      if (alpha == 0.0)
      {
         computeNullspace(nullspace, matrixToComputeNullspaceOf, nullity);
         CommonOps_DDRM.multOuter(nullspace, nullspaceProjectorToPack);
      }
      else
      {
         int size = matrixToComputeNullspaceOf.getNumCols();
         int vars = matrixToComputeNullspaceOf.getNumRows();
         int rank = Math.min(size, vars);

         transposed.reshape(size, vars);

         R1.reshape(rank, vars);
         squared.reshape(vars, vars);
         inverse.reshape(vars, vars);

         CommonOps_DDRM.transpose(matrixToComputeNullspaceOf, transposed);
         decomposer.decompose(transposed);

         decomposer.getR(R1, true);

         if (R1.getNumCols() == R1.getNumRows())
            inner_small_upper_diagonal(R1, squared);
         else
            CommonOps_DDRM.multInner(R1, squared);
         MatrixTools.addDiagonal(squared, alpha * alpha);

         linearSolver.setA(squared);
         linearSolver.invert(inverse);

         tempMatrix.reshape(size, vars);
         CommonOps_DDRM.multTransA(matrixToComputeNullspaceOf, inverse, tempMatrix);
         CommonOps_DDRM.mult(-1.0, tempMatrix, matrixToComputeNullspaceOf, nullspaceProjectorToPack);
         MatrixTools.addDiagonal(nullspaceProjectorToPack, 1.0);
      }
   }

   private final DMatrixRMaj tempMatrix = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj squared = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj inverse = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj transposed = new DMatrixRMaj(0, 0);
   private void computeNullspace(DMatrixRMaj nullspaceToPack, DMatrixRMaj matrixToComputeNullspaceOf, int nullity)
   {
      int size = matrixToComputeNullspaceOf.getNumCols();
      int rank = matrixToComputeNullspaceOf.getNumRows();
      nullspaceToPack.reshape(size, nullity);
      Q.reshape(size, size);
      transposed.reshape(size, rank);

      CommonOps_DDRM.transpose(matrixToComputeNullspaceOf, transposed);
      decomposer.decompose(transposed);
      decomposer.getQ(Q, false);

      CommonOps_DDRM.extract(Q, 0, Q.getNumRows(), Q.getNumCols() - nullity, Q.getNumCols(), nullspaceToPack, 0, 0);
   }

   static void inner_small_upper_diagonal(DMatrix1Row a, DMatrix1Row c) {

      for( int transposeRowIndex = 0; transposeRowIndex < a.numCols; transposeRowIndex++ )
      {
         for (int colIndex = transposeRowIndex; colIndex < a.numCols; colIndex++)
         {
            double sum = 0.0;
            int indexA = transposeRowIndex;
            int indexB = colIndex;

            int indexC1 = colIndex * a.numCols + transposeRowIndex;
            int indexC2 = transposeRowIndex * a.numCols + colIndex;

            for (int row = 0; row <= transposeRowIndex; row++, indexA += a.numCols, indexB += a.numCols)
            {
               sum += a.data[indexA] * a.data[indexB];
            }

            c.data[indexC1] = c.data[indexC2] = sum;
         }
      }
   }
}
