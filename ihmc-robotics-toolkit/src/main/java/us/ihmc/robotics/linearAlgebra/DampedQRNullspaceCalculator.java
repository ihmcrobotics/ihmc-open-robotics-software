package us.ihmc.robotics.linearAlgebra;

import org.ejml.EjmlParameters;
import org.ejml.alg.dense.mult.MatrixMultProduct;
import org.ejml.data.DenseMatrix64F;
import org.ejml.data.RowD1Matrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.decomposition.QRDecomposition;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixDimensionException;
import us.ihmc.commons.MathTools;

public class DampedQRNullspaceCalculator implements DampedNullspaceCalculator
{
   private final QRDecomposition<DenseMatrix64F> decomposer;
   private final LinearSolver<DenseMatrix64F> linearSolver;

   private final DenseMatrix64F nullspace;
   private final DenseMatrix64F Q;
   private final DenseMatrix64F R;

   private final DenseMatrix64F Q1;
   private final DenseMatrix64F R1;

   private final DenseMatrix64F nullspaceProjector;
   private final DenseMatrix64F tempMatrixForProjectionInPlace;

   private double alpha = 0.0;

   public DampedQRNullspaceCalculator(int matrixSize, double alpha)
   {
      this.alpha = alpha;
      MathTools.checkIntervalContains(matrixSize, 1, Integer.MAX_VALUE);

      nullspaceProjector = new DenseMatrix64F(matrixSize, matrixSize);
      tempMatrixForProjectionInPlace = new DenseMatrix64F(matrixSize, matrixSize);

      linearSolver = LinearSolverFactory.linear(matrixSize);

      decomposer = DecompositionFactory.qr(matrixSize, matrixSize);
      nullspace = new DenseMatrix64F(matrixSize, matrixSize);
      Q = new DenseMatrix64F(matrixSize, matrixSize);
      R = new DenseMatrix64F(matrixSize, matrixSize);
      Q1 = new DenseMatrix64F(matrixSize, matrixSize);
      R1 = new DenseMatrix64F(matrixSize, matrixSize);
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
   public void projectOntoNullspace(DenseMatrix64F matrixToProjectOntoNullspace, DenseMatrix64F matrixToComputeNullspaceOf)
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
   public void projectOntoNullspace(DenseMatrix64F matrixToProjectOntoNullspace, DenseMatrix64F matrixToComputeNullspaceOf, DenseMatrix64F projectedMatrixToPack)
   {
      computeNullspaceProjector(matrixToComputeNullspaceOf, nullspaceProjector);
      CommonOps.mult(matrixToProjectOntoNullspace, nullspaceProjector, projectedMatrixToPack);
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
   public void computeNullspaceProjector(DenseMatrix64F matrixToComputeNullspaceOf, DenseMatrix64F nullspaceProjectorToPack)
   {
      int nullity = matrixToComputeNullspaceOf.getNumCols() - matrixToComputeNullspaceOf.getNumRows();

      nullspaceProjectorToPack.reshape(matrixToComputeNullspaceOf.getNumCols(), matrixToComputeNullspaceOf.getNumCols());

      if (alpha == 0.0)
      {
         computeNullspace(nullspace, matrixToComputeNullspaceOf, nullity);
         CommonOps.multOuter(nullspace, nullspaceProjectorToPack);
      }
      else
      {
         int size = matrixToComputeNullspaceOf.getNumCols();
         int rank = matrixToComputeNullspaceOf.getNumRows();
         R.reshape(size, rank);
         transposed.reshape(size, rank);

         R1.reshape(rank, rank);
         squared.reshape(rank, rank);
         inverse.reshape(rank, rank);

         CommonOps.transpose(matrixToComputeNullspaceOf, transposed);
         decomposer.decompose(transposed);
         decomposer.getR(R, false);

         CommonOps.extract(R, 0, rank, 0, rank, R1, 0, 0);

         transposed.reshape(rank, rank);
         CommonOps.transpose(R1, transposed);
         multiplyLowerDiagonalByUpperDiagonal(R1, squared);
         MatrixTools.addDiagonal(squared, alpha * alpha);
         linearSolver.setA(squared);
         linearSolver.invert(inverse);

         tempMatrix.reshape(size, rank);
         CommonOps.multTransA(matrixToComputeNullspaceOf, inverse, tempMatrix);
         CommonOps.mult(-1.0, tempMatrix, matrixToComputeNullspaceOf, nullspaceProjectorToPack);
         MatrixTools.addDiagonal(nullspaceProjectorToPack, 1.0);
      }
   }

   private final DenseMatrix64F tempMatrix = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F squared = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F inverse = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F transposed = new DenseMatrix64F(0, 0);
   private void computeNullspace(DenseMatrix64F nullspaceToPack, DenseMatrix64F matrixToComputeNullspaceOf, int nullity)
   {
      int size = matrixToComputeNullspaceOf.getNumCols();
      int rank = matrixToComputeNullspaceOf.getNumRows();
      nullspaceToPack.reshape(size, nullity);
      Q.reshape(size, size);
      R.reshape(size, rank);
      transposed.reshape(size, rank);

      CommonOps.transpose(matrixToComputeNullspaceOf, transposed);
      decomposer.decompose(transposed);
      decomposer.getQ(Q, false);

      CommonOps.extract(Q, 0, Q.getNumRows(), Q.getNumCols() - nullity, Q.getNumCols(), nullspaceToPack, 0, 0);
   }

   private static void multiplyLowerDiagonalByUpperDiagonal( RowD1Matrix64F a , RowD1Matrix64F c)
   {

      /*
      if( a.numCols >= EjmlParameters.MULT_COLUMN_SWITCH ) {
         inner_reorder_upper_diagonal( a, c);
      } else {
         */
         //inner_small( a, c);
      inner_small_upper_diagonal( a, c);
      //}
   }

   // TODO make this work for diagonal matrices
   public static void inner_small(RowD1Matrix64F a, RowD1Matrix64F c) {

      for( int i = 0; i < a.numCols; i++ )
      {
         for( int j = i; j < a.numCols; j++ )
         {
            int indexC1 = i*c.numCols+j;
            int indexC2 = j*c.numCols+i;
            int indexA = i;
            int indexB = j;
            double sum = 0;
            int end = indexA + a.numRows*a.numCols;
            for( ; indexA < end; indexA += a.numCols, indexB += a.numCols )
            {
               /*
               if (indexA < i)
                  continue;
                  */
               sum += a.data[indexA]*a.data[indexB];
            }
            c.data[indexC1] = c.data[indexC2] = sum;
         }
      }
   }

   static void inner_small_upper_diagonal(RowD1Matrix64F a, RowD1Matrix64F c) {

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

   // TODO make this work for diagonal matrices
   public static void inner_reorder_upper_diagonal(RowD1Matrix64F a, RowD1Matrix64F c) {

      for( int i = 0; i < a.numCols; i++ ) {
         int indexC = i*c.numCols+i;
         double valAi = a.data[i];
         for( int j = i; j < a.numCols; j++ ) {
            c.data[indexC++] =  valAi*a.data[j];
         }

         for( int k = 1; k < a.numRows; k++ ) {
            indexC = i*c.numCols+i;
            int indexB = k*a.numCols+i;
            valAi = a.data[indexB];
            for( int j = i; j < a.numCols; j++ ) {
               c.data[indexC++] +=  valAi*a.data[indexB++];
            }
         }

         indexC = i*c.numCols+i;
         int indexC2 = indexC;
         for( int j = i; j < a.numCols; j++ , indexC2 += c.numCols) {
            c.data[indexC2] = c.data[indexC++];
         }
      }

   }
}
