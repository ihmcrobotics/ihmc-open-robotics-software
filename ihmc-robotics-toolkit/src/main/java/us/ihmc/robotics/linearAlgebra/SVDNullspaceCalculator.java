package us.ihmc.robotics.linearAlgebra;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.ops.CommonOps;
import org.ejml.ops.SingularOps;

import us.ihmc.commons.MathTools;

public class SVDNullspaceCalculator implements NullspaceCalculator
{
   private final ConfigurableSolvePseudoInverseSVD iMinusNNTSolver;

   private final SingularValueDecomposition<DenseMatrix64F> decomposer;
   private final DenseMatrix64F sigma;
   private final DenseMatrix64F v;

   private final DenseMatrix64F nullspace;
   private final DenseMatrix64F Q;
   private final DenseMatrix64F iMinusNNT;
   
   private final DenseMatrix64F nullspaceProjector;
   private final DenseMatrix64F tempMatrixForProjectionInPlace;

   private final boolean makeLargestComponentPositive;

   public SVDNullspaceCalculator(int matrixSize, boolean makeLargestComponentPositive)
   {
      MathTools.checkIntervalContains(matrixSize, 1, Integer.MAX_VALUE);

      iMinusNNT = new DenseMatrix64F(matrixSize, matrixSize);
      double singularValueLimit = 0.5; // because the singular values of I - N * N^T will be either 0 or 1.
      iMinusNNTSolver = new ConfigurableSolvePseudoInverseSVD(matrixSize, matrixSize, singularValueLimit);

      nullspaceProjector = new DenseMatrix64F(matrixSize, matrixSize);
      tempMatrixForProjectionInPlace = new DenseMatrix64F(matrixSize, matrixSize);

      decomposer = DecompositionFactory.svd(matrixSize, matrixSize, false, true, false);
      sigma = new DenseMatrix64F(matrixSize, matrixSize);
      v = new DenseMatrix64F(matrixSize, matrixSize);
      nullspace = new DenseMatrix64F(matrixSize, matrixSize);    // oversized, using reshape later
      Q = new DenseMatrix64F(matrixSize, matrixSize);    // oversized, using reshape later
      this.makeLargestComponentPositive = makeLargestComponentPositive;
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
      int nullity = Math.max(matrixToComputeNullspaceOf.getNumCols() - matrixToComputeNullspaceOf.getNumRows(), 0);
      setMatrix(matrixToComputeNullspaceOf, nullity);

      nullspaceProjectorToPack.reshape(matrixToComputeNullspaceOf.getNumCols(), matrixToComputeNullspaceOf.getNumCols());
      CommonOps.multOuter(nullspace, nullspaceProjectorToPack);
   }

   public void setMatrix(DenseMatrix64F matrix, int nullity)
   {
      computeNullspace(nullspace, matrix, nullity);
   }

   public void removeNullspaceComponent(DenseMatrix64F vectorToHaveNullspaceRemoved, DenseMatrix64F vectorWithNullspaceRemovedToPack)
   {
      iMinusNNT.reshape(nullspace.getNumRows(), nullspace.getNumRows());
      CommonOps.multOuter(nullspace, iMinusNNT);
      CommonOps.scale(-1.0, iMinusNNT);
      MatrixTools.addDiagonal(iMinusNNT, 1.0);

      /*
       *  the following is OK because singular values should be either 0 or 1 anyway, since columns of nullspace are orthonormal;
       *  fixes numerical issues. not thread safe however:
       */
      iMinusNNTSolver.setA(iMinusNNT);
      iMinusNNTSolver.solve(vectorToHaveNullspaceRemoved, vectorWithNullspaceRemovedToPack);
   }

   public void removeNullspaceComponent(DenseMatrix64F vectorToHaveNullspaceRemoved)
   {
      tempMatrixForProjectionInPlace.set(vectorToHaveNullspaceRemoved);
      removeNullspaceComponent(tempMatrixForProjectionInPlace, vectorToHaveNullspaceRemoved);
   }

   public void addNullspaceComponent(DenseMatrix64F x, DenseMatrix64F nullspaceMultipliers)
   {
      CommonOps.multAdd(nullspace, nullspaceMultipliers, x);
   }

   public DenseMatrix64F getNullspace()
   {
      return nullspace;
   }

   private void computeNullspace(DenseMatrix64F nullspaceToPack, DenseMatrix64F matrixToComputeNullspaceOf, int nullity)
   {
      nullspaceToPack.reshape(matrixToComputeNullspaceOf.getNumCols(), nullity);
      Q.reshape(matrixToComputeNullspaceOf.getNumCols(), matrixToComputeNullspaceOf.getNumCols() - nullity);
      decomposer.decompose(matrixToComputeNullspaceOf);

      sigma.reshape(matrixToComputeNullspaceOf.getNumCols(), matrixToComputeNullspaceOf.getNumRows());
      decomposer.getW(sigma);
      v.reshape(matrixToComputeNullspaceOf.getNumCols(), matrixToComputeNullspaceOf.getNumCols());
      boolean transposed = false;
      decomposer.getV(v, transposed);

      CommonOps.extract(v, 0, v.getNumRows(), 0, v.getNumCols() - nullity, Q, 0, 0);
      CommonOps.extract(v, 0, v.getNumRows(), v.getNumCols() - nullity, v.getNumCols(), nullspaceToPack, 0, 0);

      if (makeLargestComponentPositive)
      {
         makeLargestComponentInEachRowPositive(nullspaceToPack);
      }
   }

   public static void makeLargestComponentInEachRowPositive(DenseMatrix64F nullspace)
   {
      for (int column = 0; column < nullspace.getNumCols(); column++)
      {
         int largestAbsoluteComponentRow = -1;
         double largestAbsoluteComponentValue = 0.0;
         for (int row = 0; row < nullspace.getNumRows(); row++)
         {
            double absoluteComponentValue = Math.abs(nullspace.get(row, column));
            if (absoluteComponentValue > largestAbsoluteComponentValue)
            {
               largestAbsoluteComponentRow = row;
               largestAbsoluteComponentValue = absoluteComponentValue;
            }
         }
         
         if (nullspace.get(largestAbsoluteComponentRow, column) < 0.0)
         {            
            for (int row = 0; row < nullspace.getNumRows(); row++)
            {
               nullspace.set(row, column, -nullspace.get(row, column));
            }
         }
      }
   }
   
   public static void makeLargestComponentInEachColumnPositive(DenseMatrix64F nullspaceTranspose)
   {
      for (int row = 0; row < nullspaceTranspose.getNumRows(); row++)
      {
         int largestAbsoluteComponentColumn = -1;
         double largestAbsoluteComponentValue = 0.0;
         for (int column = 0; column < nullspaceTranspose.getNumCols(); column++)
         {
            double absoluteComponentValue = Math.abs(nullspaceTranspose.get(row, column));
            if (absoluteComponentValue > largestAbsoluteComponentValue)
            {
               largestAbsoluteComponentColumn = column;
               largestAbsoluteComponentValue = absoluteComponentValue;
            }
         }
         
         if (nullspaceTranspose.get(row, largestAbsoluteComponentColumn) < 0.0)
         {            
            for (int column = 0; column < nullspaceTranspose.getNumCols(); column++)
            {
               nullspaceTranspose.set(row, column, -nullspaceTranspose.get(row, column));
            }
         }
      }
   }
}
