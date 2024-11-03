package us.ihmc.robotics.linearAlgebra;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.SingularValueDecomposition_F64;

import us.ihmc.commons.MathTools;
import us.ihmc.matrixlib.MatrixTools;

public class SVDNullspaceCalculator implements NullspaceCalculator
{
   private final ConfigurableSolvePseudoInverseSVD iMinusNNTSolver;

   private final SingularValueDecomposition_F64<DMatrixRMaj> decomposer;
   private final DMatrixRMaj sigma;
   private final DMatrixRMaj v;

   private final DMatrixRMaj nullspace;
   private final DMatrixRMaj Q;
   private final DMatrixRMaj iMinusNNT;
   
   private final DMatrixRMaj nullspaceProjector;
   private final DMatrixRMaj tempMatrixForProjectionInPlace;

   private final boolean makeLargestComponentPositive;

   public SVDNullspaceCalculator(int matrixSize, boolean makeLargestComponentPositive)
   {
      MathTools.checkIntervalContains(matrixSize, 1, Integer.MAX_VALUE);

      iMinusNNT = new DMatrixRMaj(matrixSize, matrixSize);
      double singularValueLimit = 0.5; // because the singular values of I - N * N^T will be either 0 or 1.
      iMinusNNTSolver = new ConfigurableSolvePseudoInverseSVD(matrixSize, matrixSize, singularValueLimit);

      nullspaceProjector = new DMatrixRMaj(matrixSize, matrixSize);
      tempMatrixForProjectionInPlace = new DMatrixRMaj(matrixSize, matrixSize);

      decomposer = DecompositionFactory_DDRM.svd(matrixSize, matrixSize, false, true, false);
      sigma = new DMatrixRMaj(matrixSize, matrixSize);
      v = new DMatrixRMaj(matrixSize, matrixSize);
      nullspace = new DMatrixRMaj(matrixSize, matrixSize);    // oversized, using reshape later
      Q = new DMatrixRMaj(matrixSize, matrixSize);    // oversized, using reshape later
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
      setMatrix(matrixToComputeNullspaceOf, nullity);

      nullspaceProjectorToPack.reshape(matrixToComputeNullspaceOf.getNumCols(), matrixToComputeNullspaceOf.getNumCols());
      CommonOps_DDRM.multOuter(nullspace, nullspaceProjectorToPack);
   }

   public void setMatrix(DMatrixRMaj matrix, int nullity)
   {
      computeNullspace(nullspace, matrix, nullity);
   }

   public void removeNullspaceComponent(DMatrixRMaj vectorToHaveNullspaceRemoved, DMatrixRMaj vectorWithNullspaceRemovedToPack)
   {
      iMinusNNT.reshape(nullspace.getNumRows(), nullspace.getNumRows());
      CommonOps_DDRM.multOuter(nullspace, iMinusNNT);
      CommonOps_DDRM.scale(-1.0, iMinusNNT);
      MatrixTools.addDiagonal(iMinusNNT, 1.0);

      /*
       *  the following is OK because singular values should be either 0 or 1 anyway, since columns of nullspace are orthonormal;
       *  fixes numerical issues. not thread safe however:
       */
      iMinusNNTSolver.setA(iMinusNNT);
      iMinusNNTSolver.solve(vectorToHaveNullspaceRemoved, vectorWithNullspaceRemovedToPack);
   }

   public void removeNullspaceComponent(DMatrixRMaj vectorToHaveNullspaceRemoved)
   {
      tempMatrixForProjectionInPlace.set(vectorToHaveNullspaceRemoved);
      removeNullspaceComponent(tempMatrixForProjectionInPlace, vectorToHaveNullspaceRemoved);
   }

   public void addNullspaceComponent(DMatrixRMaj x, DMatrixRMaj nullspaceMultipliers)
   {
      CommonOps_DDRM.multAdd(nullspace, nullspaceMultipliers, x);
   }

   public DMatrixRMaj getNullspace()
   {
      return nullspace;
   }

   private void computeNullspace(DMatrixRMaj nullspaceToPack, DMatrixRMaj matrixToComputeNullspaceOf, int nullity)
   {
      nullspaceToPack.reshape(matrixToComputeNullspaceOf.getNumCols(), nullity);
      Q.reshape(matrixToComputeNullspaceOf.getNumCols(), matrixToComputeNullspaceOf.getNumCols() - nullity);
      decomposer.decompose(matrixToComputeNullspaceOf);

      sigma.reshape(matrixToComputeNullspaceOf.getNumCols(), matrixToComputeNullspaceOf.getNumRows());
      decomposer.getW(sigma);
      v.reshape(matrixToComputeNullspaceOf.getNumCols(), matrixToComputeNullspaceOf.getNumCols());
      boolean transposed = false;
      decomposer.getV(v, transposed);

      CommonOps_DDRM.extract(v, 0, v.getNumRows(), 0, v.getNumCols() - nullity, Q, 0, 0);
      CommonOps_DDRM.extract(v, 0, v.getNumRows(), v.getNumCols() - nullity, v.getNumCols(), nullspaceToPack, 0, 0);

      if (makeLargestComponentPositive)
      {
         makeLargestComponentInEachRowPositive(nullspaceToPack);
      }
   }

   public static void makeLargestComponentInEachRowPositive(DMatrixRMaj nullspace)
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
   
   public static void makeLargestComponentInEachColumnPositive(DMatrixRMaj nullspaceTranspose)
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
