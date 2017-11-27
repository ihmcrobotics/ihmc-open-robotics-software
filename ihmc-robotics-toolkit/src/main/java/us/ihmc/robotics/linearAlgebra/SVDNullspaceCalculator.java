package us.ihmc.robotics.linearAlgebra;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.ops.CommonOps;
import org.ejml.ops.SingularOps;

import us.ihmc.commons.MathTools;

public class SVDNullspaceCalculator implements NullspaceProjectorCalculator
{
   private final ConfigurableSolvePseudoInverseSVD iMinusNNTSolver;

   private final SingularValueDecomposition<DenseMatrix64F> svd;
   private final DenseMatrix64F sigma;
   private final DenseMatrix64F v;

   private final DenseMatrix64F nullspace;
   private final DenseMatrix64F iMinusNNT;
   
   private final DenseMatrix64F x;

   private final boolean makeLargestComponentPositive;

   private final DenseMatrix64F tempMatrixForProjectionInPlace;
   private final DenseMatrix64F nullspaceProjector;

   public SVDNullspaceCalculator(int matrixSize, boolean makeLargestComponentPositive)
   {
      MathTools.checkIntervalContains(matrixSize, 1, Integer.MAX_VALUE);

      nullspaceProjector = new DenseMatrix64F(matrixSize, matrixSize);
      tempMatrixForProjectionInPlace = new DenseMatrix64F(matrixSize, matrixSize);

      iMinusNNT = new DenseMatrix64F(matrixSize, matrixSize);
      double singularValueLimit = 0.5; // because the singular values of I - N * N^T will be either 0 or 1.
      iMinusNNTSolver = new ConfigurableSolvePseudoInverseSVD(matrixSize, matrixSize, singularValueLimit);

      svd = DecompositionFactory.svd(matrixSize, matrixSize, false, true, false);
      sigma = new DenseMatrix64F(matrixSize, matrixSize);
      v = new DenseMatrix64F(matrixSize, matrixSize);
      nullspace = new DenseMatrix64F(matrixSize, matrixSize);    // oversized, using reshape later
      x = new DenseMatrix64F(matrixSize, matrixSize);// oversized, using reshape later
      this.makeLargestComponentPositive = makeLargestComponentPositive;
   }

   @Override
   public void computeNullspaceProjector(DenseMatrix64F matrixToComputeNullspaceOf, DenseMatrix64F nullspaceProjectorToPack)
   {
      setMatrix(matrixToComputeNullspaceOf, 1);

      iMinusNNT.reshape(nullspace.getNumRows(), nullspace.getNumRows());
      CommonOps.multOuter(nullspace, iMinusNNT);
      CommonOps.scale(-1.0, iMinusNNT);
      MatrixTools.addDiagonal(iMinusNNT, 1.0);

      /*
       *  the following is OK because singular values should be either 0 or 1 anyway, since columns of nullspace are orthonormal;
       *  fixes numerical issues. not thread safe however:
       */
      iMinusNNTSolver.setA(iMinusNNT);
      this.x.set(x);
      iMinusNNTSolver.invert(nullspaceProjectorToPack);
   }

   @Override
   public void projectOntoNullspace(DenseMatrix64F matrixToProjectOntoNullspace, DenseMatrix64F matrixToComputeNullspaceOf)
   {
      tempMatrixForProjectionInPlace.set(matrixToProjectOntoNullspace);
      projectOntoNullspace(tempMatrixForProjectionInPlace, matrixToComputeNullspaceOf, matrixToProjectOntoNullspace);
   }

   @Override
   public void projectOntoNullspace(DenseMatrix64F matrixToProjectOntoNullspace, DenseMatrix64F matrixToComputeNullspaceOf, DenseMatrix64F projectedMatrixToPack)
   {
      computeNullspaceProjector(matrixToComputeNullspaceOf, nullspaceProjector);
      projectedMatrixToPack.reshape(matrixToProjectOntoNullspace.getNumRows(), matrixToProjectOntoNullspace.getNumCols());
      CommonOps.mult(matrixToProjectOntoNullspace, nullspaceProjector, projectedMatrixToPack);
   }

   public void setMatrix(DenseMatrix64F matrix, int nullity)
   {
      computeNullspace(nullspace, matrix, nullity);
   }

   public void removeNullspaceComponent(DenseMatrix64F x)
   {
      iMinusNNT.reshape(nullspace.getNumRows(), nullspace.getNumRows());
      CommonOps.multOuter(nullspace, iMinusNNT);
      CommonOps.scale(-1.0, iMinusNNT);
      MatrixTools.addDiagonal(iMinusNNT, 1.0);

//      SingularValueDecomposition<DenseMatrix64F> svd = DecompositionFactory.svd(iMinusNNT.getNumRows(), iMinusNNT.getNumCols(), false, false, false);
//      svd.decompose(iMinusNNT);
//      double[] singularValues = svd.getSingularValues();

      /*
       *  the following is OK because singular values should be either 0 or 1 anyway, since columns of nullspace are orthonormal;
       *  fixes numerical issues. not thread safe however:
       */
//      double oldEps = UtilEjml.EPS;
//      UtilEjml.EPS = 0.5 / Math.max(iMinusNNT.getNumRows(), iMinusNNT.getNumCols());
      iMinusNNTSolver.setA(iMinusNNT);
      this.x.set(x);
      iMinusNNTSolver.solve(this.x, x);
//      UtilEjml.EPS = oldEps;
   }

   public void addNullspaceComponent(DenseMatrix64F x, DenseMatrix64F nullspaceMultipliers)
   {
      CommonOps.multAdd(nullspace, nullspaceMultipliers, x);
   }

   public DenseMatrix64F getNullspace()
   {
      return nullspace;
   }

   private void computeNullspace(DenseMatrix64F nullspaceToPack, DenseMatrix64F A, int nullity)
   {
      nullspaceToPack.reshape(nullspaceToPack.getNumRows(), nullity);
      svd.decompose(A);

      sigma.reshape(A.getNumCols(), A.getNumRows());
      svd.getW(sigma);
      v.reshape(A.getNumRows(), A.getNumRows());
      boolean transposed = false;
      svd.getV(v, transposed);

      SingularOps.descendingOrder(null, false, sigma, v, transposed);
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
