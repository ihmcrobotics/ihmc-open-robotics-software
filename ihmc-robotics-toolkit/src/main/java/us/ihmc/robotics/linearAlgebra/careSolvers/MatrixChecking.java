package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;

public class MatrixChecking
{
   static void assertMultiplicationCompatible(DenseMatrix64F a, DenseMatrix64F b)
   {
      if (a.getNumCols() != b.getNumRows())
         throw new RuntimeException("Cols don't match rows.");
   }

   static void assertRowDimensionsMatch(DenseMatrix64F a, DenseMatrix64F b)
   {
      if (a.getNumRows() != b.getNumRows())
         throw new IllegalArgumentException("Number of rows do not match : " + a.getNumRows() + ", " + b.getNumRows());
   }

   static void assertColDimensionsMatch(DenseMatrix64F a, DenseMatrix64F b)
   {
      if (a.getNumCols() != b.getNumCols())
         throw new IllegalArgumentException("Number of cols do not match : " + a.getNumCols() + ", " + b.getNumCols());
   }

   static boolean isSquare(DenseMatrix64F matrix64F)
   {
      return matrix64F.getNumRows() == matrix64F.getNumRows();
   }
}
