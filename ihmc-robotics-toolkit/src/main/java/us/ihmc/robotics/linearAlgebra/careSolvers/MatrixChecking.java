package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DMatrixRMaj;

public class MatrixChecking
{
   public static void assertMultiplicationCompatible(DMatrixRMaj a, DMatrixRMaj b)
   {
      if (a.getNumCols() != b.getNumRows())
         throw new RuntimeException("Cols don't match rows.");
   }

   public static void assertRowDimensionsMatch(DMatrixRMaj a, DMatrixRMaj b)
   {
      if (a.getNumRows() != b.getNumRows())
         throw new IllegalArgumentException("Number of rows do not match : " + a.getNumRows() + ", " + b.getNumRows());
   }

   public static void assertColDimensionsMatch(DMatrixRMaj a, DMatrixRMaj b)
   {
      if (a.getNumCols() != b.getNumCols())
         throw new IllegalArgumentException("Number of cols do not match : " + a.getNumCols() + ", " + b.getNumCols());
   }

   public static void assertIsSquare(DMatrixRMaj matrix64F)
   {
      if (matrix64F.getNumRows() != matrix64F.getNumRows())
         throw new IllegalArgumentException("Matrix is not square.");
   }
}
