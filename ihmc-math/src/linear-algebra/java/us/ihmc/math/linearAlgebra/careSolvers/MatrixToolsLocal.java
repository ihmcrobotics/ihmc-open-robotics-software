package us.ihmc.math.linearAlgebra.careSolvers;

import org.ejml.data.DMatrixD1;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.tools.EuclidCoreTools;

public class MatrixToolsLocal
{
   /**
    * Computes the distance between two matrices, which is defined as the L2 normSquared of their difference.
    */
   public static double distance(DMatrixRMaj A, DMatrixRMaj B)
   {
      MatrixChecking.assertRowDimensionsMatch(A, B);
      MatrixChecking.assertColDimensionsMatch(A, B);

      double norm = 0.0;
      for (int col = 0; col < A.getNumCols(); col++)
      {
         double rowSum = 0.0;
         for (int row = 0; row < A.getNumRows(); row++)
         {
            rowSum += EuclidCoreTools.square(A.get(row, col) - B.get(row, col));
         }
         norm += EuclidCoreTools.square(rowSum);
      }

      return norm;
   }

   public static double normSquared(DMatrixRMaj A)
   {
      double normSquared = 0.0;
      for (int col = 0; col < A.getNumCols(); col++)
      {
         double rowSum = 0.0;
         for (int row = 0; row < A.getNumRows(); row++)
         {
            rowSum += EuclidCoreTools.square(A.get(row, col));
         }
         normSquared += EuclidCoreTools.square(rowSum);
      }

      return normSquared;
   }

   public static double norm(DMatrixRMaj A)
   {
      return EuclidCoreTools.squareRoot(normSquared(A));
   }

   public static void elementWiseMin(DMatrixD1 A, double value)
   {
      for (int i = 0; i < A.getNumElements(); i++)
         A.set(i, Math.min(A.get(i), value));
   }

   static boolean isZero(DMatrixRMaj P, double epsilon)
   {
      return normSquared(P) < epsilon;
   }
}
