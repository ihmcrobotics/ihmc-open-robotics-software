package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.commons.MathTools;

public class MatrixToolsLocal
{
   /** Computes the distance between two matrices, which is defined as the L2 norm of their difference. */
   public static double distance(DenseMatrix64F A, DenseMatrix64F B)
   {
      MatrixChecking.assertRowDimensionsMatch(A, B);
      MatrixChecking.assertColDimensionsMatch(A, B);

      double norm = 0.0;
      for (int col = 0; col < A.getNumCols(); col++)
      {
         double rowSum = 0.0;
         for (int row = 0; row < A.getNumRows(); row++)
         {
            rowSum += MathTools.square(A.get(row, col) - B.get(row, col));
         }
         norm += MathTools.square(rowSum);
      }

      return norm;
   }

   public static double norm(DenseMatrix64F A)
   {
      double norm = 0.0;
      for (int col = 0; col < A.getNumCols(); col++)
      {
         double rowSum = 0.0;
         for (int row = 0; row < A.getNumRows(); row++)
         {
            rowSum += MathTools.square(A.get(row, col));
         }
         norm += MathTools.square(rowSum);
      }

      return norm;
   }

   static boolean isZero(DenseMatrix64F P, double epsilon)
   {
      return norm(P) < epsilon;
   }
}
