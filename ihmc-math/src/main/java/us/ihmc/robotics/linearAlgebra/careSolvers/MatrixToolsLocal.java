package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.commons.MathTools;

public class MatrixToolsLocal
{
   public static void setMatrixBlockToIdentity(DMatrixRMaj dest, int row, int col, int sizeToSet)
   {
      setMatrixBlockToConstant(dest, row, col, sizeToSet, 1.0);
   }

   public static void setMatrixBlockToConstant(DMatrixRMaj dest, int row, int col, int sizeToSet, double value)
   {
      for (int i = 0; i < sizeToSet; i++)
      {
         dest.set(row + i, col + i, value);
      }
   }

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
            rowSum += MathTools.square(A.get(row, col) - B.get(row, col));
         }
         norm += MathTools.square(rowSum);
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
            rowSum += MathTools.square(A.get(row, col));
         }
         normSquared += MathTools.square(rowSum);
      }

      return normSquared;
   }

   public static double norm(DMatrixRMaj A)
   {
      return Math.sqrt(normSquared(A));
   }

   static boolean isZero(DMatrixRMaj P, double epsilon)
   {
      return normSquared(P) < epsilon;
   }

   public static void elementWiseMin(DMatrix matrixToClamp, double minValue)
   {
      int numRow = matrixToClamp.getNumRows();
      int numCol = matrixToClamp.getNumCols();
      for (int r = 0; r < numRow; r++)
      {
         for (int c = 0; c < numCol; c++)
         {
            if (matrixToClamp.get(r, c) < minValue)
            {
               matrixToClamp.set(r, c, minValue);
            }
         }
      }
   }

   public static void elementWiseMax(DMatrix matrixToClamp, double maxValue)
   {
      int numRow = matrixToClamp.getNumRows();
      int numCol = matrixToClamp.getNumCols();
      for (int r = 0; r < numRow; r++)
      {
         for (int c = 0; c < numCol; c++)
         {
            if (matrixToClamp.get(r, c) > maxValue)
            {
               matrixToClamp.set(r, c, maxValue);
            }
         }
      }
   }
}
