package us.ihmc.robotics.linearAlgebra;


import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.matrixlib.MatrixTools;

public class MatrixOfCofactorsCalculatorInefficient
{
   public static DMatrixRMaj computeMatrixOfCoFactors(DMatrixRMaj mat)
   {
      int m = mat.getNumRows();
      int n = mat.getNumCols();
      DMatrixRMaj ret = new DMatrixRMaj(m, n);
      
      for (int i = 0; i < m; i++)
      {
         for (int j = 0; j < n; j++)
         {
            DMatrixRMaj minor = computeMinor(mat, i, j);
            double minorDeterminant = CommonOps_DDRM.det(minor);
            int sign = isEven((i + 1) + (j + 1)) ? 1 : -1;
            ret.set(i, j, sign * minorDeterminant);
         }
      }
      return ret;
   }

   private static boolean isEven(int k)
   {
      return k % 2 == 0;
   }

   private static DMatrixRMaj computeMinor(DMatrixRMaj mat, int i, int j)
   {
      int m = mat.getNumRows();
      int n = mat.getNumCols();
      DMatrixRMaj ret = new DMatrixRMaj(m - 1, n - 1);

      int[] rows = determineIndices(i, m);
      int[] columns = determineIndices(j, n);

      MatrixTools.getMatrixBlock(ret, mat, rows, columns);
      return ret;
   }

   private static int[] determineIndices(int indexToOmit, int size)
   {
      int[] rows = new int[size - 1];
      int k = 0;
      int index = 0;
      while (index < size)
      {
         if (index != indexToOmit)
            rows[k++] = index;
         index++;
      }
      return rows;
   }
}
