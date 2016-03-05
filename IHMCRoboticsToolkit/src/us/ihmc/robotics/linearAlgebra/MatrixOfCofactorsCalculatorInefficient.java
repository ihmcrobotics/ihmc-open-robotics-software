package us.ihmc.robotics.linearAlgebra;


import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

public class MatrixOfCofactorsCalculatorInefficient
{
   public static DenseMatrix64F computeMatrixOfCoFactors(DenseMatrix64F mat)
   {
      int m = mat.getNumRows();
      int n = mat.getNumCols();
      DenseMatrix64F ret = new DenseMatrix64F(m, n);
      
      for (int i = 0; i < m; i++)
      {
         for (int j = 0; j < n; j++)
         {
            DenseMatrix64F minor = computeMinor(mat, i, j);
            double minorDeterminant = CommonOps.det(minor);
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

   private static DenseMatrix64F computeMinor(DenseMatrix64F mat, int i, int j)
   {
      int m = mat.getNumRows();
      int n = mat.getNumCols();
      DenseMatrix64F ret = new DenseMatrix64F(m - 1, n - 1);

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
