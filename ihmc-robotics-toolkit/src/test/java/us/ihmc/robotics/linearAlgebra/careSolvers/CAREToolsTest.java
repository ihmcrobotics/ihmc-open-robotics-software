package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.EjmlUnitTests;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.matrixlib.MatrixTools;

import java.util.Random;

public class CAREToolsTest
{
   private static final int iters = 1000;
   private static final double epsilon = 1e-7;

   @Test
   public void testCalculatingS()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         int rows = RandomNumbers.nextInt(random, 1, 20);
         int cols = RandomNumbers.nextInt(random, 1, 20);
         DenseMatrix64F B = new DenseMatrix64F(rows, cols);
         DenseMatrix64F R = new DenseMatrix64F(cols, cols);
         DenseMatrix64F Rinv = new DenseMatrix64F(cols, cols);
         DenseMatrix64F RinvExpected = new DenseMatrix64F(cols, cols);
         R.zero();
         for (int i = 0; i < cols; i++)
         {
            double value = RandomNumbers.nextDouble(random, 0.01, 5.0);
            R.set(i, i, value);
            RinvExpected.set(i, i, 1.0 / value);
         }
         B.setData(RandomNumbers.nextDoubleArray(random, rows * cols, 10.0));

         DenseMatrix64F BTranspose = new DenseMatrix64F(B);
         CommonOps.transpose(BTranspose);

         DenseMatrix64F S = new DenseMatrix64F(rows, rows);
         CARETools.computeM(BTranspose, R, Rinv, S);

         DenseMatrix64F RinvBTranspose = new DenseMatrix64F(cols, rows);
         CommonOps.multTransB(Rinv, B, RinvBTranspose);

         DenseMatrix64F Sexpected = new DenseMatrix64F(rows, rows);
         CommonOps.mult(B, RinvBTranspose, Sexpected);

         EjmlUnitTests.assertEquals(RinvExpected, Rinv, epsilon);
         EjmlUnitTests.assertEquals(Sexpected, S, epsilon);
      }
   }

   @Test
   public void testAssemblingHamiltonian()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         int n = RandomNumbers.nextInt(random, 1, 20);
         DenseMatrix64F A = new DenseMatrix64F(n, n);
         DenseMatrix64F ATranspose = new DenseMatrix64F(n, n);
         DenseMatrix64F S = new DenseMatrix64F(n, n);
         DenseMatrix64F Q = new DenseMatrix64F(n, n);
         DenseMatrix64F H = new DenseMatrix64F(2 * n, 2 * n);
         DenseMatrix64F HExpected = new DenseMatrix64F(2 * n, 2 * n);

         A.setData(RandomNumbers.nextDoubleArray(random, n * n, 10.0));
         S.setData(RandomNumbers.nextDoubleArray(random, n * n, 10.0));
         for (int i = 0; i < n; i++)
         {
            double value = RandomNumbers.nextDouble(random, 0.01, 5.0);
            Q.set(i, i, value);
         }

         CommonOps.transpose(A, ATranspose);
         CARETools.assembleHamiltonian(A, ATranspose, Q, S, H);

         MatrixTools.setMatrixBlock(HExpected, 0, 0, A, 0, 0, n, n, 1.0);
         MatrixTools.setMatrixBlock(HExpected, 0, n, S, 0, 0, n, n, -1.0);
         MatrixTools.setMatrixBlock(HExpected, n, 0, Q, 0, 0, n, n, -1.0);
         MatrixTools.setMatrixBlock(HExpected, n, n, ATranspose, 0, 0, n, n, -1.0);

         EjmlUnitTests.assertEquals(HExpected, H, epsilon);
      }
   }
}
