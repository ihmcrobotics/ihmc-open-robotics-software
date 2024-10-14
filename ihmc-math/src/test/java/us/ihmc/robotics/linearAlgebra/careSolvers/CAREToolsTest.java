package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.EjmlUnitTests;
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
         DMatrixRMaj B = new DMatrixRMaj(rows, cols);
         DMatrixRMaj R = new DMatrixRMaj(cols, cols);
         DMatrixRMaj Rinv = new DMatrixRMaj(cols, cols);
         DMatrixRMaj RinvExpected = new DMatrixRMaj(cols, cols);
         R.zero();
         for (int i = 0; i < cols; i++)
         {
            double value = RandomNumbers.nextDouble(random, 0.01, 5.0);
            R.set(i, i, value);
            RinvExpected.set(i, i, 1.0 / value);
         }
         B.setData(RandomNumbers.nextDoubleArray(random, rows * cols, 10.0));

         DMatrixRMaj BTranspose = new DMatrixRMaj(B);
         CommonOps_DDRM.transpose(BTranspose);

         DMatrixRMaj S = new DMatrixRMaj(rows, rows);
         CARETools.computeM(BTranspose, R, Rinv, S);

         DMatrixRMaj RinvBTranspose = new DMatrixRMaj(cols, rows);
         CommonOps_DDRM.multTransB(Rinv, B, RinvBTranspose);

         DMatrixRMaj Sexpected = new DMatrixRMaj(rows, rows);
         CommonOps_DDRM.mult(B, RinvBTranspose, Sexpected);

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
         DMatrixRMaj A = new DMatrixRMaj(n, n);
         DMatrixRMaj ATranspose = new DMatrixRMaj(n, n);
         DMatrixRMaj S = new DMatrixRMaj(n, n);
         DMatrixRMaj Q = new DMatrixRMaj(n, n);
         DMatrixRMaj H = new DMatrixRMaj(2 * n, 2 * n);
         DMatrixRMaj HExpected = new DMatrixRMaj(2 * n, 2 * n);

         A.setData(RandomNumbers.nextDoubleArray(random, n * n, 10.0));
         S.setData(RandomNumbers.nextDoubleArray(random, n * n, 10.0));
         for (int i = 0; i < n; i++)
         {
            double value = RandomNumbers.nextDouble(random, 0.01, 5.0);
            Q.set(i, i, value);
         }

         CommonOps_DDRM.transpose(A, ATranspose);
         CARETools.assembleHamiltonian(A, ATranspose, Q, S, H);

         MatrixTools.setMatrixBlock(HExpected, 0, 0, A, 0, 0, n, n, 1.0);
         MatrixTools.setMatrixBlock(HExpected, 0, n, S, 0, 0, n, n, -1.0);
         MatrixTools.setMatrixBlock(HExpected, n, 0, Q, 0, 0, n, n, -1.0);
         MatrixTools.setMatrixBlock(HExpected, n, n, ATranspose, 0, 0, n, n, -1.0);

         EjmlUnitTests.assertEquals(HExpected, H, epsilon);
      }
   }
}
