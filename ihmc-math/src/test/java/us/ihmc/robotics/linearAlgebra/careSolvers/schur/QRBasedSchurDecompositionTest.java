package us.ihmc.robotics.linearAlgebra.careSolvers.schur;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.EjmlUnitTests;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.matrixlib.NativeCommonOps;

import java.util.Random;

public class QRBasedSchurDecompositionTest
{
   private static double epsilon = 1e-8;
   private static int iters = 100;

   @Test
   public void testEasy()
   {
      DMatrixRMaj A = new DMatrixRMaj(3, 3);
      A.set(0, 0, 3);
      A.set(0, 1, 2);
      A.set(0, 2, 1);
      A.set(1, 0, 4);
      A.set(1, 1, 2);
      A.set(1, 2, 1);
      A.set(2, 0, 4);
      A.set(2, 1, 4);

      DMatrixRMaj AExpected = new DMatrixRMaj(A);


      QRBasedSchurDecomposition schur = new QRBasedSchurDecomposition(3);
      schur.setMaxIterations(1000000);
      schur.decompose(A);
      DMatrixRMaj U = schur.getU(null);
      DMatrixRMaj T = schur.getT(null);

      assertDecompositionHolds(A, U, T);
      EjmlUnitTests.assertEquals(AExpected, A, epsilon);
   }

   @Test
   public void testRandom()
   {
      QRBasedSchurDecomposition schur = new QRBasedSchurDecomposition(3);
      schur.setMaxIterations(10000);

      Random random = new Random(1738L);

      for (int i = 0; i < iters; i++)
      {
         int rows = RandomNumbers.nextInt(random, 1, 10);
         DMatrixRMaj A = new DMatrixRMaj(rows, rows);
         A.setData(RandomNumbers.nextDoubleArray(random, rows * rows, 100.0));

         DMatrixRMaj AExpected = new DMatrixRMaj(A);

         schur.decompose(A);
         DMatrixRMaj U = schur.getU(null);
         DMatrixRMaj T = schur.getT(null);

         assertDecompositionHolds(A, U, T);
         EjmlUnitTests.assertEquals(AExpected, A, epsilon);
      }
   }

   private static void assertDecompositionHolds(DMatrixRMaj Aexpected, DMatrixRMaj U, DMatrixRMaj T)
   {
      DMatrixRMaj UTranspose = new DMatrixRMaj(U);
      CommonOps_DDRM.transpose(U, UTranspose);

      DMatrixRMaj A = new DMatrixRMaj(Aexpected);

      NativeCommonOps.multQuad(UTranspose, T, A);

      EjmlUnitTests.assertEquals(Aexpected, A, epsilon);
   }

}
