package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.EjmlUnitTests;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.log.LogTools;
import us.ihmc.matrixlib.NativeCommonOps;

import java.util.Random;

public class QRBasedSchurDecompositionTest
{
   private static double epsilon = 1e-8;
   private static int iters = 100;

   @Test
   public void testEasy()
   {
      DenseMatrix64F A = new DenseMatrix64F(3, 3);
      A.set(0, 0, 3);
      A.set(0, 1, 2);
      A.set(0, 2, 1);
      A.set(1, 0, 4);
      A.set(1, 1, 2);
      A.set(1, 2, 1);
      A.set(2, 0, 4);
      A.set(2, 1, 4);

      DenseMatrix64F AExpected = new DenseMatrix64F(A);


      QRBasedSchurDecomposition schur = new QRBasedSchurDecomposition(3);
      schur.setMaxIterations(1000000);
      schur.decompose(A);
      DenseMatrix64F U = schur.getU(null);
      DenseMatrix64F T = schur.getT(null);

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
         DenseMatrix64F A = new DenseMatrix64F(rows, rows);
         A.setData(RandomNumbers.nextDoubleArray(random, rows * rows, 100.0));

         DenseMatrix64F AExpected = new DenseMatrix64F(A);

         schur.decompose(A);
         DenseMatrix64F U = schur.getU(null);
         DenseMatrix64F T = schur.getT(null);

         assertDecompositionHolds(A, U, T);
         EjmlUnitTests.assertEquals(AExpected, A, epsilon);
      }
   }

   private static void assertDecompositionHolds(DenseMatrix64F Aexpected, DenseMatrix64F U, DenseMatrix64F T)
   {
      DenseMatrix64F UTranspose = new DenseMatrix64F(U);
      CommonOps.transpose(U, UTranspose);

      DenseMatrix64F A = new DenseMatrix64F(Aexpected);

      NativeCommonOps.multQuad(UTranspose, T, A);

      EjmlUnitTests.assertEquals(Aexpected, A, epsilon);
   }

}
