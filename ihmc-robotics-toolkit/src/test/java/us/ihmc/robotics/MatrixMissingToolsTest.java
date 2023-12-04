package us.ihmc.robotics;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.misc.UnrolledInverseFromMinor_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.matrixlib.NativeCommonOps;

import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public class MatrixMissingToolsTest
{
   @Test
   public void testSetDiagonalValues()
   {
      int iters = 100;
      DMatrixRMaj matrixToSet = new DMatrixRMaj(4, 7);
      Random random = new Random(1738L);
      for (int i = 0; i < iters; i++)
      {
         matrixToSet.setData(RandomNumbers.nextDoubleArray(random, 4 * 7, 100));
         DMatrixRMaj originalMatrix = new DMatrixRMaj(matrixToSet);
         double value = RandomNumbers.nextDouble(random, 10.0);
         MatrixMissingTools.setDiagonalValues(matrixToSet, value, 1, 3);

         for (int row = 0; row < 4; row++)
         {
            for (int col = 0; col < 7; col++)
            {
               if (row == 1 && col == 3)
                  assertEquals(value, matrixToSet.get(row, col), 1e-7);
               else if (row == 2 && col == 4)
                  assertEquals(value, matrixToSet.get(row, col), 1e-7);
               else if (row == 3 && col == 5)
                  assertEquals(value, matrixToSet.get(row, col), 1e-7);
               else
                  assertEquals("(" + row + ", " + col + ")", originalMatrix.get(row, col), matrixToSet.get(row, col), 1e-7);
            }

         }
      }
   }

   @Test
   public void testFast2x2Inverse()
   {
      int iters = 500;
      double epsilon = 1e-8;
      Random random = new Random(1738L);
      for (int i = 0; i < iters; i++)
      {
         DMatrixRMaj matrix = new DMatrixRMaj(2, 2);
         DMatrixRMaj matrixInverseExpected = new DMatrixRMaj(2, 2);
         DMatrixRMaj matrixInverse = new DMatrixRMaj(2, 2);
         matrix.setData(RandomNumbers.nextDoubleArray(random, 4, 10.0));

         NativeCommonOps.invert(matrix, matrixInverseExpected);
         MatrixMissingTools.fast2x2Inverse(matrix, matrixInverse);

         MatrixTestTools.assertMatrixEquals(matrixInverseExpected, matrixInverse, epsilon);
      }
   }

   @Test
   public void testToSkewSymmetric()
   {
      int iters = 500;
      double epsilon = 1e-8;
      Random random = new Random(1738L);
      for (int i = 0; i < iters; i++)
      {
         Vector3D vectorA = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D vectorB = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D vectorC = new Vector3D();

         vectorC.cross(vectorA, vectorB);

         DMatrixRMaj vectorBVector = new DMatrixRMaj(3, 1);
         DMatrixRMaj vectorCVector = new DMatrixRMaj(3, 1);
         DMatrixRMaj vectorCActual = new DMatrixRMaj(3, 1);
         DMatrixRMaj skewVectorA = new DMatrixRMaj(3, 3);
         vectorB.get(vectorBVector);
         MatrixMissingTools.toSkewSymmetricMatrix(vectorA, skewVectorA);

         CommonOps_DDRM.mult(skewVectorA, vectorBVector, vectorCVector);
         vectorC.get(vectorCActual);

         MatrixTestTools.assertMatrixEquals(vectorCActual, vectorCVector, epsilon);
      }
   }

}
