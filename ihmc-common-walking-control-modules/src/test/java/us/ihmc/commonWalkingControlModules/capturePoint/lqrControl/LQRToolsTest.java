package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.EjmlUnitTests;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.linearAlgebra.MatrixToolsTest;

import java.util.Random;

public class LQRToolsTest
{
   @Test
   public void testMatrixExponential()
   {
      Random random = new Random(1738L);
      int depth = 100;

      for (int i = 0; i < 1000; i++)
      {
         DenseMatrix64F matrix = randomMatrix(random);
         DenseMatrix64F expectedExponential = computeMatrixExponential(matrix, depth);
         DenseMatrix64F returnedExponential = new DenseMatrix64F(matrix.numRows, matrix.numCols);
         LQRTools.matrixExponential(returnedExponential, matrix, depth);

         EjmlUnitTests.assertEquals(expectedExponential, returnedExponential, 1e-5);
      }
   }

   private static DenseMatrix64F computeMatrixExponential(DenseMatrix64F A, int depth)
   {
      DenseMatrix64F matrixToReturn = new DenseMatrix64F(A.numRows, A.numCols);
      for (int k = 0; k < depth; k++)
      {
         int denominator = factorial(1);
         DenseMatrix64F term = matrixPower(A, k);
         CommonOps.addEquals(matrixToReturn, 1.0 / denominator, term);
      }

      return matrixToReturn;
   }

   private static DenseMatrix64F matrixPower(DenseMatrix64F A, int power)
   {
      DenseMatrix64F matrixToReturn = CommonOps.identity(A.numRows);
      for (int i = 1; i < power; i++)
      {
         DenseMatrix64F tempMatrix = new DenseMatrix64F(matrixToReturn);
         CommonOps.mult(tempMatrix, A, matrixToReturn);
      }

      return matrixToReturn;
   }

   private static int factorial(int value)
   {
      int valueToReturn = 1;
      for (int i = 1; i < value; i++)
      {
         valueToReturn *= i;
      }

      return valueToReturn;
   }

   private static DenseMatrix64F randomMatrix(Random random)
   {
      int size = RandomNumbers.nextInt(random, 1, 50);
      double[] values = RandomNumbers.nextDoubleArray(random, size * size, 100.0);
      DenseMatrix64F matrix = new DenseMatrix64F(size, size);
      matrix.setData(values);

      return matrix;
   }
}
