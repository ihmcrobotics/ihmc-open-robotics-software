package us.ihmc.robotics.testing;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.jupiter.api.Test;

public class MatrixTestToolsTest
{

   @Test
   public void testAssertDenseMatrix64FEquals()
   {
      final int ITERATIONS = 1000;
      final double EPSILON = 0.000001;
      final int MATRIX_SIZE_BOUND = 100;

      Random random1 = new Random(4275L);
      Random random2 = new Random(4275L);

      for (int i = 0; i < ITERATIONS; i++)
      {

         int n1 = random1.nextInt(MATRIX_SIZE_BOUND);
         int m1 = random1.nextInt(MATRIX_SIZE_BOUND);
         DenseMatrix64F matrix1 = new DenseMatrix64F(n1, m1, true, randomDoubleArray(random1, n1 * m1));

         int n2 = random2.nextInt(MATRIX_SIZE_BOUND);
         int m2 = random2.nextInt(MATRIX_SIZE_BOUND);
         DenseMatrix64F matrix2 = new DenseMatrix64F(n2, m2, true, randomDoubleArray(random2, n2 * m2));

         MatrixTestTools.assertMatrixEquals(matrix1, matrix2, EPSILON);
         MatrixTestTools.assertMatrixEquals("testAssertDenseMatrix64FEquals", matrix1, matrix2, EPSILON);
      }
   }

   @Test
   public void testAssertMatrixEqualsZero()
   {
      final int ITERATIONS = 1000;
      final double EPSILON = 0.000001;
      final int MATRIX_SIZE_BOUND = 100;

      Random random = new Random(4276L);

      for(int i = 0; i < ITERATIONS; i++)
      {
         DenseMatrix64F matrix = new DenseMatrix64F(random.nextInt(MATRIX_SIZE_BOUND), random.nextInt(MATRIX_SIZE_BOUND));

         MatrixTestTools.assertMatrixEqualsZero(matrix, EPSILON);
         MatrixTestTools.assertMatrixEqualsZero("testAssertMatrixEqualsZero", matrix, EPSILON);
      }
   }

   private double[] randomDoubleArray(Random random, int length)
   {
      final double LARGE_VALUE = 4294967296.0; //Cannot use MIN_VALUE here because of overflows
      double[] array = new double[length];
      for (int i = 0; i < length; i++)
      {
         array[i] = -LARGE_VALUE + 2 * LARGE_VALUE * random.nextDouble();
      }
      return array;
   }
}
