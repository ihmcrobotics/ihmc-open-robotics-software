package us.ihmc.robotics.testing;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.jupiter.api.Test;

import Jama.Matrix;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
public class JUnitToolsTest
{
   @Test
   public void testAssertJamaMatrixEquals()
   {
      final int ITERATIONS = 1000;
      final double EPSILON = 0.000001;
      final int MATRIX_SIZE_BOUND = 100;

      Random random1 = new Random(4274L);
      Random random2 = new Random(4274L);

      for(int i = 0; i < ITERATIONS; i++)
      {
         int n1 = random1.nextInt(MATRIX_SIZE_BOUND);
         int m1 = random1.nextInt(MATRIX_SIZE_BOUND);
         Matrix matrix1 = new Matrix(randomDoubleArray(random1, n1 * m1), m1);
         
         int n2 = random2.nextInt(MATRIX_SIZE_BOUND);
         int m2 = random2.nextInt(MATRIX_SIZE_BOUND);
         Matrix matrix2 = new Matrix(randomDoubleArray(random2, n2 * m2), m2);
         

         JUnitTools.assertMatrixEquals(matrix1, matrix2, EPSILON);
         JUnitTools.assertMatrixEquals("testAssertJamaMatrixEquals", matrix1, matrix2, EPSILON);
      }
   }

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

         JUnitTools.assertMatrixEquals(matrix1, matrix2, EPSILON);
         JUnitTools.assertMatrixEquals("testAssertDenseMatrix64FEquals", matrix1, matrix2, EPSILON);
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

         JUnitTools.assertMatrixEqualsZero(matrix, EPSILON);
         JUnitTools.assertMatrixEqualsZero("testAssertMatrixEqualsZero", matrix, EPSILON);
      }
   }

   @Test
   public void testAssertDoubleArrayEquals()
   {
      final int ITERATIONS = 1000;
      final double EPSILON = 0.000001;
      final int MAX_ARRAY_SIZE = 500;

      Random random1 = new Random(4285L);
      Random random2 = new Random(4285L);

      for(int i = 0; i < ITERATIONS; i++)
      {
         double[] array1 = randomDoubleArray(random1, random1.nextInt(MAX_ARRAY_SIZE));
         double[] array2 = randomDoubleArray(random2, random2.nextInt(MAX_ARRAY_SIZE));

         JUnitTools.assertDoubleArrayEquals(array1, array2, EPSILON);
      }
   }
   
   private double[] randomDoubleArray(Random random, int length)
   {
      final double LARGE_VALUE = 4294967296.0; //Cannot use MIN_VALUE here because of overflows
      double[] array = new double[length];
      for(int i = 0; i < length; i++)
      {
         array[i] = -LARGE_VALUE + 2 * LARGE_VALUE * random.nextDouble();
      }
      return array;
   }
}
