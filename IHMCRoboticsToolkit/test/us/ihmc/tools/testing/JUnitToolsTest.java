package us.ihmc.tools.testing;

import java.util.Random;

import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix3f;
import javax.vecmath.Matrix4d;
import javax.vecmath.Matrix4f;
import javax.vecmath.Point2d;
import javax.vecmath.Point2f;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Quat4d;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;
import javax.vecmath.Vector4d;
import javax.vecmath.Vector4f;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import Jama.Matrix;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class JUnitToolsTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAssertPoint3dEquals()
   {
      final int ITERATIONS = 1000;
      final double EPSILON = 0.000001;

      Random random1 = new Random(4270L);
      Random random2 = new Random(4270L);

      for(int i = 0; i < ITERATIONS; i++)
      {
         Point3d point1 = new Point3d(randomDoubleArray(random1, 3));
         Point3d point2 = new Point3d(randomDoubleArray(random2, 3));

         JUnitTools.assertTuple3dEquals(point1, point2, EPSILON);
         JUnitTools.assertTuple3dEquals("testAssertPoint3dEquals", point1, point2, EPSILON);

         JUnitTools.assertPoint3dEquals("testAssertPoint3dEquals", point1, point2, EPSILON);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAssertPoint3fEquals()
   {
      final int ITERATIONS = 1000;
      final float EPSILON = 0.000001f;

      Random random1 = new Random(4271L);
      Random random2 = new Random(4271L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3f point1 = new Point3f(randomFloatArray(random1, 3));
         Point3f point2 = new Point3f(randomFloatArray(random2, 3));

         JUnitTools.assertTuple3fEquals(point1, point2, EPSILON);
         JUnitTools.assertTuple3fEquals("testAssertPoint3fEquals", point1, point2, EPSILON);

         JUnitTools.assertPoint3fEquals("testAssertPoint3fEquals", point1, point2, EPSILON);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAssertPoint2dEquals()
   {
      final int ITERATIONS = 1000;
      final double EPSILON = 0.000001;

      Random random1 = new Random(4272L);
      Random random2 = new Random(4272L);


      for(int i = 0; i < ITERATIONS; i++)
      {
         Point2d point1 = new Point2d(randomDoubleArray(random1, 2));
         Point2d point2 = new Point2d(randomDoubleArray(random2, 2));


         JUnitTools.assertTuple2dEquals(point1, point2, EPSILON);
         JUnitTools.assertTuple2dEquals("testAssertPoint2dEquals", point1, point2, EPSILON);

         JUnitTools.assertPoint2dEquals("testAssertPoint2dEquals", point1, point2, EPSILON);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAssertTuple2fEquals()
   {
      final int ITERATIONS = 1000;
      final float EPSILON = 0.000001f;

      Random random1 = new Random(4273L);
      Random random2 = new Random(4273L);

      for(int i = 0; i < ITERATIONS; i++)
      {
         Point2f point1 = new Point2f(randomFloatArray(random1, 2));
         Point2f point2 = new Point2f(randomFloatArray(random2, 2));
         
         JUnitTools.assertTuple2fEquals(point1, point2, EPSILON);
         JUnitTools.assertTuple2fEquals("testAssertTuple2fEquals", point1, point2, EPSILON);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
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

   @ContinuousIntegrationTest(estimatedDuration = 0.2)
   @Test(timeout = 30000)
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

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAssertMatrix3dEquals()
   {
      final int ITERATIONS = 1000;
      final double EPSILON = 0.000001;

      Random random1 = new Random(4277L);
      Random random2 = new Random(4277L);

      for(int i = 0; i < ITERATIONS; i++)
      {
         Matrix3d matrix1 = new Matrix3d(randomDoubleArray(random1, 9));
         Matrix3d matrix2 = new Matrix3d(randomDoubleArray(random2, 9));

         JUnitTools.assertMatrix3dEquals("testAssertMatrix3dEquals", matrix1, matrix2, EPSILON);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAssertMatrix3fEquals()
   {
      final int ITERATIONS = 1000;
      final double EPSILON = 0.000001;

      Random random1 = new Random(4277L);
      Random random2 = new Random(4277L);

      for(int i = 0; i < ITERATIONS; i++)
      {
         Matrix3f matrix1 = new Matrix3f(randomFloatArray(random1, 9));
         Matrix3f matrix2 = new Matrix3f(randomFloatArray(random2, 9));

         JUnitTools.assertMatrix3fEquals("testAssertMatrix3fEquals", matrix1, matrix2, EPSILON);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAssertMatrix4dEquals()
   {
      final int ITERATIONS = 1000;
      final double EPSILON = 0.000001;

      Random random1 = new Random(4278L);
      Random random2 = new Random(4278L);

      for(int i = 0; i < ITERATIONS; i++)
      {
         Matrix4d matrix1 = new Matrix4d(randomDoubleArray(random1, 16));
         Matrix4d matrix2 = new Matrix4d(randomDoubleArray(random2, 16));


         JUnitTools.assertMatrix4dEquals("testAssertMatrix4dEquals", matrix1, matrix2, EPSILON);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAssertMatrix4fEquals()
   {
      final int ITERATIONS = 1000;
      final double EPSILON = 0.000001;

      Random random1 = new Random(4279L);
      Random random2 = new Random(4279L);

      for(int i = 0; i < ITERATIONS; i++)
      {
         Matrix4f matrix1 = new Matrix4f(randomFloatArray(random1, 16));
         Matrix4f matrix2 = new Matrix4f(randomFloatArray(random2, 16));

         JUnitTools.assertMatrix4fEquals("testAssertMatrix4fEquals", matrix1, matrix2, EPSILON);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAssertVector3dEquals()
   {
      final int ITERATIONS = 1000;
      final double EPSILON = 0.000001;

      Random random1 = new Random(4280L);
      Random random2 = new Random(4280L);

      for(int i = 0; i < ITERATIONS; i++)
      {
         Vector3d vector1 = new Vector3d(randomDoubleArray(random1, 3));
         Vector3d vector2 = new Vector3d(randomDoubleArray(random2, 3));

         JUnitTools.assertTuple3dEquals(vector1, vector2, EPSILON);
         JUnitTools.assertTuple3dEquals("testAssertVector3dEquals", vector1, vector2, EPSILON);

         JUnitTools.assertVector3dEquals("testAssertVector3dEquals", vector1, vector2, EPSILON);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAssertVector3fEquals()
   {
      final int ITERATIONS = 1000;
      final float EPSILON = 0.000001f;

      Random random1 = new Random(4281L);
      Random random2 = new Random(4281L);

      for(int i = 0; i < ITERATIONS; i++)
      {
         Vector3f vector1 = new Vector3f(randomFloatArray(random1, 3));
         Vector3f vector2 = new Vector3f(randomFloatArray(random2, 3));

         JUnitTools.assertTuple3fEquals(vector1, vector2, EPSILON);
         JUnitTools.assertTuple3fEquals("testAssertVector3dEquals", vector1, vector2, EPSILON);

         JUnitTools.assertVector3fEquals("testAssertVector3dEquals", vector1, vector2, EPSILON);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAssertVector4dEquals()
   {
      final int ITERATIONS = 1000;
      final double EPSILON = 0.000001;

      Random random1 = new Random(4282L);
      Random random2 = new Random(4282L);

      for(int i = 0; i < ITERATIONS; i++)
      {         
         Vector4d vector1 = new Vector4d(randomDoubleArray(random1, 4));
         Vector4d vector2 = new Vector4d(randomDoubleArray(random2, 4));

         JUnitTools.assertVector4dEquals("testAssertVector4dEquals", vector1, vector2, EPSILON);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAssertVector4fEquals()
   {
      final int ITERATIONS = 1000;
      final float EPSILON = 0.000001f;

      Random random1 = new Random(4283L);
      Random random2 = new Random(4283L);

      for(int i = 0; i < ITERATIONS; i++)
      {
         Vector4f vector1 = new Vector4f(randomFloatArray(random1, 4));
         Vector4f vector2 = new Vector4f(randomFloatArray(random2, 4));

         JUnitTools.assertVector4fEquals("testAssertVector4dEquals", vector1, vector2, EPSILON);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAssertSkewSymmetric()
   {
      final int ITERATIONS = 1000;
      final double EPSILON = 0.000001;

      Random random = new Random(4284L);

      for(int i = 0; i < ITERATIONS; i++)
      {
         double[] elements = randomDoubleArray(random, 3);
         Matrix3d matrix = new Matrix3d(0.0, elements[0], elements[1], -elements[0], 0.0, elements[2], -elements[1], -elements[2], 0.0);

         JUnitTools.assertSkewSymmetric(matrix, EPSILON);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAssertQuat4fEquals()
   {
      final int ITERATIONS = 1000;
      final float EPSILON = 0.000001f;

      Random random1 = new Random(4286L);
      Random random2 = new Random(4286L);

      for(int i = 0; i < ITERATIONS; i++)
      {
         Quat4f quat1 = new Quat4f(random1.nextFloat(), random1.nextFloat(), random1.nextFloat(), random1.nextFloat());
         Quat4f quat2 = new Quat4f(random2.nextFloat(), random2.nextFloat(), random2.nextFloat(), random2.nextFloat());

         JUnitTools.assertQuaternionsEqual(quat1, quat2, EPSILON);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAssertQuat4dEquals()
   {
      final int ITERATIONS = 1000;
      final double EPSILON = 0.000001;

      Random random1 = new Random(4287L);
      Random random2 = new Random(4287L);

      for(int i = 0; i < ITERATIONS; i++)
      {
         Quat4d quat1 = new Quat4d(random1.nextDouble(), random1.nextDouble(), random1.nextDouble(), random1.nextDouble());
         Quat4d quat2 = new Quat4d(random2.nextDouble(), random2.nextDouble(), random2.nextDouble(), random2.nextDouble());
         

         JUnitTools.assertQuaternionsEqual(quat1, quat2, EPSILON);
         JUnitTools.assertQuaternionsEqualUsingDifference(quat1, quat2, EPSILON);
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
   private float[] randomFloatArray(Random random, int length)
   {
      final float LARGE_VALUE = 4294967296.0f; //Cannot use MIN_VALUE here because of overflows
      float[] array = new float[length];
      for(int i = 0; i < length; i++)
      {
         array[i] = -LARGE_VALUE + 2f * LARGE_VALUE * random.nextFloat();
      }
      return array;
   }
}
