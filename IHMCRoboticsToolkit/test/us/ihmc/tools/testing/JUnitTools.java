package us.ihmc.tools.testing;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.fail;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix3f;
import javax.vecmath.Matrix4d;
import javax.vecmath.Matrix4f;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Quat4d;
import javax.vecmath.Quat4f;
import javax.vecmath.Tuple2d;
import javax.vecmath.Tuple2f;
import javax.vecmath.Tuple3d;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector2f;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;
import javax.vecmath.Vector4d;
import javax.vecmath.Vector4f;

import org.ejml.data.DenseMatrix64F;

import Jama.Matrix;

public class JUnitTools
{
   public static void assertTuple3dEquals(Tuple3d expected, Tuple3d actual, double delta)
   {
      assertTuple3dEquals("", expected, actual, delta);
   }

   public static void assertTuple3dEquals(String message, Tuple3d expected, Tuple3d actual, double delta)
   {
      boolean areEqual = expected.epsilonEquals(actual, delta);

      if (message.equals(""))
      {
         message = "Tuple3ds are not Equal!";
      }

      if (!areEqual)
      {
         Vector3d difference = new Vector3d(actual);
         difference.sub(expected);
         fail(message + " Expected = " + expected + ", actual = " + actual + ". norm of difference = " + difference.length());
      }
   }
   
   public static void assertTuple3fEquals(Tuple3f expected, Tuple3f actual, float delta)
   {
      assertTuple3fEquals("", expected, actual, delta);
   }

   public static void assertTuple3fEquals(String message, Tuple3f expected, Tuple3f actual, float delta)
   {
      boolean areEqual = expected.epsilonEquals(actual, delta);

      if (message.equals(""))
      {
         message = "Tuple3ds are not Equal!";
      }

      if (!areEqual)
      {
         Vector3f difference = new Vector3f(actual);
         difference.sub(expected);
         fail(message + " Expected = " + expected + ", actual = " + actual + ". norm of difference = " + difference.length());
      }
   }

   public static void assertTuple2dEquals(Tuple2d expected, Tuple2d actual, double delta)
   {
      assertTuple2dEquals("", expected, actual, delta);
   }

   public static void assertTuple2dEquals(String message, Tuple2d expected, Tuple2d actual, double delta)
   {
      boolean areEqual = expected.epsilonEquals(actual, delta);

      if (message.equals(""))
      {
         message = "Tuple2ds are not Equal!";
      }

      if (!areEqual)
      {
         Vector2d difference = new Vector2d(actual);
         difference.sub(expected);
         fail(message + " Expected = " + expected + ", actual = " + actual + ". norm of difference = " + difference.length());
      }
   }

   public static void assertTuple2fEquals(Tuple2f expected, Tuple2f actual, float delta)
   {
      assertTuple2fEquals("", expected, actual, delta);
   }

   public static void assertTuple2fEquals(String message, Tuple2f expected, Tuple2f actual, float delta)
   {
      boolean areEqual = expected.epsilonEquals(actual, delta);

      if (message.equals(""))
      {
         message = "Tuple2fs are not Equal!";
      }

      if (!areEqual)
      {
         Vector2f difference = new Vector2f(actual);
         difference.sub(expected);
         fail(message + " Expected = " + expected + ", actual = " + actual + ". norm of difference = " + difference.length());
      }
   }

   public static void assertMatrixEquals(Matrix expected, Matrix actual, double delta)
   {
      assertMatrixEquals("", expected, actual, delta);
   }

   public static void assertMatrixEquals(DenseMatrix64F expected, DenseMatrix64F actual, double delta)
   {
      assertMatrixEquals("", expected, actual, delta);
   }

   public static void assertMatrixEquals(String message, Matrix expected, Matrix actual, double delta)
   {
      assertEquals(message, expected.getRowDimension(), actual.getRowDimension());
      assertEquals(message, expected.getColumnDimension(), actual.getColumnDimension());

      for (int i = 0; i < expected.getRowDimension(); i++)
      {
         for (int j = 0; j < expected.getColumnDimension(); j++)
         {
            assertEquals(message, actual.get(i, j), expected.get(i, j), delta);
         }
      }
   }
   
   public static void assertMatrixEqualsZero(DenseMatrix64F matrix, double epsilon)
   {
      assertMatrixEqualsZero("", matrix, epsilon);
   }
   
   public static void assertMatrixEqualsZero(String message, DenseMatrix64F matrix, double epsilon)
   {
      int numberOfRows = matrix.getNumRows();
      int numberOfColumns = matrix.getNumCols();

      for (int row=0; row<numberOfRows; row++)
      {
         for (int column=0; column<numberOfColumns; column++)
         {
            assertEquals(message, 0.0, matrix.get(row, column), epsilon);
         }
      }   
   }

   public static void assertMatrixEquals(String message, DenseMatrix64F expected, DenseMatrix64F actual, double delta)
   {
      assertEquals(message, expected.getNumRows(), actual.getNumRows());
      assertEquals(message, expected.getNumCols(), actual.getNumCols());

      for (int i = 0; i < expected.getNumRows(); i++)
      {
         for (int j = 0; j < expected.getNumCols(); j++)
         {
            assertEquals(message, actual.get(i, j), expected.get(i, j), delta);
         }
      }
   }

   public static void assertMatrix3dEquals(String message, Matrix3d expected, Matrix3d actual, double delta)
   {
      int size = 3;
      for (int i = 0; i < size; i++)
      {
         for (int j = 0; j < size; j++)
         {
            assertEquals(message, actual.getElement(i, j), expected.getElement(i, j), delta);
         }
      }
   }
   
   public static void assertMatrix3fEquals(String message, Matrix3f expected, Matrix3f actual, double delta)
   {
      int size = 3;
      for (int i = 0; i < size; i++)
      {
         for (int j = 0; j < size; j++)
         {
            assertEquals(message, actual.getElement(i, j), expected.getElement(i, j), delta);
         }
      }
   }
   
   public static void assertMatrix4dEquals(String message, Matrix4d expected, Matrix4d actual, double delta)
   {
      int size = 4;
      for (int i = 0; i < size; i++)
      {
         for (int j = 0; j < size; j++)
         {
            assertEquals(message, actual.getElement(i, j), expected.getElement(i, j), delta);
         }
      }
   }
   
   public static void assertMatrix4fEquals(String message, Matrix4f expected, Matrix4f actual, double delta)
   {
      int size = 4;
      for (int i = 0; i < size; i++)
      {
         for (int j = 0; j < size; j++)
         {
            assertEquals(message, actual.getElement(i, j), expected.getElement(i, j), delta);
         }
      }
   }
   
   public static void assertVector3dEquals(String message, Vector3d expected, Vector3d actual, double delta)
   {
      assertEquals(message + " [X component]",expected.getX(), actual.getX(),delta);
      assertEquals(message + " [Y component]",expected.getY(), actual.getY(),delta);
      assertEquals(message + " [Z component]", expected.getZ(), actual.getZ(), delta);
   }
   
   public static void assertPoint3dEquals(String message, Point3d expected, Point3d actual, double delta)
   {
      assertEquals(message + " [X component]",expected.getX(), actual.getX(),delta);
      assertEquals(message + " [Y component]",expected.getY(), actual.getY(),delta);
      assertEquals(message + " [Z component]",expected.getZ(), actual.getZ(),delta);
   }
   
   public static void assertPoint2dEquals(String message, Point2d expected, Point2d actual, double delta)
   {
      assertEquals(message + " [X component]", expected.getX(), actual.getX(), delta);
      assertEquals(message + " [Y component]", expected.getY(), actual.getY(), delta);
   }
   
   public static void assertPoint3fEquals(String message, Point3f expected, Point3f actual, double delta)
   {
      assertEquals(message + " [X component]",expected.getX(), actual.getX(),delta);
      assertEquals(message + " [Y component]",expected.getY(), actual.getY(),delta);
      assertEquals(message + " [Z component]", expected.getZ(), actual.getZ(), delta);
   }
   
   public static void assertVector3fEquals(String message, Vector3f expected, Vector3f actual, double delta)
   {
      assertEquals(message + " [X component]",expected.getX(), actual.getX(),delta);
      assertEquals(message + " [Y component]",expected.getY(), actual.getY(),delta);
      assertEquals(message + " [Z component]", expected.getZ(), actual.getZ(), delta);
   }
   
   public static void assertVector4dEquals(String message, Vector4d expected, Vector4d actual, double delta)
   {
      assertEquals(message + " [X component]",expected.getX(), actual.getX(),delta);
      assertEquals(message + " [Y component]",expected.getY(), actual.getY(),delta);
      assertEquals(message + " [Z component]",expected.getZ(), actual.getZ(),delta);
      assertEquals(message + " [W component]",expected.getW(), actual.getW(),delta);
   }
   
   public static void assertVector4fEquals(String message, Vector4f expected, Vector4f actual, double delta)
   {
      assertEquals(message + " [X component]",expected.getX(), actual.getX(),delta);
      assertEquals(message + " [Y component]",expected.getY(), actual.getY(),delta);
      assertEquals(message + " [Z component]",expected.getZ(), actual.getZ(),delta);
      assertEquals(message + " [W component]", expected.getW(), actual.getW(), delta);
   }

   /**
    * Verifies whether the given matrix is skew-symmetric.
    *
    * @param mat     matrix to check for skew-symmetry
    * @param epsilon numerical tolerance
    */
   public static void assertSkewSymmetric(Matrix3d mat, double epsilon)
   {
      // diagonal terms
      assertEquals(0.0, mat.getM00(), epsilon);
      assertEquals(0.0, mat.getM11(), epsilon);
      assertEquals(0.0, mat.getM22(), epsilon);

      // off-diagonal terms
      assertEquals(0.0, mat.getM01() + mat.getM10(), epsilon);
      assertEquals(0.0, mat.getM02() + mat.getM20(), epsilon);
      assertEquals(0.0, mat.getM12() + mat.getM21(), epsilon);
   }

   public static void assertDoubleArrayEquals(double[] expectedDoubleArray, double[] actualDoubleArray, double epsilon)
   {
      assertNotNull("Expected array is null.", expectedDoubleArray);
      assertNotNull("Actual array is null.", actualDoubleArray);
      assertEquals("Arrays are not the same size. ", expectedDoubleArray.length, actualDoubleArray.length);

      for (int i = 0; i < expectedDoubleArray.length; i++)
      {
         assertEquals("Array disagree at index " + i + " :", expectedDoubleArray[i], actualDoubleArray[i], epsilon);
      }
   }

   public static void assertQuaternionsEqual(Quat4f expectedQuaternion, Quat4f actualQuaternion, double epsilon)
   {
      assertQuaternionsEqual(expectedQuaternion, actualQuaternion, epsilon, true);
   }

   public static void assertQuaternionsEqual(Quat4f expectedQuaternion, Quat4f actualQuaternion, double epsilon, boolean allowOppositeSigns)
   {
      assertQuaternionsEqual(new Quat4d(expectedQuaternion), new Quat4d(actualQuaternion), epsilon, allowOppositeSigns);
   }

   public static void assertQuaternionsEqual(Quat4d expectedQuaternion, Quat4d actualQuaternion, double epsilon)
   {
      assertQuaternionsEqual(expectedQuaternion, actualQuaternion, epsilon, true);
   }

   public static void assertQuaternionsEqual(Quat4d expectedQuaternion, Quat4d actualQuaternion, double epsilon, boolean allowOppositeSigns)
   {
      double negativeOneIfOppositeSigns = 1.0;

      if (allowOppositeSigns)
      {
         boolean oppositeSigns = areOppositeSigns(expectedQuaternion, actualQuaternion);
         if (oppositeSigns )
         {
            negativeOneIfOppositeSigns = -1.0;
         }
      }

      try
      {
         assertEquals(negativeOneIfOppositeSigns * expectedQuaternion.getW(), actualQuaternion.getW(), epsilon);
         assertEquals(negativeOneIfOppositeSigns * expectedQuaternion.getX(), actualQuaternion.getX(), epsilon);
         assertEquals(negativeOneIfOppositeSigns * expectedQuaternion.getY(), actualQuaternion.getY(), epsilon);
         assertEquals(negativeOneIfOppositeSigns * expectedQuaternion.getZ(), actualQuaternion.getZ(), epsilon);
      }
      catch (AssertionError e)
      {
         throw new AssertionError("expected:\n<" + expectedQuaternion + ">\n but was:\n<" + actualQuaternion + ">");
      }
   }
   
   private static boolean areOppositeSigns(Quat4d quaternionOne, Quat4d quaternionTwo)
   {
      if ((quaternionOne.getW() < 0.0) && (quaternionTwo.getW() > 0.0)) return true;
      if ((quaternionOne.getW() > 0.0) && (quaternionTwo.getW() < 0.0)) return true;

      return false;
   }

   public static void assertQuaternionsEqualUsingDifference(Quat4d q1, Quat4d q2, double epsilon)
   {
      try
      {
         Quat4d qDifference = new Quat4d();
         qDifference.mulInverse(q1, q2);
         AxisAngle4d axisAngle = new AxisAngle4d();
         axisAngle.set(qDifference);
         assertEquals(0.0, axisAngle.getAngle(), epsilon);
      }
      catch (AssertionError e)
      {
         throw new AssertionError("expected:\n<" + q1 + ">\n but was:\n<" + q2 + ">");
      }
   }
}
