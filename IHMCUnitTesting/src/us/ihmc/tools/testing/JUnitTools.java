package us.ihmc.tools.testing;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

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
import us.ihmc.tools.thread.RunnableThatThrows;

public class JUnitTools
{
   private static final String TEST_RESOURCES_FOLDER_NAME = "testResources";
   
   public static Path deriveTestResourcesPath(Class<?> clazz)
   {
      List<String> pathNames = new ArrayList<String>();
      
      String[] packageNames = clazz.getPackage().getName().split("\\.");
      
      pathNames.addAll(Arrays.asList(packageNames));
      pathNames.add(Character.toLowerCase(clazz.getSimpleName().charAt(0)) + clazz.getSimpleName().substring(1));
      
      return Paths.get(TEST_RESOURCES_FOLDER_NAME, pathNames.toArray(new String[0]));
   }
   
   public static void assertExceptionThrown(Class<? extends Throwable> exceptionType, RunnableThatThrows methodToRun)
   {
      boolean thrown = false;
      
      try
      {
         methodToRun.run();
      }
      catch (Throwable throwable)
      {
         assertTrue("Exception type mismatch: Expected: " + exceptionType.getName() + " Actual: " + throwable.getClass().getName(), exceptionType.getName().equals(throwable.getClass().getName()));
         
         thrown = true;
      }
      
      assertTrue("Exception not thrown", thrown);
   }

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
      assertEquals(message,expected.x, actual.x,delta);
      assertEquals(message,expected.y, actual.y,delta);
      assertEquals(message, expected.z, actual.z, delta);
   }
   
   public static void assertPoint3dEquals(String message, Point3d expected, Point3d actual, double delta)
   {
      assertEquals(message,expected.x, actual.x,delta);
      assertEquals(message,expected.y, actual.y,delta);
      assertEquals(message,expected.z, actual.z,delta);
   }
   
   public static void assertPoint2dEquals(String message, Point2d expected, Point2d actual, double delta)
   {
      assertEquals(message, expected.x, actual.x, delta);
      assertEquals(message, expected.y, actual.y, delta);
   }
   
   public static void assertPoint3fEquals(String message, Point3f expected, Point3f actual, double delta)
   {
      assertEquals(message,expected.x, actual.x,delta);
      assertEquals(message,expected.y, actual.y,delta);
      assertEquals(message, expected.z, actual.z, delta);
   }
   
   public static void assertVector3fEquals(String message, Vector3f expected, Vector3f actual, double delta)
   {
      assertEquals(message,expected.x, actual.x,delta);
      assertEquals(message,expected.y, actual.y,delta);
      assertEquals(message, expected.z, actual.z, delta);
   }
   
   public static void assertVector4dEquals(String message, Vector4d expected, Vector4d actual, double delta)
   {
      assertEquals(message,expected.x, actual.x,delta);
      assertEquals(message,expected.y, actual.y,delta);
      assertEquals(message,expected.z, actual.z,delta);
      assertEquals(message,expected.w, actual.w,delta);
   }
   
   public static void assertVector4fEquals(String message, Vector4f expected, Vector4f actual, double delta)
   {
      assertEquals(message,expected.x, actual.x,delta);
      assertEquals(message,expected.y, actual.y,delta);
      assertEquals(message,expected.z, actual.z,delta);
      assertEquals(message, expected.w, actual.w, delta);
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
      assertEquals(0.0, mat.m00, epsilon);
      assertEquals(0.0, mat.m11, epsilon);
      assertEquals(0.0, mat.m22, epsilon);

      // off-diagonal terms
      assertEquals(0.0, mat.m01 + mat.m10, epsilon);
      assertEquals(0.0, mat.m02 + mat.m20, epsilon);
      assertEquals(0.0, mat.m12 + mat.m21, epsilon);
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

   public static void assertSerializable(Serializable serializable)
   {
      assertSerializable(serializable, false);
   }

   public static void assertSerializable(Serializable serializable, boolean testEquals)
   {
      try
      {
         ByteArrayOutputStream byteArrayOutputStream = new ByteArrayOutputStream();
         ObjectOutputStream objectOutputStream = new ObjectOutputStream(byteArrayOutputStream);
         objectOutputStream.writeObject(serializable);

         ByteArrayInputStream byteArrayInputStream = new ByteArrayInputStream(byteArrayOutputStream.toByteArray());
         ObjectInputStream objectInputStream = new ObjectInputStream(byteArrayInputStream);
         Object object = objectInputStream.readObject();
         if (testEquals)
         {
            assertTrue(object.equals(serializable));
         }
      }
      catch (IOException e)
      {
         fail("Object not serializable");
      }
      catch (ClassNotFoundException ce)
      {
         fail("Object not serializable: class not found");
      }
   }
   
   public static void assertQuaternionsEqual(Quat4f expectedQuaternion, Quat4f actualQuaternion, double epsilon)
   {
      assertQuaternionsEqual(new Quat4d(expectedQuaternion), new Quat4d(actualQuaternion), epsilon);
   }
   
   public static void assertQuaternionsEqual(Quat4d expectedQuaternion, Quat4d actualQuaternion, double epsilon)
   {
      try
      {
         assertEquals(expectedQuaternion.getW(), actualQuaternion.getW(), epsilon);
         assertEquals(expectedQuaternion.getX(), actualQuaternion.getX(), epsilon);
         assertEquals(expectedQuaternion.getY(), actualQuaternion.getY(), epsilon);
         assertEquals(expectedQuaternion.getZ(), actualQuaternion.getZ(), epsilon);
      }
      catch (AssertionError e)
      {
         throw new AssertionError("expected:\n<" + expectedQuaternion + ">\n but was:\n<" + actualQuaternion + ">");
      }
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
