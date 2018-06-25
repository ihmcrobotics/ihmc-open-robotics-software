package us.ihmc.robotics.testing;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

import gnu.trove.list.array.TDoubleArrayList;
import org.ejml.data.DenseMatrix64F;

import Jama.Matrix;

public class JUnitTools
{
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

   public static void assertTDoubleArrayListEquals(TDoubleArrayList expected, TDoubleArrayList actual, double epsilon)
   {
      assertEquals("Size is not equal. Expected " + expected.size() + ", received " + actual.size() + ".", expected.size(), actual.size());
      for (int i = 0; i < expected.size(); i++)
         assertEquals("Value " + i + " is not equal. Expected " + expected.get(i) + ", received " + actual.get(i) + ".", expected.get(i), actual.get(i), epsilon);
   }
}
