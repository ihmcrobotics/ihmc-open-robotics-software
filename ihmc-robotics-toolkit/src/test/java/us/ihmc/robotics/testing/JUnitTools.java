package us.ihmc.robotics.testing;

import static us.ihmc.robotics.Assert.*;

import gnu.trove.list.array.TDoubleArrayList;
import Jama.Matrix;

public class JUnitTools
{
   public static void assertMatrixEquals(Matrix expected, Matrix actual, double delta)
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
