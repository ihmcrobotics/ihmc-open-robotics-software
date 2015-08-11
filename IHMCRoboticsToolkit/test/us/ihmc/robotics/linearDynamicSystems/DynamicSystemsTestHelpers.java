package us.ihmc.robotics.linearDynamicSystems;

import Jama.Matrix;
import junit.framework.Assert;
import us.ihmc.robotics.dataStructures.ComplexNumber;

public class DynamicSystemsTestHelpers
{
   public static void assertEpsilonEquals(double[] expectedArray, double[] actualArray, double epsilon)
   {
      Assert.assertEquals(expectedArray.length, actualArray.length);

      for (int i = 0; i < expectedArray.length; i++)
      {
         Assert.assertEquals(expectedArray[i], actualArray[i], epsilon);
      }
   }

   public static void assertEpsilonEquals(ComplexNumber expectedComplexNumber, ComplexNumber actualComplexNumber, double epsilon)
   {
      Assert.assertEquals(expectedComplexNumber.real(), actualComplexNumber.real(), epsilon);
      Assert.assertEquals(expectedComplexNumber.imag(), actualComplexNumber.imag(), epsilon);
   }

   public static void assertEpsilonEquals(ComplexNumber[] expectedComplexNumbers, ComplexNumber[] actualComplexNumbers, double epsilon)
   {
      Assert.assertEquals(expectedComplexNumbers.length, actualComplexNumbers.length);

      for (int i = 0; i < expectedComplexNumbers.length; i++)
      {
         assertEpsilonEquals(expectedComplexNumbers[i], actualComplexNumbers[i], epsilon);

      }
   }


   public static void printMatrix(String string, Matrix M)
   {
      StringBuilder stringBuilder = new StringBuilder(string);
      stringBuilder.append("\n");

      int rows = M.getRowDimension();
      int cols = M.getColumnDimension();

      for (int i = 0; i < rows; i++)
      {
         for (int j = 0; j < cols; j++)
         {
            stringBuilder.append(M.get(i, j));
            stringBuilder.append(" ");
         }

         stringBuilder.append("\n");
      }

      stringBuilder.append("\n");

      System.out.println(stringBuilder.toString());
   }

   public static void printArray(String string, double[] array)
   {
      StringBuilder stringBuilder = new StringBuilder(string);
      stringBuilder.append("\n");


      for (int i = 0; i < array.length; i++)
      {
         stringBuilder.append(array[i]);
         stringBuilder.append(" ");
      }

      stringBuilder.append("\n");

      System.out.println(stringBuilder.toString());
   }

   public static void printComplexArray(String string, double[][] array)
   {
      StringBuilder stringBuilder = new StringBuilder(string);
      stringBuilder.append("\n");

      for (int i = 0; i < array.length; i++)
      {
         stringBuilder.append(array[i][0]);
         stringBuilder.append(" + ");
         stringBuilder.append(array[i][1]);
         stringBuilder.append("j");
         if (i < array.length - 1)
            stringBuilder.append(", ");

      }

      stringBuilder.append("\n");

      System.out.println(stringBuilder.toString());
   }

   public static void printComplexArray(String string, ComplexNumber[] array)
   {
      StringBuilder stringBuilder = new StringBuilder(string);
      stringBuilder.append("\n");

      for (int i = 0; i < array.length; i++)
      {
         stringBuilder.append(array[i]);
         if (i < array.length - 1)
            stringBuilder.append(", ");
      }

      stringBuilder.append("\n");

      System.out.println(stringBuilder.toString());
   }

   public static void printComplexArray(String string, ComplexNumber[][] array)
   {
      for (int i = 0; i < array.length; i++)
      {
         printComplexArray(string, array[i]);
      }
   }
}
