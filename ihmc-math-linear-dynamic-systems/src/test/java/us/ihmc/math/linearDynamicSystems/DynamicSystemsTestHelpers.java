package us.ihmc.math.linearDynamicSystems;


import java.util.ArrayList;

import Jama.Matrix;
import org.ejml.simple.SimpleMatrix;
import us.ihmc.math.ComplexConjugateMode;
import us.ihmc.math.SingleRealMode;
import us.ihmc.math.ComplexNumber;

import static org.junit.jupiter.api.Assertions.*;

public class DynamicSystemsTestHelpers
{
   public static void assertEpsilonEquals(ComplexNumber expectedComplexNumber, ComplexNumber actualComplexNumber, double epsilon)
   {
      assertEquals(expectedComplexNumber.real(), actualComplexNumber.real(), epsilon);
      assertEquals(expectedComplexNumber.imag(), actualComplexNumber.imag(), epsilon);
   }

   public static void assertEpsilonEquals(ComplexNumber[] expectedComplexNumbers, ComplexNumber[] actualComplexNumbers, double epsilon)
   {
      assertEquals(expectedComplexNumbers.length, actualComplexNumbers.length);

      for (int i = 0; i < expectedComplexNumbers.length; i++)
      {
         assertEpsilonEquals(expectedComplexNumbers[i], actualComplexNumbers[i], epsilon);

      }
   }

   public static void printMatrix(String string, SimpleMatrix M)
   {
      StringBuilder stringBuilder = new StringBuilder(string);
      stringBuilder.append("\n");

      int rows = M.numRows();
      int cols = M.numCols();

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



   public static void printComplexArray(String string, ComplexNumber[] array)
   {
      StringBuilder stringBuilder = new StringBuilder();

      if (string != null)
      {
         stringBuilder.append(string + "\n");
      }

      for (int i = 0; i < array.length; i++)
      {
         stringBuilder.append(array[i]);
         if (i < array.length - 1)
            stringBuilder.append(", ");
      }

      stringBuilder.append("\n");

      System.out.print(stringBuilder.toString());
   }

   public static void printComplexArray(String string, ComplexNumber[][] array)
   {
      if (string != null)
         System.out.println(string);

      for (int i = 0; i < array.length; i++)
      {
         printComplexArray(null, array[i]);
         //         System.out.println();
      }
   }

   public static void printRealModeArray(String string, ArrayList<SingleRealMode> realModes)
   {
      if (string != null)
         System.out.println(string);

      for (SingleRealMode mode : realModes)
      {
         printSingleRealMode(null, mode);
      }
   }

   public static void printSingleRealMode(String string, SingleRealMode mode)
   {
      if (string != null)
         System.out.println(string);

      System.out.println(mode);
   }

   public static void printComplexConjugateModeArray(String string, ArrayList<ComplexConjugateMode> complexConjugateModes)
   {
      if (string != null)
         System.out.println(string);

      for (ComplexConjugateMode mode : complexConjugateModes)
      {
         printComplexConjugateMode(null, mode);
      }
   }

   public static void printComplexConjugateMode(String string, ComplexConjugateMode mode)
   {
      if (string != null)
         System.out.println(string);

      System.out.println(mode);
   }

   public static void checkLeftAndRightEigenvectors(ComplexNumber[][] leftEigenvectorV, ComplexNumber[][] rightEigenvectorW)
   {
      for (int i=0; i<leftEigenvectorV.length; i++)
      {
         checkDotProductIsOne(leftEigenvectorV[i], rightEigenvectorW[i]);
         checkConjugateDotProductIsZero(leftEigenvectorV[i], rightEigenvectorW[i]);
      }
   }

   public static void checkDotProductIsOne(ComplexNumber[] leftEigenvector, ComplexNumber[] rightEigenvector)
   {
      ComplexNumber dotProduct = new ComplexNumber(0.0, 0.0);
      for (int i = 0; i < leftEigenvector.length; i++)
      {
         dotProduct = dotProduct.plus(leftEigenvector[i].times(rightEigenvector[i]));
      }

      assertEquals(1.0, dotProduct.real(), 1e-7);
      assertEquals(0.0, dotProduct.imag(), 1e-7);
   }

   public static void checkConjugateDotProductIsZero(ComplexNumber[] leftEigenvector, ComplexNumber[] rightEigenvector)
   {
       ComplexNumber dotProduct = new ComplexNumber(0.0, 0.0);
       for (int i = 0; i < leftEigenvector.length; i++)
       {
          dotProduct = dotProduct.plus(leftEigenvector[i].times(rightEigenvector[i].conj()));
       }

       assertEquals(0.0, dotProduct.real(), 1e-7);
       assertEquals(0.0, dotProduct.imag(), 1e-7);
   }
}
