package us.ihmc.robotics.testing;

import static us.ihmc.robotics.Assert.assertEquals;

import org.ejml.data.DenseMatrix64F;

public class MatrixTestTools
{

   public static void assertMatrixEquals(DenseMatrix64F expected, DenseMatrix64F actual, double delta)
   {
      MatrixTestTools.assertMatrixEquals("", expected, actual, delta);
   }

   public static void assertMatrixEqualsZero(DenseMatrix64F matrix, double epsilon)
   {
      MatrixTestTools.assertMatrixEqualsZero("", matrix, epsilon);
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
            assertEquals(message + " index (" + i + ", " + j + ")", expected.get(i, j), actual.get(i, j), delta);
         }
      }
   }

}
