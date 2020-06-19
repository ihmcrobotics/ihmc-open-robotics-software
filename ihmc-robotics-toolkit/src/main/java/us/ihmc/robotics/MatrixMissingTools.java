package us.ihmc.robotics;

import org.ejml.data.RowD1Matrix64F;

public class MatrixMissingTools
{
   public static void setDiagonalValues(RowD1Matrix64F mat, double diagonalValue, int rowStart, int colStart)
   {
      if (rowStart >= mat.numRows)
         throw new IllegalArgumentException("Row start cannot exceed the number of rows.");
      if (colStart >= mat.numCols)
         throw new IllegalArgumentException("Col start cannot exceed the number of columns.");

      int width = (mat.numRows - rowStart) < (mat.numCols - colStart) ? (mat.numRows - rowStart) : (mat.numCols - colStart);

      int index = colStart;
      for (int i = 0; i < rowStart && i < width; i++)
         index += mat.numCols;

      for (int i = 0; i < width; i++, index += mat.numCols + 1)
      {
         mat.data[index] = diagonalValue;
      }
   }

}
