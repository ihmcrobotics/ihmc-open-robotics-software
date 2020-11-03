package us.ihmc.robotics;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrix1Row;
import org.ejml.data.DMatrixRMaj;
import org.ejml.data.DMatrix3x3;

public class MatrixMissingTools
{
   public static void setDiagonalValues(DMatrix1Row mat, double diagonalValue, int rowStart, int colStart)
   {
      if (rowStart >= mat.getNumRows())
         throw new IllegalArgumentException("Row start cannot exceed the number of rows.");
      if (colStart >= mat.getNumCols())
         throw new IllegalArgumentException("Col start cannot exceed the number of columns.");

      int width = (mat.getNumRows() - rowStart) < (mat.getNumCols() - colStart) ? (mat.getNumRows() - rowStart) : (mat.getNumCols() - colStart);

      int index = colStart;
      for (int i = 0; i < rowStart && i < width; i++)
         index += mat.getNumCols();

      for (int i = 0; i < width; i++, index += mat.getNumCols() + 1)
      {
         mat.data[index] = diagonalValue;
      }
   }

   public static void fast2x2Inverse(DMatrixRMaj matrix, DMatrixRMaj inverseToPack)
   {
      double determinantInverse = 1.0 / (matrix.get(0, 0) * matrix.get(1, 1) - matrix.get(0, 1) * matrix.get(1, 0));
      inverseToPack.set(0, 0, determinantInverse * matrix.get(1, 1));
      inverseToPack.set(1, 1, determinantInverse * matrix.get(0, 0));
      inverseToPack.set(0, 1, -determinantInverse * matrix.get(0, 1));
      inverseToPack.set(1, 0, -determinantInverse * matrix.get(1, 0));
   }

   public static void setDiagonal(DMatrix3x3 mat, double diagonalValue)
   {
      for (int row = 0; row < 3; row++)
      {
         for (int col = 0; col < 3; col++)
         {
            if (row == col)
               mat.unsafe_set(row, col, diagonalValue);
            else
               mat.unsafe_set(row, col, 0.0);
         }
      }
   }

   public static void setMatrixBlock(DMatrix dest, int destStartRow, int destStartColumn, DMatrix3x3 src, double scale)
   {
      setMatrixBlock(dest, destStartRow, destStartColumn, src, 0, 0, 3, 3, scale);
   }

   public static void setMatrixBlock(DMatrix dest, int destStartRow, int destStartColumn, DMatrix src, int srcStartRow, int srcStartColumn,
                                     int numberOfRows, int numberOfColumns, double scale)
   {
      if (numberOfRows == 0 || numberOfColumns == 0)
         return;

      if (dest.getNumRows() < numberOfRows || dest.getNumCols() < numberOfColumns)
         throw new IllegalArgumentException("dest is too small, min size: [rows: " + numberOfRows + ", cols: " + numberOfColumns + "], was: [rows: "
                                            + dest.getNumRows() + ", cols: " + dest.getNumCols() + "]");
      if (src.getNumRows() < numberOfRows + srcStartRow || src.getNumCols() < numberOfColumns + srcStartColumn)
         throw new IllegalArgumentException("src is too small, min size: [rows: " + (numberOfRows + srcStartRow) + ", cols: "
                                            + (numberOfColumns + srcStartColumn) + "], was: [rows: " + src.getNumRows() + ", cols: " + src.getNumCols() + "]");

      for (int i = 0; i < numberOfRows; i++)
      {
         for (int j = 0; j < numberOfColumns; j++)
         {
            dest.unsafe_set(destStartRow + i, destStartColumn + j, scale * src.unsafe_get(srcStartRow + i, srcStartColumn + j));
         }
      }
   }
}
