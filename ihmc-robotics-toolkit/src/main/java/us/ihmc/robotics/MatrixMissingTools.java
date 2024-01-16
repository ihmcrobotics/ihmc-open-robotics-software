package us.ihmc.robotics;

import org.ejml.MatrixDimensionException;
import org.ejml.data.DMatrix;
import org.ejml.data.DMatrix1Row;
import org.ejml.data.DMatrix3x3;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.simple.SimpleMatrix;

import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.matrixlib.MatrixTools;

public class MatrixMissingTools
{
   /**
    * Sets a block of a matrix
    *
    * @param dest            Set a block of this matrix
    * @param destStartRow    Row index of the top left corner of the block to set
    * @param destStartColumn Column index of the top left corner of the block to set
    * @param src             Get a block of this matrix
    * @param srcStartRow     Row index of the top left corner of the block to use from otherMatrix
    * @param srcStartColumn  Column index of the top left corner of the block to use from otherMatrix
    * @param numberOfRows    Row size of the block
    * @param numberOfColumns Column size of the block
    * @param scale           Scale the block from otherMatrix by this value
    */
   public static void setMatrixBlock(DMatrix1Row dest,
                                     int destStartRow,
                                     int destStartColumn,
                                     DMatrix src,
                                     int srcStartRow,
                                     int srcStartColumn,
                                     int numberOfRows,
                                     int numberOfColumns,
                                     double scale)
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

   public static void setMatrixBlock(DMatrix dest,
                                     int destStartRow,
                                     int destStartColumn,
                                     DMatrix src,
                                     int srcStartRow,
                                     int srcStartColumn,
                                     int numberOfRows,
                                     int numberOfColumns,
                                     double scale)
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

   public static void setMatrixRows(DMatrixRMaj dest,
                                       int destStartRow,
                                       DMatrixRMaj src,
                                       int srcStartRow,
                                       int numberOfRows)
   {
      if (numberOfRows == 0)
         return;

      if (dest.getNumCols() != src.getNumCols())
         throw new IllegalArgumentException("dest and src must have the same number of columns, was: [dest cols: " + dest.getNumCols() + ", src cols: "
                                            + src.getNumCols() + "]");
      if (dest.getNumRows() < numberOfRows + destStartRow)
         throw new IllegalArgumentException("dest is too small, min size: [rows: " + (numberOfRows + destStartRow) + "], was: [rows: "
                                            + dest.getNumRows() + "]");
      if (src.getNumRows() < numberOfRows + srcStartRow)
         throw new IllegalArgumentException("src is too small, min size: [rows: " + (numberOfRows + srcStartRow) + "], was: [rows: "
                                            + src.getNumRows() + "]");

      for (int i = 0; i < numberOfRows; i++)
      {
         for (int j = 0; j < dest.getNumCols(); j++)
         {
            dest.unsafe_set(destStartRow + i, j, src.unsafe_get(srcStartRow + i, j));
         }
      }
   }

   public static void setMatrixRow(DMatrixRMaj dest,
                                   int destRow,
                                   DMatrixRMaj src,
                                   int srcRow)
   {
      if (dest.getNumCols() != src.getNumCols())
         throw new IllegalArgumentException("dest and src must have the same number of columns, was: [dest cols: " + dest.getNumCols() + ", src cols: "
                                            + src.getNumCols() + "]");
      if (dest.getNumRows() < destRow + 1)
         throw new IllegalArgumentException("dest is too small, min size: [rows: " + (destRow + 1) + "], was: [rows: "
                                            + dest.getNumRows() + "]");
      if (src.getNumRows() < srcRow + 1)
         throw new IllegalArgumentException("src is too small, min size: [rows: " + (srcRow + 1) + "], was: [rows: "
                                            + src.getNumRows() + "]");

      for (int j = 0; j < dest.getNumCols(); j++)
      {
         dest.unsafe_set(destRow, j, src.unsafe_get(srcRow, j));
      }
   }

   public static void setMatrixColumns(DMatrixRMaj dest,
                                       int destStartColumn,
                                       DMatrixRMaj src,
                                       int srcStartColumn,
                                       int numberOfColumns)
   {
      if (numberOfColumns == 0)
         return;

      if (dest.getNumRows() != src.getNumRows())
         throw new IllegalArgumentException("dest and src must have the same number of rows, was: [dest rows: " + dest.getNumRows() + ", src rows: "
                                            + src.getNumRows() + "]");
      if (dest.getNumCols() < numberOfColumns + destStartColumn)
         throw new IllegalArgumentException("dest is too small, min size: [cols: " + (numberOfColumns + destStartColumn) + "], was: [cols: "
                                            + dest.getNumCols() + "]");
      if (src.getNumCols() < numberOfColumns + srcStartColumn)
         throw new IllegalArgumentException("src is too small, min size: [cols: " + (numberOfColumns + srcStartColumn) + "], was: [cols: "
                                            + src.getNumCols() + "]");

      for (int i = 0; i < dest.getNumRows(); i++)
      {
         for (int j = 0; j < numberOfColumns; j++)
         {
            dest.unsafe_set(i, destStartColumn + j, src.unsafe_get(i, srcStartColumn + j));
         }
      }
   }

   public static void setMatrixColumn(DMatrixRMaj dest,
                                      int destColumn,
                                      DMatrixRMaj src,
                                      int srcColumn)
   {
      if (dest.getNumRows() != src.getNumRows())
         throw new IllegalArgumentException("dest and src must have the same number of rows, was: [dest rows: " + dest.getNumRows() + ", src rows: "
                                            + src.getNumRows() + "]");
      if (dest.getNumCols() < destColumn + 1)
         throw new IllegalArgumentException("dest is too small, min size: [cols: " + (destColumn + 1) + "], was: [cols: "
                                            + dest.getNumCols() + "]");
      if (src.getNumCols() < srcColumn + 1)
         throw new IllegalArgumentException("src is too small, min size: [cols: " + (srcColumn + 1) + "], was: [cols: "
                                            + src.getNumCols() + "]");

      for (int i = 0; i < dest.getNumRows(); i++)
      {
         dest.unsafe_set(i, destColumn, src.unsafe_get(i, srcColumn));
      }
   }

   public static DMatrixRMaj createVector(int size, double fillValue)
   {
      DMatrixRMaj vector = new DMatrixRMaj(size, 1);
      CommonOps_DDRM.fill(vector, fillValue);
      return vector;
   }

   public static DMatrixRMaj createVector(Tuple3DReadOnly tuple)
   {
      DMatrixRMaj vector = new DMatrixRMaj(3, 1);
      tuple.get(vector);
      return vector;
   }

   public static DMatrixRMaj createRowVector(double... values)
   {
      DMatrixRMaj vector = new DMatrixRMaj(1, values.length);
      for (int i = 0; i < values.length; i++)
      {
         vector.set(i, values[i]);
      }

      return vector;
   }

   public static boolean epsilonEquals(DMatrix1Row a, DMatrix1Row b, double epsilon)
   {
      if (a.numRows != b.numRows)
         return false;
      if (a.numCols != b.numCols)
         return false;
      for (int i = 0; i < a.getNumElements(); i++)
      {
         if (!EuclidCoreTools.epsilonEquals(a.get(i), b.get(i), epsilon))
            return false;
      }
      return true;
   }

   public static SimpleMatrix toSkewSymmetricMatrix(DMatrix1Row vector)
   {
      SimpleMatrix skewSymmetric = new SimpleMatrix(vector.getNumElements(), vector.getNumElements());
      skewSymmetric.set(0, 0, 0.0);
      skewSymmetric.set(0, 1, -vector.get(2));
      skewSymmetric.set(0, 2, vector.get(1));

      skewSymmetric.set(1, 0, vector.get(2));
      skewSymmetric.set(1, 1, 0.0);
      skewSymmetric.set(1, 2, -vector.get(0));

      skewSymmetric.set(2, 0, -vector.get(1));
      skewSymmetric.set(2, 1, vector.get(0));
      skewSymmetric.set(2, 2, 0.0);
      return skewSymmetric;
   }

   public static void fromSkewSymmetricMatrix(DMatrixRMaj skewSymmetric, Vector3DBasics vectorToPack)
   {
      vectorToPack.setX(skewSymmetric.get(2, 1));
      vectorToPack.setY(skewSymmetric.get(0, 2));
      vectorToPack.setZ(skewSymmetric.get(1, 0));
   }

   public static void fromSkewSymmetricMatrix(Matrix3DReadOnly skewSymmetric, Vector3DBasics vectorToPack)
   {
      vectorToPack.setX(skewSymmetric.getM21());
      vectorToPack.setY(skewSymmetric.getM02());
      vectorToPack.setZ(skewSymmetric.getM10());
   }

   public static void toSkewSymmetricMatrix(Tuple3DReadOnly vector, DMatrixRMaj skewSymmetricToPack)
   {
      toSkewSymmetricMatrix(vector.getX(), vector.getY(), vector.getZ(), skewSymmetricToPack, 0, 0);
   }

   public static void toSkewSymmetricMatrix(Tuple3DReadOnly vector, DMatrixRMaj skewSymmetricToPack, int rowStart, int colStart)
   {
      toSkewSymmetricMatrix(vector.getX(), vector.getY(), vector.getZ(), skewSymmetricToPack, rowStart, colStart);
   }

   public static void toSkewSymmetricMatrix(double scalar, Tuple3DReadOnly vector, DMatrixRMaj skewSymmetricToPack, int rowStart, int colStart)
   {
      toSkewSymmetricMatrix(scalar, vector.getX(), vector.getY(), vector.getZ(), skewSymmetricToPack, rowStart, colStart);
   }

   public static void toSkewSymmetricMatrix(DMatrix1Row vector, DMatrixRMaj skewSymmetricToPack)
   {
      toSkewSymmetricMatrix(vector, skewSymmetricToPack, 0, 0);
   }

   public static void toSkewSymmetricMatrix(DMatrix1Row vector, DMatrixRMaj skewSymmetricToPack, int rowStart, int colStart)
   {
      toSkewSymmetricMatrix(vector.get(0), vector.get(1), vector.get(2), skewSymmetricToPack, rowStart, colStart);
   }

   public static void toSkewSymmetricMatrix(double x, double y, double z, DMatrixRMaj skewSymmetricToPack, int rowStart, int colStart)
   {
      int row1 = rowStart + 1;
      int row2 = rowStart + 2;
      int col1 = colStart + 1;
      int col2 = colStart + 2;
      skewSymmetricToPack.set(rowStart, colStart, 0.0);
      skewSymmetricToPack.set(rowStart, col1, -z);
      skewSymmetricToPack.set(rowStart, col2, y);

      skewSymmetricToPack.set(row1, colStart, z);
      skewSymmetricToPack.set(row1, col1, 0.0);
      skewSymmetricToPack.set(row1, col2, -x);

      skewSymmetricToPack.set(row2, colStart, -y);
      skewSymmetricToPack.set(row2, col1, x);
      skewSymmetricToPack.set(row2, col2, 0.0);
   }

   public static void toSkewSymmetricMatrix(double scalar, double x, double y, double z, DMatrixRMaj skewSymmetricToPack, int rowStart, int colStart)
   {
      int row1 = rowStart + 1;
      int row2 = rowStart + 2;
      int col1 = colStart + 1;
      int col2 = colStart + 2;
      skewSymmetricToPack.set(rowStart, colStart, 0.0);
      skewSymmetricToPack.set(rowStart, col1, -scalar * z);
      skewSymmetricToPack.set(rowStart, col2, scalar * y);

      skewSymmetricToPack.set(row1, colStart, scalar * z);
      skewSymmetricToPack.set(row1, col1, 0.0);
      skewSymmetricToPack.set(row1, col2, -scalar * x);

      skewSymmetricToPack.set(row2, colStart, -scalar * y);
      skewSymmetricToPack.set(row2, col1, scalar * x);
      skewSymmetricToPack.set(row2, col2, 0.0);
   }

   public static void toSkewSymmetricMatrix(Tuple3DReadOnly vector, Matrix3DBasics skewSymmetricToPack)
   {
      toSkewSymmetricMatrix(vector.getX(), vector.getY(), vector.getZ(), skewSymmetricToPack);
   }

   public static void toSkewSymmetricMatrix(DMatrix1Row vector, Matrix3DBasics skewSymmetricToPack)
   {
      toSkewSymmetricMatrix(vector.get(0), vector.get(1), vector.get(2), skewSymmetricToPack);
   }

   public static void toSkewSymmetricMatrix(double x, double y, double z, Matrix3DBasics skewSymmetricToPack)
   {
      skewSymmetricToPack.setM00(0.0);
      skewSymmetricToPack.setM01(-z);
      skewSymmetricToPack.setM02(y);

      skewSymmetricToPack.setM10(z);
      skewSymmetricToPack.setM11(0.0);
      skewSymmetricToPack.setM12(-x);

      skewSymmetricToPack.setM20(-y);
      skewSymmetricToPack.setM21(x);
      skewSymmetricToPack.setM22(0.0);
   }

   public static void addMatrixBlock(DMatrix1Row dest, int destStartRow, int destStartColumn, DMatrix1Row src)
   {
      addMatrixBlock(dest, destStartRow, destStartColumn, src, 1.0);
   }

   public static void addMatrixBlock(DMatrix1Row dest, int destStartRow, int destStartColumn, DMatrix1Row src, double scale)
   {
      MatrixTools.addMatrixBlock(dest, destStartRow, destStartColumn, src, 0, 0, src.getNumRows(), src.getNumCols(), scale);
   }

   /**
    * <p>
    * Performs the following operation:<br>
    * <br>
    * c = a * b </br>
    * </p>
    * where we are only modifying a block of the c matrix, starting a rowStart, colStart
    *
    * @param a The left matrix in the multiplication operation. Not modified.
    * @param b The right matrix in the multiplication operation. Not modified.
    * @param c Where the results of the operation are stored. Modified.
    */
   public static void multSetBlock(DMatrix1Row a, DMatrix1Row b, DMatrix1Row c, int rowStart, int colStart)
   {
      if (a == c || b == c)
         throw new IllegalArgumentException("Neither 'a' or 'b' can be the same matrix as 'c'");
      else if (a.numCols != b.numRows)
      {
         throw new MatrixDimensionException("The 'a' and 'b' matrices do not have compatible dimensions");
      }

      int aIndexStart = 0;

      for (int i = 0; i < a.numRows; i++)
      {
         for (int j = 0; j < b.numCols; j++)
         {
            double total = 0;

            int indexA = aIndexStart;
            int indexB = j;
            int end = indexA + b.numRows;
            while (indexA < end)
            {
               total += a.data[indexA++] * b.data[indexB];
               indexB += b.numCols;
            }

            int cIndex = (i + rowStart) * c.numCols + j + colStart;
            c.data[cIndex] = total;
         }
         aIndexStart += a.numCols;
      }
   }

   public static void unsafe_add(DMatrix matrix, int row, int col, double value)
   {
      if (matrix instanceof DMatrixRMaj)
         unsafe_add((DMatrixRMaj) matrix, row, col, value);
      else
         matrix.unsafe_set(row, col, value + matrix.unsafe_get(row, col));
   }

   public static void unsafe_add(DMatrixRMaj matrix, int row, int col, double value)
   {
      matrix.data[row * matrix.numCols + col] += value;
   }

   /**
    * Negates the input matrix in-place.
    *
    * @param matrix the matrix to be negated.
    */
   public static void negate(DMatrixRMaj matrix)
   {
      CommonOps_DDRM.scale(-1.0, matrix);
   }
}
