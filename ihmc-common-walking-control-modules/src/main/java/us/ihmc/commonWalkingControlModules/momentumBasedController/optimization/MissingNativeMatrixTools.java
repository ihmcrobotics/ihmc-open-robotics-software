package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.MatrixDimensionException;
import org.ejml.data.DMatrix1Row;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import us.ihmc.matrixlib.NativeMatrix;

public class MissingNativeMatrixTools
{
   /**
    * Extracts rows from {@code source} and copy them into {@code dest}.
    * <p>
    * The rows are written in consecutive order in {@code dest} regardless of whether the given row
    * indices are ordered or successive.
    * </p>
    *
    * @param source       any N-by-M matrix.
    * @param srcRows      the set of rows indices to be extracted.
    * @param dest         the matrix in which the rows are to be copied over, it should have a number
    *                     of rows at least equal to {@code srcRows.length + destStartRow} and a number
    *                     of columns at least equal to {@code source}'s number of columns.
    * @param destStartRow the index of the first row to start writing at.
    */
   public static void extractRows(NativeMatrix source, int[] srcRows, NativeMatrix dest, int destStartRow)
   {
      for (int srcRow : srcRows)
      {
         dest.insert(source, srcRow, srcRow + 1, 0, source.getNumCols(), destStartRow, 0);
         destStartRow++;
      }
   }

   /**
    * Extracts columns from {@code source} and copy them into {@code dest}.
    * <p>
    * The columns are written in consecutive order in {@code dest} regardless of whether the given row
    * indices are ordered or successive.
    * </p>
    *
    * @param source          any N-by-M matrix.
    * @param srcColumns      the set of columns indices to be extracted.
    * @param dest            the matrix in which the columns are to be copied over, it should have a
    *                        number of columns at least equal to
    *                        {@code srcColumns.length + destStartColumn} and a number of rows at least
    *                        equal to {@code source}'s number of rows.
    * @param destStartColumn the index of the first column to start writing at.
    */
   public static void extractColumns(NativeMatrix source, int[] srcColumns, NativeMatrix dest, int destStartColumn)
   {
      for (int srcColumn : srcColumns)
      {
         dest.insert(source, 0, source.getNumRows(), srcColumn, srcColumn + 1, 0, destStartColumn);
         destStartColumn++;
      }
   }

   public static void extract(NativeMatrix src, int indexes[] , int length , NativeMatrix dst ) {
      if( !MatrixFeatures_DDRM.isVector(dst))
         throw new MatrixDimensionException("Dst must be a vector");
      if( length != dst.getNumElements())
         throw new MatrixDimensionException("Unexpected number of elements in dst vector");

      for (int i = 0; i < length; i++)
      {
         dst.setElement(i, 0, src, indexes[i], 0);
      }
   }
}
