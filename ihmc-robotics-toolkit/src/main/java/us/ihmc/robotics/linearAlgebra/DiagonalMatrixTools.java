package us.ihmc.robotics.linearAlgebra;

import java.util.Arrays;

import org.ejml.data.RowD1Matrix64F;
import org.ejml.ops.MatrixDimensionException;

public class DiagonalMatrixTools
{
   /**
    * Finds the inverse of a diagonal matrix
    * @param matrixToInvert matrix to compute inverse of
    * @param invertedMatrixToPack matrix to store inverse
    */
   public static void invertDiagonalMatrix(RowD1Matrix64F matrixToInvert, RowD1Matrix64F invertedMatrixToPack)
   {
      if(matrixToInvert.numRows != matrixToInvert.numCols)
      {
         throw new MatrixDimensionException( "Diagonal matrix to invert is not square. Number of rows in matrix: " + matrixToInvert.getNumRows() + ", number of"
               + " cols in matrix: " + matrixToInvert.getNumCols() + ".");
      }

      if(invertedMatrixToPack.numRows != matrixToInvert.numRows && invertedMatrixToPack.numCols != matrixToInvert.numCols)
      {
         throw new MatrixDimensionException( "Matrix destination is the wrong size. Number of rows in matrix: " + matrixToInvert.getNumRows() + ", number of"
               + " cols in matrix: " + matrixToInvert.getNumCols() + ".");
      }

      int size = matrixToInvert.getNumRows();
      Arrays.fill(invertedMatrixToPack.data, 0, invertedMatrixToPack.getNumElements(), 0.0);

      for (int index = 0; index < size; index++)
         invertedMatrixToPack.unsafe_set(index, index,  1.0 / matrixToInvert.unsafe_get(index, index));
   }

   /**
    * Finds the inverse of a diagonal matrix
    * @param matrixToInvertAndPack matrix to compute inverse of
    */
   public static void invertDiagonalMatrix(RowD1Matrix64F matrixToInvertAndPack)
   {
      if(matrixToInvertAndPack.numRows != matrixToInvertAndPack.numCols)
      {
         throw new MatrixDimensionException( "Diagonal matrix to invert is not square. Number of rows in matrix: " + matrixToInvertAndPack.getNumRows() + ", number of"
               + " cols in matrix: " + matrixToInvertAndPack.getNumCols() + ".");
      }

      int size = matrixToInvertAndPack.getNumRows();

      for (int index = 0; index < size; index++)
         matrixToInvertAndPack.unsafe_set(index, index,  1.0 / matrixToInvertAndPack.unsafe_get(index, index));
   }

   /**
    * <p>Performs the following operation:<br>
    * <br>
    * c = a * b <br>
    * <br>
    * c<sub>ij</sub> = &sum;<sub>k=1:n</sub> { a<sub>ik</sub> * b<sub>kj</sub>}
    * </p>
    * <p>  where we assume that matrix 'a' is a diagonal matrix. </p>
    * @param a The left matrix in the multiplication operation. Not modified. Assumed to be diagonal.
    * @param b The right matrix in the multiplication operation. Not modified.
    * @param c Where the results of the operation are stored. Modified.
    */
   public static void preMult(RowD1Matrix64F a, RowD1Matrix64F b, RowD1Matrix64F c)
   {
      if( a == c || b == c )
      {
         throw new IllegalArgumentException("Neither 'a' or 'b' can be the same matrix as 'c'");
      }
      else if( a.numCols != b.numRows )
      {
         throw new MatrixDimensionException("The 'a' and 'b' matrices do not have compatible dimensions");
      }
      else if( a.numRows != c.numRows || b.numCols != c.numCols )
      {
         throw new MatrixDimensionException("The results matrix does not have the desired dimensions");
      }

      for (int row = 0; row < Math.min(a.numRows, a.numCols); row++)
      {
         for (int col = 0; col < b.numCols; col++)
         {
            c.unsafe_set(row, col, a.unsafe_get(row, row) * b.unsafe_get(row, col));
         }
      }
   }

   /**
    * <p>Performs the following operation:<br>
    * <br>
    * c = a * b
    * </br>
    * c<sub>ij</sub> = &sum;<sub>k=1:n</sub> { a<sub>ik</sub> * b<sub>kj</sub>}
    * </p>
    * <p>  where we assume that matrix 'b' is a diagonal matrix. </p>
    * @param a The left matrix in the multiplication operation. Not modified.
    * @param b The right matrix in the multiplication operation. Not modified. Assumed to be diagonal.
    * @param c Where the results of the operation are stored. Modified.
    */
   public static void postMult(RowD1Matrix64F a, RowD1Matrix64F b, RowD1Matrix64F c)
   {
      if( a == c || b == c )
      {
         throw new IllegalArgumentException("Neither 'a' or 'b' can be the same matrix as 'c'");
      }
      else if( a.numCols != b.numRows )
      {
         throw new MatrixDimensionException("The 'a' and 'b' matrices do not have compatible dimensions");
      }
      else if( a.numRows != c.numRows || b.numCols != c.numCols )
      {
         throw new MatrixDimensionException("The results matrix does not have the desired dimensions");
      }

      for (int row = 0; row < a.numRows; row++)
      {
         for (int col = 0; col < Math.min(b.numRows, b.numCols); col++)
         {
            c.unsafe_set(row, col, b.unsafe_get(col, col) * a.unsafe_get(row, col));
         }
      }
   }

   /**
    * <p>Performs the following operation:<br>
    * <br>
    * c = a<sup>T</sup> * b
    * </br>
    * </p>
    * <p>  where we assume that matrix 'b' is a diagonal matrix. </p>
    * @param a The left matrix in the multiplication operation. Not modified.
    * @param b The right matrix in the multiplication operation. Not modified. Assumed to be diagonal.
    * @param c Where the results of the operation are stored. Modified.
    */
   public static void postMultTransA(RowD1Matrix64F a, RowD1Matrix64F b, RowD1Matrix64F c)
   {
      if (a == c || b == c)
         throw new IllegalArgumentException("Neither 'a' or 'b' can be the same matrix as 'c'");
      else if (a.numRows != b.numRows)
         throw new MatrixDimensionException("The 'a' and 'b' matrices do not have compatible dimensions");
      else if (a.numCols != c.numRows || b.numCols != c.numCols)
         throw new MatrixDimensionException("The results matrix does not have the desired dimensions");

      int index = 0;
      int bIndex = 0;
      int maxValue = Math.min(b.numRows, b.numCols);
      for( int i = 0; i < maxValue; i++ )
      {
         int index2 = i;

         int end = index + a.numCols;
         while( index < end )
         {
            c.data[index2] = b.data[bIndex] * a.data[index++];
            index2 += c.numCols;
         }
         bIndex += b.numCols + 1;
      }
   }
}
