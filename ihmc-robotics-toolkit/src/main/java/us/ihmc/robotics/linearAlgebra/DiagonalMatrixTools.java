package us.ihmc.robotics.linearAlgebra;

import java.util.Arrays;

import org.ejml.data.RowD1Matrix64F;
import org.ejml.ops.MatrixDimensionException;
import us.ihmc.commons.PrintTools;

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
    * c = c + a * b <br>
    * <br>
    * c<sub>(startRow + i) (startCol + j)</sub> = c<sub>(startRow + i) (startCol + j)</sub> + &sum;<sub>k=1:n</sub> { a<sub>ik</sub> * b<sub>kj</sub>}
    * </p>
    * <p> where we assume that matrix 'a' is a diagonal matrix. </p>
    * <p> The block is added to matrix c starting at startRow, startCol </p>
    * @param a The left matrix in the multiplication operation. Not modified. Assumed to be diagonal.
    * @param b The right matrix in the multiplication operation. Not modified.
    * @param c Where the results of the operation are stored. Modified.
    * @param startRow The row index to start writing to in the block 'c'.
    * @param startCol The col index to start writing to in the block 'c'.
    */
   public static void preMultAddBlock(RowD1Matrix64F a, RowD1Matrix64F b, RowD1Matrix64F c, int startRow, int startCol)
   {
      if( a == c || b == c )
      {
         throw new IllegalArgumentException("Neither 'a' or 'b' can be the same matrix as 'c'");
      }
      else if( a.numCols != b.numRows )
      {
         throw new MatrixDimensionException("The 'a' and 'b' matrices do not have compatible dimensions");
      }


      for (int row = 0; row < Math.min(a.numRows, a.numCols); row++)
      {
         for (int col = 0; col < b.numCols; col++)
         {
            c.unsafe_set(startRow + row, startCol + col, a.unsafe_get(row, row) * b.unsafe_get(row, col));
         }
      }
   }

   /**
    * p>Performs the following operation:<br>
    * <br>
    * c = c + d * a * b <br>
    * <br>
    * c<sub>(startRow + i) (startCol + j)</sub> = c<sub>(startRow + i) (startCol + j)</sub> + d * &sum;<sub>k=1:n</sub> { a<sub>ik</sub> * b<sub>kj</sub>}
    * </p>
    * <p> where we assume that matrix 'a' is a diagonal matrix. </p>
    * <p> The block is added to matrix c starting at startRow, startCol </p>
    * @param a The left matrix in the multiplication operation. Not modified. Assumed to be diagonal.
    * @param b The right matrix in the multiplication operation. Not modified.
    * @param c Where the results of the operation are stored. Modified.
    * @param startRow The row index to start writing to in the block 'c'.
    * @param startCol The col index to start writing to in the block 'c'.
    */
   public static void preMultAddBlock(double d, RowD1Matrix64F a, RowD1Matrix64F b, RowD1Matrix64F c, int startRow, int startCol)
   {
      if( a == c || b == c )
      {
         throw new IllegalArgumentException("Neither 'a' or 'b' can be the same matrix as 'c'");
      }
      else if( a.numCols != b.numRows )
      {
         throw new MatrixDimensionException("The 'a' and 'b' matrices do not have compatible dimensions");
      }

      for (int row = 0; row < Math.min(a.numRows, a.numCols); row++)
      {
         for (int col = 0; col < b.numCols; col++)
         {
            c.unsafe_set(startRow + row, startCol + col, d * a.unsafe_get(row, row) * b.unsafe_get(row, col));
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
    * <p> where we assume that matrix 'b' is a diagonal matrix. </p>
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
    * <p> where we assume that matrix 'b' is a diagonal matrix. </p>
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

   /**
    * <p>Computes the matrix multiplication inner product:<br>
    * <br>
    * c = a<sup>T</sup> * b * a <br>
    * <br>
    * c<sub>ij</sub> = &sum;<sub>k=1:n</sub> { a<sub>ki</sub> * a<sub>kj</sub> * b<sub>k</sub>}
    * </p>
    * <p> where we assume that matrix 'b' is a diagonal matrix. </p>
    * <p>
    * Is faster than using a generic matrix multiplication by taking advantage of symmetry.
    * </p>
    * @param a The matrix being multiplied. Not modified.
    * @param b The inner diagonal matrix in the multiplication. Not Modified.
    * @param c Where the results of the operation are stored. Modified.
    */
   public static void multInner(RowD1Matrix64F a, RowD1Matrix64F b, RowD1Matrix64F c)
   {
      for( int i = 0; i < a.numCols; i++ )
      {
         for( int j = i; j < a.numCols; j++ )
         {
            int indexC1 = i*c.numCols+j;
            int indexC2 = j*c.numCols+i;
            int indexA = i;
            int indexB = j;
            int indexC = 0;
            double sum = 0;
            int end = indexA + a.numRows*a.numCols;
            for( ; indexA < end; indexA += a.numCols, indexB += a.numCols, indexC += (b.numCols + 1) )
            {
               sum += a.data[indexA]*a.data[indexB] * b.data[indexC];
            }
            c.data[indexC1] = c.data[indexC2] = sum;
         }
      }
   }

   /**
    * <p>Computes the matrix multiplication inner product:<br>
    * <br>
    * c = c + a<sup>T</sup> * b * a <br>
    * <br>
    * c<sub>ij</sub> = c<sub>ij</sub> + &sum;<sub>k=1:n</sub> { a<sub>ki</sub> * a<sub>kj</sub> * b<sub>k</sub>}
    * </p>
    * <p> where we assume that matrix 'b' is a diagonal matrix. </p>
    * <p>
    * Is faster than using a generic matrix multiplication by taking advantage of symmetry.
    * </p>
    * @param a The matrix being multiplied. Not modified.
    * @param b The inner diagonal matrix in the multiplication. Not Modified.
    * @param c Where the results of the operation are stored. Modified.
    */
   public static void multAddInner(RowD1Matrix64F a, RowD1Matrix64F b, RowD1Matrix64F c)
   {
      for( int i = 0; i < a.numCols; i++ )
      {
         int j = i;
         int indexC1 = i*c.numCols+j;
         int indexA = i;
         int indexC = 0;
         double sum = 0;
         int end = indexA + a.numRows*a.numCols;
         for( ; indexA < end; indexA += a.numCols,  indexC += (b.numCols + 1) )
         {
            sum += a.data[indexA]*a.data[indexA] * b.data[indexC];
         }
         c.data[indexC1] += sum;
         j++;

         for( ; j < a.numCols; j++ )
         {
            indexC1 = i*c.numCols+j;
            int indexC2 = j*c.numCols+i;
            indexA = i;
            int indexB = j;
            indexC = 0;
            sum = 0;
            end = indexA + a.numRows*a.numCols;
            for( ; indexA < end; indexA += a.numCols, indexB += a.numCols, indexC += (b.numCols + 1) )
            {
               sum += a.data[indexA]*a.data[indexB] * b.data[indexC];
            }
            c.data[indexC1] += sum;
            c.data[indexC2] += sum;
         }
      }
   }

   /**
    * <p>Computes the matrix multiplication inner product:<br>
    * <br>
    * c = c + b * a<sup>T</sup> * a <br>
    * <br>
    * c<sub>ij</sub> = c<sub>ij</sub> + b * &sum;<sub>k=1:n</sub> { a<sub>ki</sub> * a<sub>kj</sub>}
    * </p>
    * <p>
    * Is faster than using a generic matrix multiplication by taking advantage of symmetry.
    * </p>
    * @param a The matrix being multiplied. Not modified.
    * @param b The scalar multiplier of the matrix.
    * @param c Where the results of the operation are stored. Modified.
    */
   public static void multAddInner(RowD1Matrix64F a, double b, RowD1Matrix64F c)
   {
      for( int i = 0; i < a.numCols; i++ )
      {
         int j = i;
         int indexC1 = i*c.numCols+j;
         int indexA = i;
         double sum = 0;
         int end = indexA + a.numRows*a.numCols;
         for( ; indexA < end; indexA += a.numCols)
         {
            sum += a.data[indexA]*a.data[indexA];
         }
         c.data[indexC1] += b*sum;
         j++;

         for( ; j < a.numCols; j++ )
         {
            indexC1 = i*c.numCols+j;
            int indexC2 = j*c.numCols+i;
            indexA = i;
            int indexB = j;
            sum = 0;
            end = indexA + a.numRows*a.numCols;
            for( ; indexA < end; indexA += a.numCols, indexB += a.numCols)
            {
               sum += a.data[indexA]*a.data[indexB];
            }
            sum *= b;
            c.data[indexC1] += sum;
            c.data[indexC2] += sum;
         }
      }
   }

   /**
    * <p>Computes the matrix multiplication inner product:<br>
    * <br>
    * c = c + a<sup>T</sup> * b * a <br>
    * <br>
    * c<sub>(cRowStart + i) (cColStart + j)</sub> = c<sub>(cRowStart + i) (cColStart + j)</sub> + &sum;<sub>k=1:n</sub> { a<sub>ki</sub> * a<sub>kj</sub> * b<sub>k</sub>}
    * </p>
    * <p> where we assume that matrix 'b' is a diagonal matrix. </p>
    * <p> The block is added to matrix 'c' starting at cStartRow, cStartCol </p>
    * <p>
    * Is faster than using a generic matrix multiplication by taking advantage of symmetry.
    * </p>
    * @param a The matrix being multiplied. Not modified.
    * @param b The matrix on the inside of the multiplication. Assumed to be diagonal. Not modified.
    * @param c Where the results of the operation are stored. Modified.
    * @param cRowStart The row index to start writing to in the block 'c'.
    * @param cColStart The col index to start writing to in the block 'c'.
    */
   public static void multAddBlockInner(RowD1Matrix64F a, RowD1Matrix64F b, RowD1Matrix64F c, int cRowStart, int cColStart)
   {
      for( int i = 0; i < a.numCols; i++ )
      {
         int j = i;
         int indexA = i;
         int indexC = 0;
         double sum = 0;
         int end = indexA + a.numRows*a.numCols;
         for( ; indexA < end; indexA += a.numCols,  indexC += (b.numCols + 1) )
         {
            sum += a.data[indexA]*a.data[indexA] * b.data[indexC];
         }
         int indexC1 = (i+cRowStart)*c.numCols+j+cColStart;
         c.data[indexC1] += sum;
         j++;

         for( ; j < a.numCols; j++ )
         {
            indexA = i;
            int indexB = j;
            indexC = 0;
            sum = 0;
            end = indexA + a.numRows*a.numCols;
            for( ; indexA < end; indexA += a.numCols, indexB += a.numCols, indexC += (b.numCols + 1) )
            {
               sum += a.data[indexA]*a.data[indexB] * b.data[indexC];
            }
            indexC1 = (i+cRowStart)*c.numCols+j+cColStart;
            int indexC2 = (j+cRowStart)*c.numCols+i+cColStart; // this one is wrong
            c.data[indexC1] += sum;
            c.data[indexC2] += sum;
         }
      }
   }

   /**
    * <p>Computes the matrix multiplication inner product:<br>
    * <br>
    * c = c + b * a<sup>T</sup> * a <br>
    * <br>
    * c<sub>(cRowStart + i) (cColStart + j)</sub> = c<sub>(cRowStart + i) (cColStart + j)</sub> + b * &sum;<sub>k=1:n</sub> { a<sub>ki</sub> * a<sub>kj</sub> }
    * </p>
    * <p> The block is added to matrix 'c' starting at cStartRow, cStartCol </p>
    * <p>
    * Is faster than using a generic matrix multiplication by taking advantage of symmetry.
    * </p>
    * @param a The matrix being multiplied. Not modified.
    * @param b The scalar multiplier for the inner operation.
    * @param c Where the results of the operation are stored. Modified.
    * @param cRowStart The row index to start writing to in the block 'c'.
    * @param cColStart The col index to start writing to in the block 'c'.
    */
   public static void multAddBlockInner(RowD1Matrix64F a, double b, RowD1Matrix64F c, int cRowStart, int cColStart)
   {
      for( int i = 0; i < a.numCols; i++ )
      {
         int j = i;
         int indexA = i;
         double sum = 0;
         int end = indexA + a.numRows*a.numCols;
         for( ; indexA < end; indexA += a.numCols )
         {
            sum += a.data[indexA]*a.data[indexA];
         }
         int indexC1 = (i+cRowStart)*c.numCols+j+cColStart;
         c.data[indexC1] += b*sum;
         j++;

         for( ; j < a.numCols; j++ )
         {
            indexA = i;
            int indexB = j;
            sum = 0;
            end = indexA + a.numRows*a.numCols;
            for( ; indexA < end; indexA += a.numCols, indexB += a.numCols )
            {
               sum += a.data[indexA]*a.data[indexB];
            }
            indexC1 = (i+cRowStart)*c.numCols+j+cColStart;
            int indexC2 = (j+cRowStart)*c.numCols+i+cColStart;
            sum *= b;
            c.data[indexC1] += sum;
            c.data[indexC2] += sum;
         }
      }
   }

   /**
    * <p>Computes the matrix multiplication inner product:<br>
    * <br>
    * d = d + a * b<sup>T</sup> * c * b <br>
    * <br>
    * d<sub>ij</sub> = d<sub>ij</sub> + b * &sum;<sub>k=1:n</sub> { b<sub>ki</sub> * b<sub>kj</sub> * c<sub>k</sub>}
    * </p>
    * <p> where we assume that matrix 'c' is a diagonal matrix. </p>
    * <p>
    * Is faster than using a generic matrix multiplication by taking advantage of symmetry.
    * </p>
    * @param a The scalar multiplying the inner operation.
    * @param b The matrix being multiplied. Not modified.
    * @param c The matrix on the inside of the multiplication. Assumed to be diagonal. Not modified.
    * @param d Where the results of the operation are stored. Modified.
    */
   public static void multAddInner(double a, RowD1Matrix64F b, RowD1Matrix64F c, RowD1Matrix64F d)
   {
      for( int i = 0; i < b.numCols; i++ )
      {
         int j = i;
         int indexC1 = i*d.numCols+j;
         int indexA = i;
         int indexC = 0;
         double sum = 0;
         int end = indexA + b.numRows*b.numCols;
         for( ; indexA < end; indexA += b.numCols,  indexC += (c.numCols + 1) )
         {
            sum += b.data[indexA]*b.data[indexA] * c.data[indexC];
         }
         d.data[indexC1] += a * sum;
         j++;

         for( ; j < b.numCols; j++ )
         {
            indexC1 = i*d.numCols+j;
            int indexC2 = j*d.numCols+i;
            indexA = i;
            int indexB = j;
            indexC = 0;
            sum = 0;
            end = indexA + b.numRows*b.numCols;
            for( ; indexA < end; indexA += b.numCols, indexB += b.numCols, indexC += (c.numCols + 1) )
            {
               sum += b.data[indexA]*b.data[indexB] * c.data[indexC];
            }
            d.data[indexC1] += a * sum;
            d.data[indexC2] += a * sum;
         }
      }
   }

   /**
    * <p>Computes the matrix multiplication inner product:<br>
    * <br>
    * c = a * b * a<sup>T</sup> <br>
    * <br>
    * c<sub>ij</sub> = c<sub>ij</sub> + b * &sum;<sub>k=1:n</sub> { a<sub>ki</sub> * a<sub>kj</sub>}
    * </p>
    * <p>
    * Is faster than using a generic matrix multiplication by taking advantage of symmetry.
    * </p>
    * @param a The matrix being multiplied. Not modified.
    * @param b The scalar multiplier of the inner product multiplication.
    * @param c Where the results of the operation are stored. Modified.
    */
   public static void multOuter(RowD1Matrix64F a, double b, RowD1Matrix64F c)
   {
      for( int i = 0; i < a.numRows; i++ ) {
         int indexC1 = i*c.numCols+i;
         int indexC2 = indexC1;
         for( int j = i; j < a.numRows; j++ , indexC2 += c.numCols) {
            int indexA = i*a.numCols;
            int indexB = j*a.numCols;
            double sum = 0;
            int end = indexA + a.numCols;
            for( ; indexA < end; indexA++,indexB++ ) {
               sum += a.data[indexA]*a.data[indexB] * b;
            }
            c.data[indexC2] = c.data[indexC1++] = sum;
         }
      }
   }

   /**
    * <p>Computes the matrix multiplication inner product:<br>
    * <br>
    * c = a * b * a<sup>T</sup> <br>
    * <br>
    * c<sub>ij</sub> = &sum;<sub>k=1:n</sub> { a<sub>ki</sub> * a<sub>kj</sub> * b<sub>k</sub>}
    * </p>
    * <p> where we assume that matrix 'b' is a diagonal matrix. </p>
    * <p>
    * Is faster than using a generic matrix multiplication by taking advantage of symmetry.
    * </p>
    * @param a The matrix being multiplied. Not modified.
    * @param b The matrix on the inside of the multiplication. Assumed to be diagonal. Not modified.
    * @param c Where the results of the operation are stored. Modified.
    */
   public static void multOuter(RowD1Matrix64F a, RowD1Matrix64F b, RowD1Matrix64F c)
   {
      for( int i = 0; i < a.numRows; i++ ) {
         int indexC1 = i*c.numCols+i;
         int indexC2 = indexC1;
         for( int j = i; j < a.numRows; j++ , indexC2 += c.numCols) {
            int indexA = i*a.numCols;
            int indexB = j*a.numCols;
            int indexC = 0;
            double sum = 0;
            int end = indexA + a.numCols;
            for( ; indexA < end; indexA++,indexB++, indexC += (b.numCols + 1) ) {
               sum += a.data[indexA]*a.data[indexB] * b.data[indexC];
            }
            c.data[indexC2] = c.data[indexC1++] = sum;
         }
      }
   }

   /**
    * <p>Computes the matrix multiplication inner product:<br>
    * <br>
    * c = b * a<sup>T</sup> * a <br>
    * <br>
    * c<sub>ij</sub> = b * &sum;<sub>k=1:n</sub> { a<sub>ki</sub> * a<sub>kj</sub>}
    * </p>
    * <p>
    * Is faster than using a generic matrix multiplication by taking advantage of symmetry.
    * </p>
    * @param a The matrix being multiplied. Not modified.
    * @param b The scalar multiplier of the outer operation.
    * @param c Where the results of the operation are stored. Modified.
    */
   public static void multInner(RowD1Matrix64F a, double b, RowD1Matrix64F c)
   {
      for( int i = 0; i < a.numCols; i++ )
      {
         for( int j = i; j < a.numCols; j++ )
         {
            int indexC1 = i*c.numCols+j;
            int indexC2 = j*c.numCols+i;
            int indexA = i;
            int indexB = j;
            double sum = 0;
            int end = indexA + a.numRows*a.numCols;
            for( ; indexA < end; indexA += a.numCols, indexB += a.numCols )
            {
               sum += a.data[indexA]*a.data[indexB];
            }
            c.data[indexC1] = c.data[indexC2] = b*sum;
         }
      }
   }

   /**
    * <p>Performs the following operation:<br>
    * <br>
    * c = a * b * c
    * </br>
    * </p>
    * <p>  where we assume that matrix 'b' is a diagonal matrix. </p>
    * @param a The left matrix in the multiplication operation. Not modified.
    * @param b The middle matrix in the multiplication operation. Not modified. Assumed to be diagonal.
    * @param c The right matrix in the multiplication operation. Not modified.
    * @param d Where the results of the operation are stored. Modified.
    */
   public static void innerDiagonalMult(RowD1Matrix64F a, RowD1Matrix64F b, RowD1Matrix64F c, RowD1Matrix64F d)
   {
      if (a == c || b == c || c == d)
         throw new IllegalArgumentException("Neither 'a' or 'b' can be the same matrix as 'c'");
      else if (a.numCols != b.numRows)
         throw new MatrixDimensionException("The 'a' and 'b' matrices do not have compatible dimensions");
      else if (b.numCols != c.numRows)
         throw new MatrixDimensionException("The 'b' and 'c' matrices do not have compatible dimensions");
      else if (a.numRows != d.numRows || c.numCols != d.numCols)
         throw new MatrixDimensionException("The results matrix does not have the desired dimensions");


      int aIndexStart = 0;
      int dIndex = 0;

      for( int i = 0; i < a.numRows; i++ ) {
         for( int j = 0; j < c.numCols; j++ ) {
            double total = 0;

            int indexA = aIndexStart;
            int indexC = j;
            int indexB = 0;
            int end = indexA + c.numRows;
            while( indexA < end )
            {
               total += a.data[indexA++] * c.data[indexC] * b.data[indexB];
               indexC += c.numCols;
               indexB += b.numCols + 1;
            }

            d.data[dIndex++] = total;
         }
         aIndexStart += a.numCols;
      }
   }

   /**
    * <p>Performs the following operation:<br>
    * <br>
    * d = a<sup>T</sup> * b * c
    * </br>
    * </p>
    * <p>  where we assume that matrix 'b' is a diagonal matrix. </p>
    * @param a The left matrix in the multiplication operation. Not modified.
    * @param b The middle matrix in the multiplication operation. Not modified. Assumed to be diagonal.
    * @param c The right matrix in the multiplication operation. Not modified.
    * @param d Where the results of the operation are stored. Modified.
    */
   public static void innerDiagonalMultTransA(RowD1Matrix64F a, RowD1Matrix64F b, RowD1Matrix64F c, RowD1Matrix64F d)
   {
      if (a == c || b == c || c == d)
         throw new IllegalArgumentException("Neither 'a' or 'b' can be the same matrix as 'c'");
      else if (a.numRows != b.numRows)
         throw new MatrixDimensionException("The 'a' and 'b' matrices do not have compatible dimensions");
      else if (b.numCols != c.numRows)
         throw new MatrixDimensionException("The 'b' and 'c' matrices do not have compatible dimensions");
      else if (a.numCols != d.numRows || c.numCols != d.numCols)
         throw new MatrixDimensionException("The results matrix does not have the desired dimensions");


      int dIndex = 0;

      for( int i = 0; i < a.numCols; i++ ) {
         for( int j = 0; j < c.numCols; j++ ) {
            int indexA = i;
            int indexB = 0;
            int indexC = j;

            int end = indexC + c.numRows*c.numCols;

            double total = 0;

            // loop for k
            for(; indexC < end; indexC += c.numCols ) {
               total += a.data[indexA] * c.data[indexC] * b.data[indexB];
               indexA += a.numCols;
               indexB += b.numCols + 1;
            }

            d.data[dIndex++] = total;
         }
      }
   }

   /**
    * <p>Performs the following operation:<br>
    * <br>
    * d = d + a<sup>T</sup> * b * c
    * </br>
    * </p>
    * <p>  where we assume that matrix 'b' is a diagonal matrix. </p>
    * @param a The left matrix in the multiplication operation. Not modified.
    * @param b The middle matrix in the multiplication operation. Not modified. Assumed to be diagonal.
    * @param c The right matrix in the multiplication operation. Not modified.
    * @param d Where the results of the operation are stored. Modified.
    */
   public static void innerDiagonalMultAddTransA(RowD1Matrix64F a, RowD1Matrix64F b, RowD1Matrix64F c, RowD1Matrix64F d)
   {
      if (a == c || b == c || c == d)
         throw new IllegalArgumentException("Neither 'a' or 'b' can be the same matrix as 'c'");
      else if (a.numRows != b.numRows)
         throw new MatrixDimensionException("The 'a' and 'b' matrices do not have compatible dimensions");
      else if (b.numCols != c.numRows)
         throw new MatrixDimensionException("The 'b' and 'c' matrices do not have compatible dimensions");
      else if (a.numCols != d.numRows || c.numCols != d.numCols)
         throw new MatrixDimensionException("The results matrix does not have the desired dimensions");


      int dIndex = 0;

      for( int i = 0; i < a.numCols; i++ ) {
         for( int j = 0; j < c.numCols; j++ ) {
            int indexA = i;
            int indexB = 0;
            int indexC = j;

            int end = indexC + c.numRows*c.numCols;

            double total = 0;

            // loop for k
            for(; indexC < end; indexC += c.numCols ) {
               total += a.data[indexA] * c.data[indexC] * b.data[indexB];
               indexA += a.numCols;
               indexB += b.numCols + 1;
            }

            d.data[dIndex++] += total;
         }
      }
   }



   /**
    * <p>Performs the following operation:<br>
    * <br>
    * d = d + a<sup>T</sup> * b * c
    * </br>
    * </p>
    * <p>  where we assume that matrix 'b' is a diagonal matrix. </p>
    * @param a The left matrix in the multiplication operation. Not modified.
    * @param b The middle matrix in the multiplication operation. Not modified. Assumed to be diagonal.
    * @param c The right matrix in the multiplication operation. Not modified.
    * @param d Where the results of the operation are stored. Modified.
    * @param rowStart the start row to write to of the 'd' matrix.
    * @param colStart the start column to write to fo the 'd' matrix.
    */
   public static void innerDiagonalMultAddBlockTransA(RowD1Matrix64F a, RowD1Matrix64F b, RowD1Matrix64F c, RowD1Matrix64F d, int rowStart, int colStart)
   {
      if (a == c || b == c || c == d)
         throw new IllegalArgumentException("Neither 'a' or 'b' can be the same matrix as 'c'");
      else if (a.numRows != b.numRows)
         throw new MatrixDimensionException("The 'a' and 'b' matrices do not have compatible dimensions");
      else if (b.numCols != c.numRows)
         throw new MatrixDimensionException("The 'b' and 'c' matrices do not have compatible dimensions");


      for( int i = 0; i < a.numCols; i++ )
      {
         for( int j = 0; j < c.numCols; j++ )
         {
            int indexA = i;
            int indexB = 0;
            int indexC = j;

            int end = indexC + c.numRows*c.numCols;

            double total = 0;

            // loop for k
            for(; indexC < end; indexC += c.numCols ) {
               total += a.data[indexA] * c.data[indexC] * b.data[indexB];
               indexA += a.numCols;
               indexB += b.numCols + 1;
            }

            int dIndex = (i+rowStart)*d.numCols + j + colStart;
            d.data[dIndex] += total;
         }
      }
   }

   /**
    * <p>Performs the following operation:<br>
    * <br>
    * e = e + a * b<sup>T</sup> * c * d
    * </br>
    * </p>
    * <p>  where we assume that matrix 'c' is a diagonal matrix. </p>
    * @param a The scalar multiplier of the matrix operation.
    * @param b The left matrix in the multiplication operation. Not modified.
    * @param c The middle matrix in the multiplication operation. Not modified. Assumed to be diagonal.
    * @param d The right matrix in the multiplication operation. Not modified.
    * @param e Where the results of the operation are stored. Modified.
    * @param rowStart The start row of matrix 'e' to write to.
    * @param colStart The start col of matrix 'e' to write to.
    */
   public static void innerDiagonalMultAddBlockTransA(double a, RowD1Matrix64F b, RowD1Matrix64F c, RowD1Matrix64F d, RowD1Matrix64F e, int rowStart, int colStart)
   {
      if (b == d || c == d || d == e)
         throw new IllegalArgumentException("Neither 'b' or 'c' can be the same matrix as 'd'");
      else if (b.numRows != c.numRows)
         throw new MatrixDimensionException("The 'b' and 'c' matrices do not have compatible dimensions");
      else if (c.numCols != d.numRows)
         throw new MatrixDimensionException("The 'c' and 'd' matrices do not have compatible dimensions");


      for( int i = 0; i < b.numCols; i++ )
      {
         for( int j = 0; j < d.numCols; j++ )
         {
            int indexA = i;
            int indexB = 0;
            int indexC = j;

            int end = indexC + d.numRows*d.numCols;

            double total = 0;

            // loop for k
            for(; indexC < end; indexC += d.numCols ) {
               total += b.data[indexA] * d.data[indexC] * c.data[indexB];
               indexA += b.numCols;
               indexB += c.numCols + 1;
            }

            int dIndex = (i+rowStart)*e.numCols + j + colStart;
            e.data[dIndex] += a * total;
         }
      }
   }
}
