package us.ihmc.math;

import Jama.Matrix;
import org.ejml.data.ZMatrixRMaj;
import org.ejml.dense.row.CommonOps_ZDRM;
import org.ejml.simple.SimpleMatrix;

/**
 * This is a matrix that consists of complex numbers. It is stored as a two dimensional, row major array of {@link ComplexNumber}. This class is, in general,
 * garbage free.
 */
public class ComplexMatrix
{
   /**
    * Value holders in this array.
    */
   private final ComplexNumber[][] elements;

   /**
    * Creates a matrix of size specified by {@param numRows} and {@param numColumns} of complex numbers that are of zero magnitude
    *
    * @param numRows    number of rows in the matrix
    * @param numColumns number of columns in the matrix
    */
   public ComplexMatrix(int numRows, int numColumns)
   {
      this.elements = new ComplexNumber[numRows][numColumns];

      for (int i = 0; i < numRows; i++)
      {
         for (int j = 0; j < numColumns; j++)
         {
            elements[i][j] = new ComplexNumber(0.0, 0.0);
         }
      }
   }

   /**
    * Creates a matrix of complex numbers.
    * <p>The number of rows is the length of the first entry in {@param elements}, or {@param elements.length}. </p>
    * <p>The number of columns is the length of the second entry in {@param elements}, or {@param elements[0].length}. </p>
    * <p>The third entry in {@param elements} stores the data for each complex number, and should be of length 2. </p>
    *
    * @param elements container of all the complex number data fields.
    */
   public ComplexMatrix(double[][][] elements)
   {
      int numRows = elements.length;
      int numColumns = elements[0].length;

      if (elements[0][0].length != 2)
         throw new IllegalArgumentException("There should be two values for each entry in the array. There are " + elements[0][0].length);

      this.elements = new ComplexNumber[numRows][numColumns];

      for (int i = 0; i < numRows; i++)
      {
         for (int j = 0; j < numColumns; j++)
         {
            set(i, j, new ComplexNumber(elements[i][j][0], elements[i][j][1]));
         }
      }
   }

   /**
    * Creates a matrix of complex numbers.
    * <p>The number of rows is the length of the first entry in {@param elements}, or {@param elements.length}. </p>
    * <p>The number of columns is the length of the second entry in {@param elements}, or {@param elements[0].length}. </p>
    *
    * @param elements container of all the complex number data fields. These values are deep-copied, and pointers are not maintained.
    */
   public ComplexMatrix(ComplexNumber[][] elements)
   {
      int numRows = elements.length;
      int numColumns = elements[0].length;

      this.elements = new ComplexNumber[numRows][numColumns];

      for (int i = 0; i < numRows; i++)
      {
         for (int j = 0; j < numColumns; j++)
         {
            ComplexNumber complexNumber = elements[i][j];
            if (complexNumber == null)
            {
               complexNumber = new ComplexNumber(0.0, 0.0);
            }

            this.elements[i][j] = complexNumber;
         }
      }
   }

   /**
    * Creates a square matrix of complex numbers that is non-zero on the diagonal, and zero off the diagonal.
    * <p>The length of the array in {@param diagonalElements} determines the square size of this matrix</p>
    *
    * @param diagonalElements container of all the complex number data fields to populate the diagonal elements of this matrix. These values are deep copied,
    *                         and pointers are not maintained.
    * @return matrix of complex numbers with values along the diagonal.
    */
   public static ComplexMatrix constructDiagonalMatrix(ComplexNumber[] diagonalElements)
   {
      int length = diagonalElements.length;
      ComplexMatrix ret = constructZeros(length);

      for (int i = 0; i < length; i++)
      {
         ret.set(i, i, new ComplexNumber(diagonalElements[i]));
      }

      return ret;
   }

   /**
    * Creates a square matrix of size {@param length} of complex numbers that is of real value of 1 only on the diagonal, and zero off the diagonal.
    *
    * @param length square size of the returned matrix
    * @return matrix of complex numbers with unity values along the diagonal.
    */
   public static ComplexMatrix constructIdentity(int length)
   {
      ComplexMatrix ret = new ComplexMatrix(length, length);

      for (int i = 0; i < length; i++)
      {
         ret.set(i, i, new ComplexNumber(1.0, 0.0));
      }

      return ret;
   }

   /**
    * Creates a square matrix of size {@param length} of zero values everywhere.
    *
    * @param length square size of the returned matrix
    * @return matrix of zero complex numbers.
    */
   public static ComplexMatrix constructZeros(int length)
   {
      return new ComplexMatrix(length, length);
   }

   /**
    * Sets the entry in this matrix at position ({@param row}, @{param column}) to be {@param complexNumber}
    * @param row row entry to set
    * @param column column entry to set
    * @param complexNumber data container to set. This value is not copied, and the link is maintained.
    */
   public void set(int row, int column, ComplexNumber complexNumber)
   {
      elements[row][column] = complexNumber;
   }

   /**
    * Returns the complex number stored in this matrix at position ({@param row}, @{param column})
    * @param row row entry to set
    * @param column column entry to set
    * @return complex number contained at this entry.
    */
   public ComplexNumber get(int row, int column)
   {
      return elements[row][column];
   }

   /**
    * Computes the inverse of this matrix, and returns the result. It uses EJML tools to compute this inverse.
    */
   public ComplexMatrix inverse()
   {
      if (this.getRowDimension() != this.getColumnDimension())
      {
         throw new RuntimeException("Can only call inverse on square, invertible matrices!");
      }

      ZMatrixRMaj ejmlMatrix = ComplexTools.ihmcComplexToEjmlComplex(this);

      CommonOps_ZDRM.invert(ejmlMatrix);

      return ComplexTools.ejmlToIhmComplex(ejmlMatrix);
   }

   /**
    * Performs an element-wise multplication of this matrix by another complex number.
    * @param complexNumber complex number to multiply each element by.
    * @return resulting scaled matrix
    */
   public ComplexMatrix times(ComplexNumber complexNumber)
   {
      int numRows = getRowDimension();
      int numColumns = getColumnDimension();

      ComplexNumber[][] newElements = new ComplexNumber[numRows][numColumns];
      for (int i = 0; i < numRows; i++)
      {
         for (int j = 0; j < numColumns; j++)
         {
            newElements[i][j] = elements[i][j].times(complexNumber);
         }
      }

      return new ComplexMatrix(newElements);
   }

   /**
    * Performs an element-wise multplication of this matrix by another scalar number.
    * @param scalar scalar to multiply each element by.
    * @return resulting scaled matrix
    */
   public ComplexMatrix times(double scalar)
   {
      int numRows = getRowDimension();
      int numColumns = getColumnDimension();

      ComplexNumber[][] newElements = new ComplexNumber[numRows][numColumns];
      for (int i = 0; i < numRows; i++)
      {
         for (int j = 0; j < numColumns; j++)
         {
            newElements[i][j] = elements[i][j].times(scalar);
         }
      }

      return new ComplexMatrix(newElements);
   }

   /**
    * Subtracts a matrix of real-only values from this matrix, and returns the result
    * @param matrixA matrix containing real values to subtract
    * @return result = this - matrixA
    */
   public ComplexMatrix minus(SimpleMatrix matrixA)
   {
      int numRows = getRowDimension();
      int numColumns = getColumnDimension();

      ComplexNumber[][] newElements = new ComplexNumber[numRows][numColumns];
      for (int i = 0; i < numRows; i++)
      {
         for (int j = 0; j < numColumns; j++)
         {
            newElements[i][j] = elements[i][j].minus(matrixA.get(i, j));
         }
      }

      return new ComplexMatrix(newElements);
   }

   /**
    * Muliplies this matrix by another complex matrix, and returns the result.
    * @param complexMatrix complex number to multiply each element by.
    * @return resulting  matrix, result = this * other
    */
   public ComplexMatrix times(ComplexMatrix complexMatrix)
   {
      int numRows = this.getRowDimension();
      int numColumns = complexMatrix.getColumnDimension();

      int innerDimension = this.getColumnDimension();
      if (innerDimension != complexMatrix.getRowDimension())
      {
         throw new RuntimeException("ComplexMatrix.times() called with two matrices with differing inner dimenstions!");
      }

      ComplexNumber[][] newElements = new ComplexNumber[numRows][numColumns];

      for (int i = 0; i < numRows; i++)
      {
         for (int j = 0; j < numColumns; j++)
         {
            newElements[i][j] = new ComplexNumber(0.0, 0.0);

            for (int k = 0; k < innerDimension; k++)
            {
               newElements[i][j] = newElements[i][j].plus(this.get(i, k).times(complexMatrix.get(k, j)));
            }
         }
      }

      return new ComplexMatrix(newElements);
   }

   /**
    * Transposes this matrix and returns the result
    * @return transposed matrix
    */
   public ComplexMatrix transpose()
   {
      int numColumns = this.getColumnDimension();
      int numRows = this.getRowDimension();

      ComplexNumber[][] newElements = new ComplexNumber[numColumns][numRows];

      for (int i = 0; i < numRows; i++)
      {
         for (int j = 0; j < numColumns; j++)
         {
            newElements[j][i] = new ComplexNumber(elements[i][j].real(), elements[i][j].imaginary());
         }
      }

      return new ComplexMatrix(newElements);
   }

   /**
    * Performs an element-wise check to see if this matrix equals another matrix.
    * @param complexMatrix matrix to check if equals
    * @param epsilon epsilon tolerance of which to define equals
    * @return {@code true} if equal, {@code false} if not.
    */
   public boolean epsilonEquals(ComplexMatrix complexMatrix, double epsilon)
   {
      int numRows = complexMatrix.getRowDimension();
      int numColumns = complexMatrix.getColumnDimension();

      if (this.getRowDimension() != numRows)
         return false;
      if (this.getColumnDimension() != numColumns)
         return false;

      for (int i = 0; i < numRows; i++)
      {
         for (int j = 0; j < numColumns; j++)
         {
            ComplexNumber thisComplexNumber = this.get(i, j);
            ComplexNumber otherComplexNumber = complexMatrix.get(i, j);

            if (!thisComplexNumber.epsilonEquals(otherComplexNumber, epsilon))
               return false;
         }
      }

      return true;
   }

   /**
    * Performs an element-wise check to see if this matrix equals another matrix that is defined as real value only
    * @param matrix matrix to check if equals
    * @param epsilon epsilon tolerance of which to define equals
    * @return {@code true} if equal, {@code false} if not.
    */
   public boolean epsilonEquals(Matrix matrix, double epsilon)
   {
      int numRows = matrix.getRowDimension();
      int numColumns = matrix.getColumnDimension();

      if (this.getRowDimension() != numRows)
         return false;
      if (this.getColumnDimension() != numColumns)
         return false;

      for (int i = 0; i < numRows; i++)
      {
         for (int j = 0; j < numColumns; j++)
         {
            if (!this.get(i, j).epsilonEquals(matrix.get(i, j), epsilon))
               return false;
         }
      }

      return true;
   }

   /**
    * @return number of rows in this matrix
    */
   public int getRowDimension()
   {
      return elements.length;
   }

   /**
    * @return number of columns in this matrix
    */
   public int getColumnDimension()
   {
      return elements[0].length;
   }

   public String toString()
   {
      StringBuilder builder = new StringBuilder();

      for (int i = 0; i < elements.length; i++)
      {
         for (int j = 0; j < elements[i].length; j++)
         {
            builder.append(elements[i][j]);
            if (j < elements[i].length - 1)
               builder.append(", ");
         }

         builder.append("\n");
      }

      builder.append("\n");

      return builder.toString();
   }
}
