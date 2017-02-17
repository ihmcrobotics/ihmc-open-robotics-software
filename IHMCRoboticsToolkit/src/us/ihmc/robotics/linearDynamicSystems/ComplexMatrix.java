package us.ihmc.robotics.linearDynamicSystems;

import org.ejml.data.CDenseMatrix64F;
import org.ejml.ops.CCommonOps;

import Jama.Matrix;
import us.ihmc.robotics.dataStructures.ComplexNumber;

public class ComplexMatrix
{
   private final ComplexNumber[][] elements;

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

   public ComplexMatrix(double[][][] elements)
   {
      int numRows = elements.length;
      int numColumns = elements[0].length;

      this.elements = new ComplexNumber[numRows][numColumns];

      for (int i = 0; i < numRows; i++)
      {
         for (int j = 0; j < numColumns; j++)
         {
            set(i, j, new ComplexNumber(elements[i][j][0], elements[i][j][1]));
         }
      }
   }

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

            this.elements[i][j] = elements[i][j];
         }
      }
   }

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

   public static ComplexMatrix constructIdentity(int length)
   {
      ComplexMatrix ret = new ComplexMatrix(length, length);

      for (int i = 0; i < length; i++)
      {
         ret.set(i, i, new ComplexNumber(1.0, 0.0));
      }

      return ret;
   }

   public static ComplexMatrix constructZeros(int length)
   {
      ComplexMatrix ret = new ComplexMatrix(length, length);

      for (int i = 0; i < length; i++)
      {
         ret.set(i, i, new ComplexNumber(0.0, 0.0));
      }

      return ret;
   }

   public void set(int row, int column, ComplexNumber complexNumber)
   {
      elements[row][column] = complexNumber;
   }

   public ComplexNumber get(int row, int column)
   {
      return elements[row][column];
   }

   /**
    * Uses EJML. @pabeles
    */
   public ComplexMatrix inverse()
   {
      if (this.getRowDimension() != this.getColumnDimension())
      {
         throw new RuntimeException("Can only call inverse on square, invertible matrices!");
      }

      CDenseMatrix64F ejmlMatrix = ComplexTools.ihmcComplexToEjmlComplex(this);
      
      CCommonOps.invert(ejmlMatrix);
      
      return ComplexTools.ejmlToIhmComplex(ejmlMatrix);
   }

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

   public ComplexMatrix times(double timesBy)
   {
      int numRows = getRowDimension();
      int numColumns = getColumnDimension();

      ComplexNumber[][] newElements = new ComplexNumber[numRows][numColumns];
      for (int i = 0; i < numRows; i++)
      {
         for (int j = 0; j < numColumns; j++)
         {
            newElements[i][j] = elements[i][j].times(timesBy);
         }
      }

      return new ComplexMatrix(newElements);
   }

   public ComplexMatrix minus(Matrix matrixA)
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

   public ComplexMatrix transpose()
   {
      int numColumns = this.getColumnDimension();
      int numRows = this.getRowDimension();

      ComplexNumber[][] newElements = new ComplexNumber[numColumns][numRows];

      for (int i = 0; i < numRows; i++)
      {
         for (int j = 0; j < numColumns; j++)
         {
            newElements[j][i] = new ComplexNumber(elements[i][j].real(), elements[i][j].imag());
         }
      }

      return new ComplexMatrix(newElements);
   }


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

   public int getRowDimension()
   {
      return elements.length;
   }

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
