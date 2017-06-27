package us.ihmc.robotics.math.frames;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * YoMatrix. Object for holding a matrix of YoVariables so that Matrices can be rewound.
 * Has a maximum number of rows and columns and an actual number of rows and columns.
 * If you set with a smaller matrix, then the actual size will be the size of the
 * passed in matrix.  extra
 * entries will be set to NaN. If you get the contents the matrix you pack must be the
 * correct size.
 * @author JerryPratt
 *
 */
public class YoMatrix
{
   private final int maxNumberOfRows, maxNumberOfColumns;

   private final YoInteger numberOfRows, numberOfColumns;
   private final YoDouble[][] variables;

   public YoMatrix(String name, int maxNumberOfRows, int maxNumberOfColumns, YoVariableRegistry registry)
   {
      this.maxNumberOfRows = maxNumberOfRows;
      this.maxNumberOfColumns = maxNumberOfColumns;

      this.numberOfRows = new YoInteger(name + "NumRows", registry);
      this.numberOfColumns = new YoInteger(name + "NumCols", registry);

      this.numberOfRows.set(maxNumberOfRows);
      this.numberOfColumns.set(maxNumberOfColumns);

      variables = new YoDouble[maxNumberOfRows][maxNumberOfColumns];

      for (int row = 0; row < maxNumberOfRows; row++)
      {
         for (int column = 0; column < maxNumberOfColumns; column++)
         {
            variables[row][column] = new YoDouble(name + "_" + row + "_" + column, registry);
         }
      }
   }

   public void set(DenseMatrix64F matrix)
   {
      int numRows = matrix.getNumRows();
      int numCols = matrix.getNumCols();

      if (((numRows > maxNumberOfRows) || (numCols > maxNumberOfColumns)) && (numRows > 0) && (numCols > 0))
         throw new RuntimeException("Not enough rows or columns. matrix to set is " + matrix.getNumRows() + " by " + matrix.getNumCols());

      this.numberOfRows.set(numRows);
      this.numberOfColumns.set(numCols);

      for (int row = 0; row < maxNumberOfRows; row++)
      {
         for (int column = 0; column < maxNumberOfColumns; column++)
         {
            if ((row < numRows) && (column < numCols))
            {
               variables[row][column].set(matrix.get(row, column));
            }
            else
            {
               variables[row][column].set(Double.NaN);
            }
         }
      }
   }

   public void getAndReshape(DenseMatrix64F matrixToPack)
   {
      matrixToPack.reshape(getNumberOfRows(), getNumberOfColumns());
      get(matrixToPack);
   }

   public void get(DenseMatrix64F matrixToPack)
   {
      int numRows = matrixToPack.getNumRows();
      int numCols = matrixToPack.getNumCols();

      if (((numRows > maxNumberOfRows) || (numCols > maxNumberOfColumns)) && (numRows > 0) && (numCols > 0))
         throw new RuntimeException("Not enough rows or columns. matrixToPack is " + matrixToPack.getNumRows() + " by " + matrixToPack.getNumCols());
      if ((numRows != this.numberOfRows.getIntegerValue()) || (numCols != this.numberOfColumns.getIntegerValue()))
         throw new RuntimeException("Numer of rows and columns must be the same. Call getAndReshape() if you want to reshape the matrixToPack");

      for (int row = 0; row < numRows; row++)
      {
         for (int column = 0; column < numCols; column++)
         {
            matrixToPack.set(row, column, variables[row][column].getDoubleValue());
         }
      }
   }

   public int getNumberOfRows()
   {
      return numberOfRows.getIntegerValue();
   }

   public int getNumberOfColumns()
   {
      return numberOfColumns.getIntegerValue();
   }

   public void setToZero(int numberOfRows, int numberOfColumns)
   {
      if (((numberOfRows > maxNumberOfRows) || (numberOfColumns > maxNumberOfColumns)) && (numberOfRows > 0) && (numberOfColumns > 0))
         throw new RuntimeException("Not enough rows or columns: " + numberOfRows + " by " + numberOfColumns);

      this.numberOfRows.set(numberOfRows);
      this.numberOfColumns.set(numberOfColumns);

      for (int row = 0; row < maxNumberOfRows; row++)
      {
         for (int column = 0; column < maxNumberOfColumns; column++)
         {
            if ((row < numberOfRows) && (column < numberOfColumns))
            {
               variables[row][column].set(0.0);
            }
            else
            {
               variables[row][column].set(Double.NaN);
            }
         }
      }

   }
}
