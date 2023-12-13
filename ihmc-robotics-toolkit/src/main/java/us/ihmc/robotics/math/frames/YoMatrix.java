package us.ihmc.robotics.math.frames;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * YoMatrix. Object for holding a matrix of YoVariables so that Matrices can be rewound. Has a
 * maximum number of rows and columns and an actual number of rows and columns. If you set with a
 * smaller matrix, then the actual size will be the size of the passed in matrix. extra entries will
 * be set to NaN. If you get the contents the matrix you pack must be the correct size.
 * 
 * @author JerryPratt
 */
public class YoMatrix
{
   private final int maxNumberOfRows, maxNumberOfColumns;

   private final YoInteger numberOfRows, numberOfColumns;
   private final YoDouble[][] variables;

   /**
    * Create a YoMatrix with the given name, number of rows, and number of columns. The constituent YoDoubles are named by index.
    *
    * @param name               common name component of the YoMatrix entries.
    * @param maxNumberOfRows    maximum number of rows in the YoMatrix.
    * @param maxNumberOfColumns maximum number of columns in the YoMatrix.
    * @param registry           YoRegistry to register the YoMatrix with.
    */
   public YoMatrix(String name, int maxNumberOfRows, int maxNumberOfColumns, YoRegistry registry)
   {
      this(name, maxNumberOfRows, maxNumberOfColumns, null, null, registry);
   }

   /**
    * Create a YoMatrix with the given name, number of rows, and number of columns. The constituent YoDoubles are named by row name.
    * <p>
    * The number of columns must be equal to 1, that is, the YoMatrix must be a column vector. Otherwise, it is difficult to provide an API that will name the
    * YoDoubles uniquely.
    * </p>
    *
    * @param name               common name component of the YoMatrix entries.
    * @param maxNumberOfRows    maximum number of rows in the YoMatrix.
    * @param maxNumberOfColumns maximum number of columns in the YoMatrix.
    * @param rowNames           names of the rows.
    * @param registry           YoRegistry to register the YoMatrix with.
    */
   public YoMatrix(String name, int maxNumberOfRows, int maxNumberOfColumns, String[] rowNames, YoRegistry registry)
   {
      this(name, maxNumberOfRows, maxNumberOfColumns, rowNames, null, registry);
   }

   /**
    * Create a YoMatrix with the given name, number of rows, and number of columns. The constituent YoDoubles are named by the entries in {@code rowNames} and
    * {@code columnNames}.
    * <p>
    * NOTE: the entries in {@code rowNames} and {@code columnNames} must be unique. Otherwise, the YoDoubles will not have unique names.
    * </p>
    *
    * @param name               common name component of the YoMatrix entries.
    * @param maxNumberOfRows    maximum number of rows in the YoMatrix.
    * @param maxNumberOfColumns maximum number of columns in the YoMatrix.
    * @param rowNames           names of the rows.
    * @param columnNames        names of the columns.
    * @param registry           YoRegistry to register the YoMatrix with.
    */
   public YoMatrix(String name, int maxNumberOfRows, int maxNumberOfColumns, String[] rowNames, String[] columnNames, YoRegistry registry)
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
            switch (checkNames(rowNames, columnNames))
            {
               case NONE:
               {
                  variables[row][column] = new YoDouble(name + "_" + row + "_" + column, registry);  // names are simply the row and column indices
                  break;
               }
               case ROWS:
               {
                  if (maxNumberOfColumns > 1)
                     throw new IllegalArgumentException("The YoMatrix must be a column vector if only row names are provided, else unique names cannot be generated.");

                  variables[row][column] = new YoDouble(name + "_" + rowNames[row], registry);  // names are the row names, no column identifier
                  break;
               }
               case ROWS_AND_COLUMNS:
               {
                  variables[row][column] = new YoDouble(name + "_" + rowNames[row] + "_" + columnNames[column], registry);  // names are the row and column names
                  break;
               }
            }
         }
      }
   }

   private enum NamesProvided
   {
      NONE, ROWS, ROWS_AND_COLUMNS
   }

   private NamesProvided checkNames(String[] rowNames, String[] columnNames)
   {
      if (rowNames == null && columnNames == null)
         return NamesProvided.NONE;
      else if (rowNames != null && columnNames == null)
         return NamesProvided.ROWS;
      else
         return NamesProvided.ROWS_AND_COLUMNS;
   }

   public void set(DMatrixRMaj matrix)
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

   public void getAndReshape(DMatrixRMaj matrixToPack)
   {
      matrixToPack.reshape(getNumberOfRows(), getNumberOfColumns());
      get(matrixToPack);
   }

   public void get(DMatrixRMaj matrixToPack)
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

   public void setToNaN(int numberOfRows, int numberOfColumns)
   {
      if (((numberOfRows > maxNumberOfRows) || (numberOfColumns > maxNumberOfColumns)) && (numberOfRows > 0) && (numberOfColumns > 0))
         throw new RuntimeException("Not enough rows or columns: " + numberOfRows + " by " + numberOfColumns);

      this.numberOfRows.set(numberOfRows);
      this.numberOfColumns.set(numberOfColumns);

      for (int row = 0; row < maxNumberOfRows; row++)
      {
         for (int column = 0; column < maxNumberOfColumns; column++)
         {
            variables[row][column].set(Double.NaN);
         }
      }
   }
}
