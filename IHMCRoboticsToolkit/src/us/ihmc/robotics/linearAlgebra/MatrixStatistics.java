package us.ihmc.robotics.linearAlgebra;

import Jama.Matrix;

//
//Matrix manipulation
//

public class MatrixStatistics
{
   /**
    * Locates the Maximum element of a Matrix.  Returns the
    * indeces of that element
    *
    * @param m Matrix
    * @return int[]
    */
   public static int[] indecesOfMaxElement(Matrix m)
   {
	  // TODO: rename this method to firstIndicesOfMaxElement to better represent functionality as well as fixing the typo.
      // TODO: Consider removing this method, since it is currently not used anywhere in IHMC code.
	  int[] ret = new int[2];
      double max = Double.NEGATIVE_INFINITY;
      for (int i = 0; i < m.getRowDimension(); i++)
      {
         for (int j = 0; j < m.getColumnDimension(); j++)
         {
            if (m.get(i, j) > max)
            {
               max = m.get(i, j);
               ret[0] = i;
               ret[1] = j;
            }
         }
      }

      return ret;
   }


   public static Matrix getCovarianceMatrix(Matrix m)
   {
	  // TODO: Suggestion for javadoc comments:
	  // This method assumes sample sets in rows
      int N = m.getColumnDimension();
      Matrix ret = subtractAverageColumnFromEachRow(m);
      ret = ret.times(ret.transpose());
      ret = ret.times(1.0 / (N));

      return ret;
   }

   public static Matrix subtractAverageColumnFromEachRow(Matrix m)
   {
      Matrix u = getAverageColumnVector(m);
      Matrix h = createRowVector(m.getColumnDimension(), 1.0);
      Matrix ret = m.copy();

      // System.out.println("Stats::subtractAverageColumnFromEachRow: u,
      // h, u*h : ");
      // u.print(3, 3);
      // h.print(3, 3);
      // u.times(h).print(3, 3);
      return ret.minus(u.times(h));
   }


   public static double sumAllElements(Matrix m)
   {
      final DoubleWrapper ret = new DoubleWrapper();
      forEachElement(m, new ElementHandler()
      {
         public void handleElement(double value, int rowNumber, int columnNumber)
         {
            ret.val += value;
         }
      });

      return ret.val;
   }

   /**
    * divideEachRowByStdDevOfRow
    *
    * @param m Matrix
    * @return Matrix
    */
   public static Matrix divideEachRowByStdDevOfRow(Matrix m)
   {
      final Matrix variance = getVarianceOfEachRow(m);
      final Matrix ret = m.copy();
      forEachElement(ret, new ElementHandler()
      {
         public void handleElement(double value, int rowNumber, int columnNumber)
         {
            ret.set(rowNumber, columnNumber, value / Math.sqrt(variance.get(rowNumber, 0)));
         }
      });

      return ret;
   }

   /**
    * get the variance of each row in the matrix
    *
    * @param m
    *            Matrix
    * @return Matrix
    */
   public static Matrix getVarianceOfEachRow(Matrix m)
   {
      final Matrix ret = createColumnVector(m.getRowDimension());
      final Matrix mean = getAverageColumnVector(m);
      forEachRow(m, new RowHandler()
      {
         public void handleRow(Matrix row, final int rowNumber)
         {
            final DoubleWrapper total = new DoubleWrapper();
            forEachColumn(row, new ColumnHandler()
            {
               public void handleColumn(Matrix column, int columnNumber)
               {
                  total.val += (column.get(0, 0) - mean.get(rowNumber, 0)) * (column.get(0, 0) - mean.get(rowNumber, 0));
               }
            });
            double var = total.val / (row.getColumnDimension());
            ret.set(rowNumber, 0, var);
         }
      });

      return ret;
   }

   /**
    * get the mean of each row in the matrix
    *
    * @param m
    *            Matrix
    * @return Matrix
    */
   public static Matrix getAverageColumnVector(Matrix m)
   {
      final Matrix ret = createColumnVector(m.getRowDimension());
      forEachRow(m, new RowHandler()
      {
         public void handleRow(Matrix row, int rowNumber)
         {
            final DoubleWrapper total = new DoubleWrapper();
            forEachColumn(row, new ColumnHandler()
            {
               public void handleColumn(Matrix column, int columnNumber)
               {
                  total.val += column.get(0, 0);
               }
            });
            ret.set(rowNumber, 0, total.val / row.getColumnDimension());
         }
      });

      return ret;
   }

   public static Matrix createColumnVector(int numberOfRows, double initialVal)
   {
      return new Matrix(numberOfRows, 1, initialVal);
   }

   public static Matrix createRowVector(int numberOfColumns, double initialVal)
   {
      return new Matrix(1, numberOfColumns, initialVal);
   }

   public static Matrix createColumnVector(int numberOfRows)
   {
      return new Matrix(numberOfRows, 1);
   }

   public static Matrix createRowVector(int numberOfColumns)
   {
      return new Matrix(1, numberOfColumns);
   }

   public static Matrix getRowNumber(int i, Matrix m)
   {
      Matrix ret = m.getMatrix(i, i, 0, m.getColumnDimension() - 1);

      return ret;
   }

   public static Matrix getColumnNumber(int i, Matrix m)
   {
      return m.getMatrix(0, m.getRowDimension() - 1, i, i);
   }

   public static void forEachElement(Matrix m, final ElementHandler elementHandler)
   {
      forEachRow(m, new RowHandler()
      {
         public void handleRow(Matrix row, final int rowNumber)
         {
            forEachColumn(row, new ColumnHandler()
            {
               public void handleColumn(Matrix column, int columnNumber)
               {
                  elementHandler.handleElement(column.get(0, 0), rowNumber, columnNumber);
               }
            });
         }
      });
   }

   public static void forEachColumn(Matrix m, ColumnHandler columnHandler)
   {
      for (int i = 0; i < m.getColumnDimension(); i++)
      {
         columnHandler.handleColumn(getColumnNumber(i, m), i);
      }
   }

   public static void forEachRow(Matrix m, RowHandler rowHandler)
   {
      for (int i = 0; i < m.getRowDimension(); i++)
      {
         rowHandler.handleRow(getRowNumber(i, m), i);
      }
   }

   public static class DoubleWrapper
   {
      public double val;
   }


   public static interface ElementHandler
   {
      public abstract void handleElement(double value, int rowNumber, int columnNumber);
   }


   public static interface RowHandler
   {
      public abstract void handleRow(Matrix row, int rowNumber);
   }


   public static interface ColumnHandler
   {
      public abstract void handleColumn(Matrix column, int columnNumber);
   }
}
