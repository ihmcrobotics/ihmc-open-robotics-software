package us.ihmc.exampleSimulations.singgleLeggedRobot;

public class MatrixForML
{
   private double[][] Mat;
   private int row;
   private int col;

   public MatrixForML()
   {

   }

   public MatrixForML(int row, int col)
   {
      Mat = new double[row][col];
      this.row = row;
      this.col = col;
   }

   public MatrixForML(int row, int col, double[][] matrix)
   {
      Mat = new double[row][col];
      this.row = row;
      this.col = col;

      for (int i = 0; i < row; i++)
      {
         for (int j = 0; j < col; j++)
         {
            Mat[row][col] = matrix[row][col];
         }
      }
   }

   public void set(double[][] matrix)
   {
      for (int i = 0; i < row; i++)
      {
         for (int j = 0; j < col; j++)
         {
            Mat[i][j] = matrix[i][j];
         }
      }
   }

   public void toIdentityMatrix()
   {
      for (int i = 0; i < col; i++)
      {
         Mat[i][i] = 1.0;
      }
   }

   public void set(int row, int col, double value)
   {
      Mat[row][col] = value;
   }

   public void printMat()
   {
      for (int i = 0; i < row; i++)
      {
         for (int j = 0; j < col; j++)
         {
            System.out.print(getDoubleValue(i, j) + " ");
         }
         System.out.println("");
      }
   }

   public double getDoubleValue(int i, int j)
   {
      return Mat[i][j];
   }

   public MatrixForML dot(MatrixForML matrix)
   {
      if (col != matrix.row)
      {
         System.out.println("Can't do matrix dot product. (DIM ERROR)");
         return null;
      }
      else
      {
         MatrixForML ret = new MatrixForML(row, matrix.col);
         for (int i = 0; i < row; i++)
         {
            for (int j = 0; j < matrix.col; j++)
            {
               double tempValue = 0.0;
               for (int k = 0; k < col; k++)
               {
                  tempValue += Mat[i][k] * matrix.getDoubleValue(k, j);
               }
               ret.set(i, j, tempValue);
            }
         }
         return ret;
      }
   }

   public MatrixForML add(MatrixForML matrix)
   {
      if ((col != matrix.col) || (row != matrix.row))
      {
         System.out.println("Can't do matrix add. (DIM ERROR)");
         return null;
      }
      else
      {
         MatrixForML ret = new MatrixForML(row, col);
         for (int i = 0; i < row; i++)
         {
            for (int j = 0; j < col; j++)
            {
               double tempValue = 0.0;
               tempValue = Mat[i][j] + matrix.getDoubleValue(i, j);
               ret.set(i, j, tempValue);
            }
         }
         return ret;
      }
   }

   public MatrixForML sub(MatrixForML matrix)
   {
      if ((col != matrix.col) || (row != matrix.row))
      {
         System.out.println("Can't do matrix add. (DIM ERROR)");
         return null;
      }
      else
      {
         MatrixForML ret = new MatrixForML(row, col);
         for (int i = 0; i < row; i++)
         {
            for (int j = 0; j < col; j++)
            {
               double tempValue = 0.0;
               tempValue = Mat[i][j] - matrix.getDoubleValue(i, j);
               ret.set(i, j, tempValue);
            }
         }
         return ret;
      }
   }

   public MatrixForML mul(double scalar)
   {

      MatrixForML ret = new MatrixForML(row, col);
      for (int i = 0; i < row; i++)
      {
         for (int j = 0; j < col; j++)
         {
            double tempValue = 0.0;
            tempValue = Mat[i][j] * scalar;
            ret.set(i, j, tempValue);
         }
      }
      return ret;

   }
   
   public MatrixForML div(double scalar)
   {

      MatrixForML ret = new MatrixForML(row, col);
      for (int i = 0; i < row; i++)
      {
         for (int j = 0; j < col; j++)
         {
            double tempValue = 0.0;
            tempValue = Mat[i][j] / scalar;
            ret.set(i, j, tempValue);
         }
      }
      return ret;

   }
}
