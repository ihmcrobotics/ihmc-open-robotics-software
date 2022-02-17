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
      this.Mat = new double[row][col];
      this.row = row;
      this.col = col;
   }

   public MatrixForML(int row, int col, double[][] matrix)
   {
      this.Mat = new double[row][col];
      this.row = row;
      this.col = col;

      for (int i = 0; i < row; i++)
      {
         for (int j = 0; j < col; j++)
         {
            this.Mat[i][j] = matrix[i][j];
         }
      }
   }
   public int getRow()
   {
      return row;
   }
   
   public int getCol()
   {
      return col;
   }
   
   public double[][] getMat()
   {
      return Mat;
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

   public MatrixForML inverse()
   {
      if (row != col)
      {
         System.out.println("Can't do matrix inverse. (DIM ERROR)");
         return null;
      }
      else
      {
         MatrixForML ret = new MatrixForML(row, col);
         double det = this.determinant(Mat, row);
         ret.set(this.adjoint(row));
         ret = ret.div(det);
         return ret;
      }
   }
   
   public void getCofactor(double[][] mat,double temp[][], int p, int q, int n)
   {
      int i = 0, j = 0;

      for (int row = 0; row < n; row++)
      {
         for (int col = 0; col < n; col++)
         {
            if (row != p && col != q)
            {
               temp[i][j++] = mat[row][col];
               if (j == n - 1)
               {
                  j = 0;
                  i++;
               }
            }
         }
      }
   }
   
   public double determinant(double[][] mat, int n)
   {
      double D = 0; 
      int sign = 1;
      double temp[][] = new double[row][row];
      
      if(row != col)
      {
         System.out.println("Can't calculate matrix determinant. (DIM ERROR)");
         return D;
      }
      
      if (n == 1)
         return mat[0][0];  
      
      for (int f = 0; f < n; f++)
      {
         getCofactor(mat, temp, 0, f, n);
         D += sign * mat[0][f] * determinant(temp, n - 1);

         sign = -sign;
      }

      return D;
   }
   
   
   public double[][] adjoint(int N)
   {
      double[][] adj = new double[row][row];
       if (N == 1)
       {
           adj[0][0] = 1;
           return adj;
       }
    
       int sign = 1;
       double [][]temp = new double[N][N];
    
       for (int i = 0; i < N; i++)
       {
           for (int j = 0; j < N; j++)
           {
               getCofactor(Mat, temp, i, j, N);
    
               sign = ((i + j) % 2 == 0)? 1: -1;
    
               adj[j][i] = (sign)*(determinant(temp, N-1));
           }
       }
       return adj;
   }

   public MatrixForML adj()
   {
      if (row != col)
      {
         System.out.println("Can't make adjoint matrix. (DIM ERROR)");
         return this;
      }
      else
      {
         MatrixForML ret = new MatrixForML(row, col);

         return ret;
      }
   }
   
   public MatrixForML transpose()
   {
      
         MatrixForML ret = new MatrixForML(col, row);
         for (int i = 0 ; i<col ; i++)
         {
            for(int j = 0; j<row ; j ++)
            {
               ret.set(i, j, this.getDoubleValue(j, i));
            }
         }
         
         return ret;
      
   }
   
   public MatrixForML cross(MatrixForML vectorB)
   {
      // this x vectorB
      
      if ((row != 3) || (col != 1) || (vectorB.row != 3) || (vectorB.col != 1))
      {
         System.out.println("Can't calculate cross product. (DIM ERROR)");
         return this;
      }
      
      else
      {
         MatrixForML matA = new MatrixForML(3, 3);
         matA.set(0,0,0);
         matA.set(0,1,-this.getDoubleValue(2, 0));
         matA.set(0,2,this.getDoubleValue(1, 0));
         
         matA.set(1,0,this.getDoubleValue(2, 0));
         matA.set(1,1,0);
         matA.set(1,2,-this.getDoubleValue(0, 0));
         
         matA.set(2,0,-this.getDoubleValue(1, 0));
         matA.set(2,1,this.getDoubleValue(0, 0));
         matA.set(2,2,0);
         
         MatrixForML ret = new MatrixForML(3, 1);
         
         ret = matA.dot(vectorB);
         
         return ret;
      }
   }


}
