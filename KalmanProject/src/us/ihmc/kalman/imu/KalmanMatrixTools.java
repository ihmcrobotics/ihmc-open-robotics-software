package us.ihmc.kalman.imu;

public class KalmanMatrixTools
{
   /**
    * setArray: copies a array to dest, returns nothing
    *
    * @param a double[][]
    * @param dest double[][]
    */
   public static void setArray(double[][] a, double[][] dest)
   {
      int m, n;
      if ((m = dest.length) == a.length && (n = dest[0].length) == a[0].length)
      {
         for (int i = 0; i < m; i++)
         {
            for (int j = 0; j < n; j++)
            {
               dest[i][j] = a[i][j];
            }
         }
      }
      else
         System.err.println("MatrixTools.setArray: incompatible dimensions.");
   }

   /**
    * normalize in place, such that a0*a0 + a1*a1 +... + an*an = 1
    * equivalent to div(a,mag(a),a)
    * returns nothing
    *
    * @param a double[][]
    */
   public static void normalize(double[][] a)
   {
      div(a, mag(a), a);
   }

   /**
    * normalize, such that a0*a0 + a1*a1 +... + an*an = 1
    * equivalent to div(a,mag(a),dest)
    * returns nothing
    *
    * @param dest double[][]
    */
   public static void normalize(double[][] a, double[][] dest)
   {
      div(a, mag(a), dest);
   }

   /**
    * mul
    *
    * @param a double[][]
    * @param b double[][]
    * @param dest double[][]
    */
   public static void mul(double[][] a, double[][] b, double[][] dest)
   {
      int m = a.length;
      int n = a[0].length;
      if ((n != b.length) || (m != dest.length) || (b[0].length != dest[0].length))
         System.err.println("MatrixTools.mul: incompatible dimensions.");

      for (int i = 0; i < m; i++)
      {
         for (int j = 0; j < b[0].length; j++)
         {
            dest[i][j] = 0.0;

            for (int k = 0; k < n; k++)
            {
               dest[i][j] += a[i][k] * b[k][j];
            }
         }
      }
   }

   /**
    * mulScalarMul
    *
    * @param a double[][]
    * @param b double[][]
    * @param dest double[][]
    * @param s double
    */
   public static void mulScalarMul(double[][] a, double[][] b, double[][] dest, double s)
   {
      int m = a.length;
      int n = a[0].length;
      if ((n != b.length) || (m != dest.length) || (b[0].length != dest[0].length))
         System.err.println("MatrixTools.mulScalarMul: incompatible dimensions.");

      for (int i = 0; i < m; i++)
      {
         for (int j = 0; j < b[0].length; j++)
         {
            dest[i][j] = 0.0;

            for (int k = 0; k < n; k++)
            {
               dest[i][j] += a[i][k] * b[k][j];
            }

            dest[i][j] *= s;
         }
      }
   }

   /**
    * mul
    *
    * @param a double[][]
    * @param b double
    * @param dest double[][]
    */
   public static void mul(double[][] a, double b, double[][] dest)
   {
      if ((a.length != dest.length) || (a[0].length != dest[0].length))
         System.err.println("MatrixTools.mul: incompatible dimensions.");

      for (int i = 0; i < dest.length; i++)
      {
         for (int j = 0; j < dest[0].length; j++)
         {
            dest[i][j] = a[i][j] * b;
         }
      }
   }

   /**
    * transpose
    *
    * @param a double[][]
    * @param dest double[][]
    */
   public static void transpose(double[][] a, double[][] dest)
   {
      if ((a.length != dest[0].length) || (a[0].length != dest.length))
         System.err.println("MatrixTools.tranpose: incompatible dimensions.");

      for (int i = 0; i < a.length; i++)
      {
         for (int j = 0; j < a[0].length; j++)
         {
            dest[j][i] = a[i][j];
         }
      }
   }

   /**
    * add
    *
    * @param a double[][]
    * @param b double[][]
    * @param dest double[][]
    */
   public static void add(double[][] a, double[][] b, double[][] dest)
   {
      if ((a.length != dest.length) || (b[0].length != dest[0].length))
         System.err.println("MatrixTools.add: incompatible dimensions.");

      for (int i = 0; i < dest.length; i++)
      {
         for (int j = 0; j < dest[0].length; j++)
         {
            dest[i][j] = a[i][j] + b[i][j];
         }
      }
   }

   /**
    * sub
    *
    * @param a double[][]
    * @param b double[][]
    * @param dest double[][]
    */
   public static void sub(double[][] a, double[][] b, double[][] dest)
   {
      if ((a.length != dest.length) || (b[0].length != dest[0].length))
         System.err.println("MatrixTools.sub: incompatible dimensions.");

      for (int i = 0; i < dest.length; i++)
      {
         for (int j = 0; j < dest[0].length; j++)
         {
            dest[i][j] = a[i][j] - b[i][j];
         }
      }
   }

   /**
    * div
    *
    * @param a double[][]
    * @param s double
    * @param dest double[][]
    */
   public static void div(double[][] a, double s, double[][] dest)
   {
      if ((a.length != dest.length) || (a[0].length != dest[0].length))
         System.err.println("MatrixTools.add: incompatible dimensions.");

      for (int i = 0; i < dest.length; i++)
      {
         for (int j = 0; j < dest[0].length; j++)
         {
            dest[i][j] = a[i][j] / s;
         }
      }
   }

   /**
    * identity
    *
    * @param a double[][]
    */
   public static void identity(double[][] a)
   {
      for (int i = 0; i < a.length; i++)
      {
         for (int j = 0; j < a[0].length; j++)
         {
            a[i][j] = (i == j) ? 1.0 : 0.0;
         }
      }
   }

   /**
    * trace
    *
    * @param a double[][]
    * @return double
    */
   public static double trace(double[][] a)
   {
      double trace = 0.0;
      for (int i = 0; i < a.length; i++)
      {
         for (int j = 0; j < a[0].length; j++)
         {
            trace += (i == j) ? a[i][j] : 0.0;
         }
      }

      return trace;
   }

   /**
    * zero
    *
    * @param a double[][]
    */
   public static void zero(double[][] a)
   {
      for (int i = 0; i < a.length; i++)
      {
         for (int j = 0; j < a[0].length; j++)
         {
            a[i][j] = 0.0;
         }
      }
   }

   /**
    * mag
    *
    * @param a double[][]
    * @return double
    */
   public static double mag(double[][] a)
   {
      double ret = 0.0;
      for (int i = 0; i < a.length; i++)
      {
         for (int j = 0; j < a[0].length; j++)
         {
            ret += a[i][j] * a[i][j];
         }
      }

      return Math.sqrt(ret);
   }

   // matrix inversion code from: http://www.cvl.iis.u-tokyo.ac.jp/~miyazaki/tech/teche23.html

   /**
    * inverse22
    *
    * @param a double[][]
    * @param dest double[][]
    */
   public static void inverse22(double[][] a, double[][] dest)
   {
      double det = a[0][0] * a[1][1] - a[0][1] * a[2][0];
      if (det == 0.0)
      {
         System.err.println("MatrixTools.inverse22: determinant is zero.");

         return;
      }

      det = 1.0 / det;
      dest[0][0] = det * a[1][1];
      dest[0][1] = -det * a[0][1];
      dest[1][0] = -det * a[1][0];
      dest[1][1] = det * a[0][0];
   }

   /**
    * inverse33
    *
    * @param a double[][]
    * @param dest double[][]
    */
   public static void inverse33(double[][] a, double[][] dest)
   {
      double det = a[0][0] * a[1][1] * a[2][2] + a[1][0] * a[2][1] * a[0][2] + a[2][0] * a[0][1] * a[1][2] - a[0][0] * a[2][1] * a[1][2]
                   - a[2][0] * a[1][1] * a[0][2] - a[1][0] * a[0][1] * a[2][1];
      if (det == 0.0)
      {
         System.err.println("MatrixTools.inverse33: determinant is zero.");

         return;
      }

      det = 1.0 / det;
      dest[0][0] = det * a[1][1] * a[2][2] - a[1][2] * a[2][1];
      dest[0][1] = det * a[0][2] * a[2][1] - a[0][1] * a[2][2];
      dest[0][2] = det * a[0][1] * a[1][2] - a[0][2] * a[1][1];
      dest[1][0] = det * a[1][2] * a[2][0] - a[1][0] * a[2][2];
      dest[1][1] = det * a[0][0] * a[2][2] - a[0][2] * a[2][0];
      dest[1][2] = det * a[0][2] * a[1][0] - a[0][0] * a[1][2];
      dest[2][0] = det * a[1][0] * a[2][1] - a[1][1] * a[2][0];
      dest[2][1] = det * a[0][1] * a[2][0] - a[0][0] * a[2][1];
      dest[2][2] = det * a[0][0] * a[1][1] - a[0][1] * a[1][0];
   }

   /**
    * inverse44
    *
    * @param a double[][]
    * @param dest double[][]
    */
   public static void inverse44(double[][] a, double[][] dest)
   {
      double det = a[0][0] * a[1][1] * a[2][2] * a[3][3] + a[0][0] * a[1][2] * a[2][3] * a[3][1] + a[0][0] * a[1][3] * a[2][1] * a[3][2]
                   + a[0][1] * a[1][0] * a[2][3] * a[3][2] + a[0][1] * a[1][2] * a[2][0] * a[3][3] + a[0][1] * a[1][3] * a[2][2] * a[3][0]
                   + a[0][2] * a[1][0] * a[2][1] * a[3][3] + a[0][2] * a[1][1] * a[2][3] * a[3][0] + a[0][2] * a[1][3] * a[2][0] * a[3][1]
                   + a[0][3] * a[1][0] * a[2][2] * a[3][1] + a[0][3] * a[1][1] * a[2][0] * a[3][2] + a[0][3] * a[1][2] * a[2][1] * a[3][0]
                   - a[0][0] * a[1][1] * a[2][3] * a[3][2] - a[0][0] * a[1][2] * a[2][1] * a[3][3] - a[0][0] * a[1][3] * a[2][2] * a[3][1]
                   - a[0][1] * a[1][0] * a[2][2] * a[3][3] - a[0][1] * a[1][2] * a[2][3] * a[3][0] - a[0][1] * a[1][3] * a[2][0] * a[3][2]
                   - a[0][2] * a[1][0] * a[2][3] * a[3][1] - a[0][2] * a[1][1] * a[2][0] * a[3][3] - a[0][2] * a[1][3] * a[2][1] * a[3][0]
                   - a[0][3] * a[1][0] * a[2][1] * a[3][2] - a[0][3] * a[1][1] * a[2][2] * a[3][0] - a[0][3] * a[1][2] * a[2][0] * a[3][1];
      if (det == 0.0)
      {
         System.err.println("MatrixTools.inverse44: determinant is zero.");

         return;
      }

      det = 1.0 / det;
      dest[0][0] = det
                   * (a[1][1] * a[2][2] * a[3][3] + a[1][2] * a[2][3] * a[3][1] + a[1][3] * a[2][1] * a[3][2] - a[1][1] * a[2][3] * a[3][2]
                      - a[1][2] * a[2][1] * a[3][3] - a[1][3] * a[2][2] * a[3][1]);
      dest[0][1] = det
                   * (a[0][1] * a[2][3] * a[3][2] + a[0][2] * a[2][1] * a[3][3] + a[0][3] * a[2][2] * a[3][1] - a[0][1] * a[2][2] * a[3][3]
                      - a[0][2] * a[2][3] * a[3][1] - a[0][3] * a[2][1] * a[3][2]);
      dest[0][2] = det
                   * (a[0][1] * a[1][2] * a[3][3] + a[0][2] * a[1][3] * a[3][1] + a[0][3] * a[1][1] * a[3][2] - a[0][1] * a[1][3] * a[3][2]
                      - a[0][2] * a[1][1] * a[3][3] - a[0][3] * a[1][2] * a[3][1]);
      dest[0][3] = det
                   * (a[0][1] * a[1][3] * a[2][2] + a[0][2] * a[1][1] * a[2][3] + a[0][3] * a[1][2] * a[2][1] - a[0][1] * a[1][2] * a[2][3]
                      - a[0][2] * a[1][3] * a[2][1] - a[0][3] * a[1][1] * a[2][2]);
      dest[1][0] = det
                   * (a[1][0] * a[2][3] * a[3][2] + a[1][2] * a[2][0] * a[3][3] + a[1][3] * a[2][2] * a[3][0] - a[1][0] * a[2][2] * a[3][3]
                      - a[1][2] * a[2][3] * a[3][0] - a[1][3] * a[2][0] * a[3][2]);
      dest[1][1] = det
                   * (a[0][0] * a[2][2] * a[3][3] + a[0][2] * a[2][3] * a[3][0] + a[0][3] * a[2][0] * a[3][2] - a[0][0] * a[2][3] * a[3][2]
                      - a[0][2] * a[2][0] * a[3][3] - a[0][3] * a[2][2] * a[3][0]);
      dest[1][2] = det
                   * (a[0][0] * a[1][3] * a[3][2] + a[0][2] * a[1][0] * a[3][3] + a[0][3] * a[1][2] * a[3][0] - a[0][0] * a[1][2] * a[3][3]
                      - a[0][2] * a[1][3] * a[3][0] - a[0][3] * a[1][0] * a[3][2]);
      dest[1][3] = det
                   * (a[0][0] * a[1][2] * a[2][3] + a[0][2] * a[1][3] * a[2][0] + a[0][3] * a[1][0] * a[2][2] - a[0][0] * a[1][3] * a[2][2]
                      - a[0][2] * a[1][0] * a[2][3] - a[0][3] * a[1][2] * a[2][0]);
      dest[2][0] = det
                   * (a[1][0] * a[2][1] * a[3][3] + a[1][1] * a[2][3] * a[3][0] + a[1][3] * a[2][0] * a[3][1] - a[1][0] * a[2][3] * a[3][1]
                      - a[1][1] * a[2][0] * a[3][3] - a[1][3] * a[2][1] * a[3][0]);
      dest[2][1] = det
                   * (a[0][0] * a[2][3] * a[3][1] + a[0][1] * a[2][0] * a[3][3] + a[0][3] * a[2][1] * a[3][0] - a[0][0] * a[2][1] * a[3][3]
                      - a[0][1] * a[2][3] * a[3][0] - a[0][3] * a[2][0] * a[3][1]);
      dest[2][2] = det
                   * (a[0][0] * a[1][1] * a[3][3] + a[0][1] * a[1][3] * a[3][0] + a[0][3] * a[1][0] * a[3][1] - a[0][0] * a[1][3] * a[3][1]
                      - a[0][1] * a[1][0] * a[3][3] - a[0][3] * a[1][1] * a[3][0]);
      dest[2][3] = det
                   * (a[0][0] * a[1][3] * a[2][1] + a[0][1] * a[1][0] * a[2][3] + a[0][3] * a[1][1] * a[2][0] - a[0][0] * a[1][1] * a[2][3]
                      - a[0][1] * a[1][3] * a[2][0] - a[0][3] * a[1][0] * a[2][1]);
      dest[3][0] = det
                   * (a[1][0] * a[2][2] * a[3][1] + a[1][1] * a[2][0] * a[3][2] + a[1][2] * a[2][1] * a[3][0] - a[1][0] * a[2][1] * a[3][2]
                      - a[1][1] * a[2][2] * a[3][0] - a[1][2] * a[2][0] * a[3][1]);
      dest[3][1] = det
                   * (a[0][0] * a[2][1] * a[3][2] + a[0][1] * a[2][2] * a[3][0] + a[0][2] * a[2][0] * a[3][1] - a[0][0] * a[2][2] * a[3][1]
                      - a[0][1] * a[2][0] * a[3][2] - a[0][2] * a[2][1] * a[3][0]);
      dest[3][2] = det
                   * (a[0][0] * a[1][2] * a[3][1] + a[0][1] * a[1][0] * a[3][2] + a[0][2] * a[1][1] * a[3][0] - a[0][0] * a[1][1] * a[3][2]
                      - a[0][1] * a[1][2] * a[3][0] - a[0][2] * a[1][0] * a[3][1]);
      dest[3][3] = det
                   * (a[0][0] * a[1][1] * a[2][2] + a[0][1] * a[1][2] * a[2][0] + a[0][2] * a[1][0] * a[2][1] - a[0][0] * a[1][2] * a[2][1]
                      - a[0][1] * a[1][0] * a[2][2] - a[0][2] * a[1][1] * a[2][0]);
   }

}
