package us.ihmc.kalman.imu;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2004</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class QuaternionBasedArrayFullIMUKalmanFilter implements QuaternionBasedFullIMUKalmanFilter
{
   @SuppressWarnings("unused")
   private static final boolean verbose = true;
   private static final int N = 7;

   private final DoubleYoVariable k_qs, k_qxyz;

   // State Variables:
// public final double[][] Quat = new double[4][1]; // Estimated orientation in quaternions.
// public final double[][] bias = new double[3][1]; // Rate gyro bias offset estimates. The Kalman filter adapts to these.

   private final DoubleYoVariable[] Quat = new DoubleYoVariable[4];    // Estimated orientation in quaternions.
   private final DoubleYoVariable[] bias = new DoubleYoVariable[3];    // Rate gyro bias offset estimates. The Kalman filter adapts to these.

   /*
    * Covariance matrix and covariance matrix derivative are updated
    * every other state step.  This is because the covariance should change
    * at a rate somewhat slower than the dynamics of the system.
    */

// public final double[][] P = new double[N][N]; // Covariance matrix

   private final DoubleYoVariable[][] P = new DoubleYoVariable[N][N];    // Covariance matrix
   private final DoubleYoVariable[][] KCopy = new DoubleYoVariable[N][4];

   private double[][] Pdot = new double[N][N];

   /*
    * A represents the Jacobian of the derivative of the system with respect
    * its states.  We do not allocate the bottom three rows since we know that
    * the derivatives of bias_dot are all zero.
    */
   private final double[][] A = new double[N][N];
   private final double[][] At = new double[N][N];

   /*
    * Q is our estimate noise variance.  It is supposed to be an NxN
    * matrix, but with elements only on the diagonals.  Additionally,
    * since the quaternion has no expected noise (we can't directly measure
    * it), those are zero.  For the gyro, we expect around 5 deg/sec noise,
    * which is 0.08 rad/sec.  The variance is then 0.08^2 ~= 0.0075.
    */
   private final double[][] Q = new double[N][N];    // Noise estimate

   /*
    * R is our measurement noise estimate.  Like Q, it is supposed to be
    * an NxN matrix with elements on the diagonals.  However, since we can
    * not directly measure the gyro bias, we have no estimate for it.
    * We only have an expected noise in the pitch and roll accelerometers
    * and in the compass.
    */
   private final double[][] R = new double[4][4];    // State estimate for angles

   private final double[][] K = new double[N][4];
   private final double[][] Wxq = new double[4][4];
   private final double[] quatError = new double[4];

   private final double dt;    // = .001;
   @SuppressWarnings("unused")
   private static final double PI = Math.PI;

   /*
    * C represents the Jacobian of the measurements of the attitude
    * with respect to the states of the filter.
    */
   private final double[][] C = new double[4][N];
   private final double[][] Ct = new double[N][4];
   private final double[][] E = new double[4][4];
   private final double[][] inverseE = new double[4][4];


   // Temporary storage that does not store state between the updates:

   private final double[] X = new double[N];    // Temporary storage of Quat and bias when doing the compuations.

   // Pool of scratch arrays used by various routines. They do not store state of the Kalman Filter.
   private final double[][] t1 = new double[4][N];
   private final double[][] t2 = new double[4][4];
   private final double[][] t3 = new double[N][4];
   private final double[] t4 = new double[N];
   private final double[][] t5 = new double[N][N];
   private final double[][] t6 = new double[N][N];
   private final double[][] t7 = new double[N][N];
   private final double[][] t8 = new double[N][N];
   private final double[] t9 = new double[4];


   @SuppressWarnings("unused")
   private static final java.text.DecimalFormat fmt = new java.text.DecimalFormat();

   public QuaternionBasedArrayFullIMUKalmanFilter(double dt)
   {
      this(dt, null);
   }

   public QuaternionBasedArrayFullIMUKalmanFilter(double dt, YoVariableRegistry registry)
   {
      k_qs = new DoubleYoVariable("k_qs", registry);
      k_qxyz = new DoubleYoVariable("k_qxyz", registry);

      if (registry != null)
      {
         Quat[0] = new DoubleYoVariable("Quat[0]", registry);
         Quat[1] = new DoubleYoVariable("Quat[1]", registry);
         Quat[2] = new DoubleYoVariable("Quat[2]", registry);
         Quat[3] = new DoubleYoVariable("Quat[3]", registry);

         bias[0] = new DoubleYoVariable("bias[0]", registry);
         bias[1] = new DoubleYoVariable("bias[1]", registry);
         bias[2] = new DoubleYoVariable("bias[2]", registry);

         for (int i = 0; i < N; i++)
         {
            for (int j = 0; j < N; j++)
            {
               P[i][j] = new DoubleYoVariable("P[" + i + "][" + j + "]", registry);
            }
         }

         for (int i = 0; i < N; i++)
         {
            for (int j = 0; j < 4; j++)
            {
               KCopy[i][j] = new DoubleYoVariable("K[" + i + "][" + j + "]", registry);
            }
         }
      }

      else
      {
         Quat[0] = new DoubleYoVariable("Quat[0]", "", null);
         Quat[1] = new DoubleYoVariable("Quat[1]", "", null);
         Quat[2] = new DoubleYoVariable("Quat[2]", "", null);
         Quat[3] = new DoubleYoVariable("Quat[3]", "", null);

         bias[0] = new DoubleYoVariable("bias[0]", "", null);
         bias[1] = new DoubleYoVariable("bias[1]", "", null);
         bias[2] = new DoubleYoVariable("bias[2]", "", null);

         for (int i = 0; i < N; i++)
         {
            for (int j = 0; j < N; j++)
            {
               P[i][j] = new DoubleYoVariable("P[" + i + "][" + j + "]", "", null);
            }
         }
      }

      this.dt = dt;
      reset(P);
   }


   // Private Static Matrix Manipulation Methods.

   @SuppressWarnings("unused")
   private static void setArray(double[][] a, double[][] d)
   {
      int m, n;
      if ((m = a.length) == d.length && (n = a[0].length) == d[0].length)
      {
         for (int i = 0; i < m; i++)
         {
            for (int j = 0; j < n; j++)
            {
               a[i][j] = d[i][j];
            }
         }
      }
      else
         System.err.println("setArray: incompatible dimensions.");
   }

   @SuppressWarnings("unused")
   private static void setArray(double[] a, double[] d)
   {
      int m;
      if ((m = a.length) == d.length)
      {
         for (int i = 0; i < m; i++)
         {
            a[i] = d[i];
         }
      }
      else
         System.err.println("setArray: incompatible dimensions.");
   }


   private static void setArray(DoubleYoVariable[] a, double[] d)
   {
      int m;
      if ((m = a.length) == d.length)
      {
         for (int i = 0; i < m; i++)
         {
            a[i].set(d[i]);
         }
      }
      else
         System.err.println("setArray: incompatible dimensions.");
   }


   @SuppressWarnings("unused")
   private static void normalize(double[][] M)
   {
      double mag = 0;
      double s;
      int m = M.length, n = M[0].length;
      for (int i = 0; i < m; i++)
      {
         for (int j = 0; j < n; j++)
         {
            s = M[i][j];
            mag += s * s;
         }
      }

      Mmul(M, 1.0 / Math.sqrt(mag), M);
   }

   private static void normalize(DoubleYoVariable[] M)
   {
      double mag = 0;
      double s;
      int m = M.length;
      for (int i = 0; i < m; i++)
      {
         s = M[i].getDoubleValue();
         mag += s * s;
      }

      if (mag < 1e-12)
         throw new RuntimeException("mag < 1e-12");
      
      Mmul(M, 1.0 / Math.sqrt(mag), M);
   }


   private static void Mmul(double[][] a, double[][] b, double[][] c)
   {
      int m = a.length;
      int n = a[0].length;
      if ((n != b.length) || (m != c.length) || (b[0].length != c[0].length))
         System.err.println("Mmul: incompatible dimensions.");

      for (int i = 0; i < m; i++)
      {
         for (int j = 0; j < b[0].length; j++)
         {
            c[i][j] = 0.0;

            for (int k = 0; k < n; k++)
            {
               c[i][j] += a[i][k] * b[k][j];
            }
         }
      }
   }

   private static void Mmul(DoubleYoVariable[][] a, double[][] b, double[][] c)
   {
      int m = a.length;
      int n = a[0].length;
      if ((n != b.length) || (m != c.length) || (b[0].length != c[0].length))
         System.err.println("Mmul: incompatible dimensions.");

      for (int i = 0; i < m; i++)
      {
         for (int j = 0; j < b[0].length; j++)
         {
            c[i][j] = 0.0;

            for (int k = 0; k < n; k++)
            {
               c[i][j] += a[i][k].getDoubleValue() * b[k][j];
            }
         }
      }
   }


   private static void Mmul(double[][] a, DoubleYoVariable[][] b, double[][] c)
   {
      int m = a.length;
      int n = a[0].length;
      if ((n != b.length) || (m != c.length) || (b[0].length != c[0].length))
         System.err.println("Mmul: incompatible dimensions.");

      for (int i = 0; i < m; i++)
      {
         for (int j = 0; j < b[0].length; j++)
         {
            c[i][j] = 0.0;

            for (int k = 0; k < n; k++)
            {
               c[i][j] += a[i][k] * b[k][j].getDoubleValue();
            }
         }
      }
   }


   private static void Mmul(double[][] a, double[] b, double[] c)
   {
      int m = a.length;
      int n = a[0].length;
      if ((n != b.length) || (m != c.length))
         System.err.println("Mmul: incompatible dimensions.");

      for (int i = 0; i < m; i++)
      {
         c[i] = 0.0;

         for (int k = 0; k < n; k++)
         {
            c[i] += a[i][k] * b[k];
         }
      }
   }



   @SuppressWarnings("unused")
   private static void MmulScalarMul(double[][] a, double[][] b, double[][] c, double s)
   {
      int m = a.length;
      int n = a[0].length;
      if ((n != b.length) || (m != c.length) || (b[0].length != c[0].length))
         System.err.println("MmulScalarMul: incompatible dimensions.");

      for (int i = 0; i < m; i++)
      {
         for (int j = 0; j < b[0].length; j++)
         {
            c[i][j] = 0.0;

            for (int k = 0; k < n; k++)
            {
               c[i][j] += a[i][k] * b[k][j];
            }

            c[i][j] *= s;
         }
      }
   }

   private static void MmulScalarMul(double[][] a, DoubleYoVariable[] b, double[] c, double s)
   {
      int m = a.length;
      int n = a[0].length;
      if ((n != b.length) || (m != c.length))
         System.err.println("MmulScalarMul: incompatible dimensions.");

      for (int i = 0; i < m; i++)
      {
         c[i] = 0.0;

         for (int k = 0; k < n; k++)
         {
            c[i] += a[i][k] * b[k].getDoubleValue();
         }

         c[i] *= s;
      }
   }


   private static void Mmul(double[][] a, double b, double[][] c)
   {
      if ((a.length != c.length) || (a[0].length != c[0].length))
         System.err.println("Mul: incompatible dimensions.");

      for (int i = 0; i < c.length; i++)
      {
         for (int j = 0; j < c[0].length; j++)
         {
            c[i][j] = a[i][j] * b;
         }
      }
   }

   private static void Mmul(DoubleYoVariable[] a, double b, DoubleYoVariable[] c)
   {
      if (a.length != c.length)
         System.err.println("Mul: incompatible dimensions.");

      for (int i = 0; i < c.length; i++)
      {
         c[i].set(a[i].getDoubleValue() * b);
      }
   }


   private static void Mtranspose(double[][] a, double[][] b)
   {
      if ((a.length != b[0].length) || (a[0].length != b.length))
         System.err.println("Mtranpose: incompatible dimensions.");

      for (int i = 0; i < a.length; i++)
      {
         for (int j = 0; j < a[0].length; j++)
         {
            b[j][i] = a[i][j];
         }
      }
   }

   private static void Madd(double[][] a, double[][] b, double[][] c)
   {
      if ((a.length != c.length) || (b[0].length != c[0].length))
         System.err.println("Madd: incompatible dimensions.");

      for (int i = 0; i < c.length; i++)
      {
         for (int j = 0; j < c[0].length; j++)
         {
            c[i][j] = a[i][j] + b[i][j];
         }
      }
   }

   private static void Madd(DoubleYoVariable[][] a, double[][] b, DoubleYoVariable[][] c)
   {
      if ((a.length != c.length) || (b[0].length != c[0].length))
         System.err.println("Madd: incompatible dimensions.");

      for (int i = 0; i < c.length; i++)
      {
         for (int j = 0; j < c[0].length; j++)
         {
            c[i][j].set(a[i][j].getDoubleValue() + b[i][j]);
         }
      }
   }


   private static void Madd(double[] a, double[] b, double[] c)
   {
      if (a.length != c.length)
         System.err.println("Madd: incompatible dimensions.");

      for (int i = 0; i < c.length; i++)
      {
         c[i] = a[i] + b[i];
      }
   }


   private static void Madd(DoubleYoVariable[] a, double[] b, DoubleYoVariable[] c)
   {
      if (a.length != c.length)
         System.err.println("Madd: incompatible dimensions.");

      for (int i = 0; i < c.length; i++)
      {
         c[i].set(a[i].getDoubleValue() + b[i]);
      }
   }


   @SuppressWarnings("unused")
   private static void Msub(double[][] a, double[][] b, double[][] c)
   {
      if ((a.length != c.length) || (b[0].length != c[0].length))
         System.err.println("Msub: incompatible dimensions.");

      for (int i = 0; i < c.length; i++)
      {
         for (int j = 0; j < c[0].length; j++)
         {
            c[i][j] = a[i][j] - b[i][j];
         }
      }
   }

   private static void Msub(DoubleYoVariable[][] a, double[][] b, DoubleYoVariable[][] c)
   {
      if ((a.length != c.length) || (b[0].length != c[0].length))
         System.err.println("Msub: incompatible dimensions.");

      for (int i = 0; i < c.length; i++)
      {
         for (int j = 0; j < c[0].length; j++)
         {
            c[i][j].set(a[i][j].getDoubleValue() - b[i][j]);
         }
      }
   }


   @SuppressWarnings("unused")
   private static void Msub(double[][] a, DoubleYoVariable[][] b, double[][] c)
   {
      if ((a.length != c.length) || (b[0].length != c[0].length))
         System.err.println("Msub: incompatible dimensions.");

      for (int i = 0; i < c.length; i++)
      {
         for (int j = 0; j < c[0].length; j++)
         {
            c[i][j] = a[i][j] - b[i][j].getDoubleValue();
         }
      }
   }

   private static void Msub(double[] a, DoubleYoVariable[] b, double[] c)
   {
      if (a.length != c.length)
         System.err.println("Msub: incompatible dimensions.");

      for (int i = 0; i < c.length; i++)
      {
         c[i] = a[i] - b[i].getDoubleValue();
      }
   }



   private static void Midentity(double[][] a)
   {
      for (int i = 0; i < a.length; i++)
      {
         for (int j = 0; j < a[0].length; j++)
         {
            a[i][j] = (i == j) ? 1.0 : 0.0;
         }
      }
   }

   private static void Midentity(DoubleYoVariable[][] a)
   {
      for (int i = 0; i < a.length; i++)
      {
         for (int j = 0; j < a[0].length; j++)
         {
            a[i][j].set((i == j) ? 1.0 : 0.0);
         }
      }
   }


   private static double Mtrace(DoubleYoVariable[][] a)
   {
      double trace = 0.0;
      for (int i = 0; i < a.length; i++)
      {
         for (int j = 0; j < a[0].length; j++)
         {
            trace += (i == j) ? a[i][j].getDoubleValue() : 0.0;
         }
      }

      return trace;
   }

   private static void zero(double[][] a)
   {
      for (int i = 0; i < a.length; i++)
      {
         for (int j = 0; j < a[0].length; j++)
         {
            a[i][j] = 0.0;
         }
      }
   }

   @SuppressWarnings("unused")
   private static double mag(double[][] a)
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

   private static double mag(double[] a)
   {
      double ret = 0.0;
      for (int i = 0; i < a.length; i++)
      {
         ret += a[i] * a[i];
      }

      return Math.sqrt(ret);
   }


// 4x4 Matrix Inversion
// http://www.cvl.iis.u-tokyo.ac.jp/~miyazaki/tech/teche23.html
   private static void Minverse(double[][] a, double[][] b)
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
         System.err.println("Minverse: determinant is zero.");

         return;
      }

      det = 1.0 / det;
      b[0][0] = det
                * (a[1][1] * a[2][2] * a[3][3] + a[1][2] * a[2][3] * a[3][1] + a[1][3] * a[2][1] * a[3][2] - a[1][1] * a[2][3] * a[3][2]
                   - a[1][2] * a[2][1] * a[3][3] - a[1][3] * a[2][2] * a[3][1]);
      b[0][1] = det
                * (a[0][1] * a[2][3] * a[3][2] + a[0][2] * a[2][1] * a[3][3] + a[0][3] * a[2][2] * a[3][1] - a[0][1] * a[2][2] * a[3][3]
                   - a[0][2] * a[2][3] * a[3][1] - a[0][3] * a[2][1] * a[3][2]);
      b[0][2] = det
                * (a[0][1] * a[1][2] * a[3][3] + a[0][2] * a[1][3] * a[3][1] + a[0][3] * a[1][1] * a[3][2] - a[0][1] * a[1][3] * a[3][2]
                   - a[0][2] * a[1][1] * a[3][3] - a[0][3] * a[1][2] * a[3][1]);
      b[0][3] = det
                * (a[0][1] * a[1][3] * a[2][2] + a[0][2] * a[1][1] * a[2][3] + a[0][3] * a[1][2] * a[2][1] - a[0][1] * a[1][2] * a[2][3]
                   - a[0][2] * a[1][3] * a[2][1] - a[0][3] * a[1][1] * a[2][2]);
      b[1][0] = det
                * (a[1][0] * a[2][3] * a[3][2] + a[1][2] * a[2][0] * a[3][3] + a[1][3] * a[2][2] * a[3][0] - a[1][0] * a[2][2] * a[3][3]
                   - a[1][2] * a[2][3] * a[3][0] - a[1][3] * a[2][0] * a[3][2]);
      b[1][1] = det
                * (a[0][0] * a[2][2] * a[3][3] + a[0][2] * a[2][3] * a[3][0] + a[0][3] * a[2][0] * a[3][2] - a[0][0] * a[2][3] * a[3][2]
                   - a[0][2] * a[2][0] * a[3][3] - a[0][3] * a[2][2] * a[3][0]);
      b[1][2] = det
                * (a[0][0] * a[1][3] * a[3][2] + a[0][2] * a[1][0] * a[3][3] + a[0][3] * a[1][2] * a[3][0] - a[0][0] * a[1][2] * a[3][3]
                   - a[0][2] * a[1][3] * a[3][0] - a[0][3] * a[1][0] * a[3][2]);
      b[1][3] = det
                * (a[0][0] * a[1][2] * a[2][3] + a[0][2] * a[1][3] * a[2][0] + a[0][3] * a[1][0] * a[2][2] - a[0][0] * a[1][3] * a[2][2]
                   - a[0][2] * a[1][0] * a[2][3] - a[0][3] * a[1][2] * a[2][0]);
      b[2][0] = det
                * (a[1][0] * a[2][1] * a[3][3] + a[1][1] * a[2][3] * a[3][0] + a[1][3] * a[2][0] * a[3][1] - a[1][0] * a[2][3] * a[3][1]
                   - a[1][1] * a[2][0] * a[3][3] - a[1][3] * a[2][1] * a[3][0]);
      b[2][1] = det
                * (a[0][0] * a[2][3] * a[3][1] + a[0][1] * a[2][0] * a[3][3] + a[0][3] * a[2][1] * a[3][0] - a[0][0] * a[2][1] * a[3][3]
                   - a[0][1] * a[2][3] * a[3][0] - a[0][3] * a[2][0] * a[3][1]);
      b[2][2] = det
                * (a[0][0] * a[1][1] * a[3][3] + a[0][1] * a[1][3] * a[3][0] + a[0][3] * a[1][0] * a[3][1] - a[0][0] * a[1][3] * a[3][1]
                   - a[0][1] * a[1][0] * a[3][3] - a[0][3] * a[1][1] * a[3][0]);
      b[2][3] = det
                * (a[0][0] * a[1][3] * a[2][1] + a[0][1] * a[1][0] * a[2][3] + a[0][3] * a[1][1] * a[2][0] - a[0][0] * a[1][1] * a[2][3]
                   - a[0][1] * a[1][3] * a[2][0] - a[0][3] * a[1][0] * a[2][1]);
      b[3][0] = det
                * (a[1][0] * a[2][2] * a[3][1] + a[1][1] * a[2][0] * a[3][2] + a[1][2] * a[2][1] * a[3][0] - a[1][0] * a[2][1] * a[3][2]
                   - a[1][1] * a[2][2] * a[3][0] - a[1][2] * a[2][0] * a[3][1]);
      b[3][1] = det
                * (a[0][0] * a[2][1] * a[3][2] + a[0][1] * a[2][2] * a[3][0] + a[0][2] * a[2][0] * a[3][1] - a[0][0] * a[2][2] * a[3][1]
                   - a[0][1] * a[2][0] * a[3][2] - a[0][2] * a[2][1] * a[3][0]);
      b[3][2] = det
                * (a[0][0] * a[1][2] * a[3][1] + a[0][1] * a[1][0] * a[3][2] + a[0][2] * a[1][1] * a[3][0] - a[0][0] * a[1][1] * a[3][2]
                   - a[0][1] * a[1][2] * a[3][0] - a[0][2] * a[1][0] * a[3][1]);
      b[3][3] = det
                * (a[0][0] * a[1][1] * a[2][2] + a[0][1] * a[1][2] * a[2][0] + a[0][2] * a[1][0] * a[2][1] - a[0][0] * a[1][2] * a[2][1]
                   - a[0][1] * a[1][0] * a[2][2] - a[0][2] * a[1][1] * a[2][0]);
   }

   // Private State Changing internal methods:

   /*
    * This will construct the quaternion omega matrix
    * W(4,4)
    * p, q, r (rad/sec)
    */
   private void quatW(double[] w_xyz)
   {
      double p = w_xyz[0] / 2.0;
      double q = w_xyz[1] / 2.0;
      double r = w_xyz[2] / 2.0;

//    double[][] m =
//        {
//        {0, -p, -q, -r},
//        {p, 0, r, -q},
//        {q, -r, 0, p},
//        {r, q, -p, 0}
//    };
//    setArray(Wxq, m);

      Wxq[0][0] = 0.0;
      Wxq[0][1] = -p;
      Wxq[0][2] = -q;
      Wxq[0][3] = -r;
      Wxq[1][0] = p;
      Wxq[1][1] = 0.0;
      Wxq[1][2] = r;
      Wxq[1][3] = -q;
      Wxq[2][0] = q;
      Wxq[2][1] = -r;
      Wxq[2][2] = 0.0;
      Wxq[2][3] = p;
      Wxq[3][0] = r;
      Wxq[3][1] = q;
      Wxq[3][2] = -p;
      Wxq[3][3] = 0;
   }


   private void makeAMatrix(double[] pqr)
   {
      quatW(pqr);

      /*
       * A[0..4][0..4] is the partials of d(Qdot) / d(Q),
       * which is the Body rates euler cross.
       * A[0..3][4..6] is the partials of d(Qdot) / d(Gyro_bias)
       * Qdot = quatW( pqr - gyro_bias) * Q
       * A[4..6][0..3] is the partials of d(Gyro_bias_dot)/d(Q)
       * which is zero.
       * A[4..6][4..6] is the partials of d(Gyro_bias_dot)/d(Gyro_bias)
       * which is also zero.
       */

      double q0 = Quat[0].getDoubleValue();
      double q1 = Quat[1].getDoubleValue();
      double q2 = Quat[2].getDoubleValue();
      double q3 = Quat[3].getDoubleValue();

//    double[][] m =
//        {
//        {Wxq[0][0], Wxq[0][1], Wxq[0][2], Wxq[0][3], q1 / 2.0, q2 / 2.0, q3 / 2.0},
//        {Wxq[1][0], Wxq[1][1], Wxq[1][2], Wxq[1][3], -q0 / 2.0, q3 / 2.0, -q2 / 2.0},
//        {Wxq[2][0], Wxq[2][1], Wxq[2][2], Wxq[2][3], -q3 / 2.0, -q0 / 2.0, q1 / 2.0},
//        {Wxq[3][0], Wxq[3][1], Wxq[3][2], Wxq[3][3], q2 / 2.0, -q1 / 2.0, -q0 / 2.0},
//        {0, 0, 0, 0, 0, 0, 0},
//        {0, 0, 0, 0, 0, 0, 0},
//        {0, 0, 0, 0, 0, 0, 0}
//    };

      A[0][0] = Wxq[0][0];
      A[0][1] = Wxq[0][1];
      A[0][2] = Wxq[0][2];
      A[0][3] = Wxq[0][3];
      A[0][4] = q1 / 2.0;
      A[0][5] = q2 / 2.0;
      A[0][6] = q3 / 2.0;
      A[1][0] = Wxq[1][0];
      A[1][1] = Wxq[1][1];
      A[1][2] = Wxq[1][2];
      A[1][3] = Wxq[1][3];
      A[1][4] = -q0 / 2.0;
      A[1][5] = q3 / 2.0;
      A[1][6] = -q2 / 2.0;
      A[2][0] = Wxq[2][0];
      A[2][1] = Wxq[2][1];
      A[2][2] = Wxq[2][2];
      A[2][3] = Wxq[2][3];
      A[2][4] = -q3 / 2.0;
      A[2][5] = -q0 / 2.0;
      A[2][6] = q1 / 2.0;
      A[3][0] = Wxq[3][0];
      A[3][1] = Wxq[3][1];
      A[3][2] = Wxq[3][2];
      A[3][3] = Wxq[3][3];
      A[3][4] = q2 / 2.0;
      A[3][5] = -q1 / 2.0;
      A[3][6] = -q0 / 2.0;
      A[4][0] = 0.0;
      A[4][1] = 0.0;
      A[4][2] = 0.0;
      A[4][3] = 0.0;
      A[4][4] = 0.0;
      A[4][5] = 0.0;
      A[4][6] = 0.0;
      A[5][0] = 0.0;
      A[5][1] = 0.0;
      A[5][2] = 0.0;
      A[5][3] = 0.0;
      A[5][4] = 0.0;
      A[5][5] = 0.0;
      A[5][6] = 0.0;
      A[6][0] = 0.0;
      A[6][1] = 0.0;
      A[6][2] = 0.0;
      A[6][3] = 0.0;
      A[6][4] = 0.0;
      A[6][5] = 0.0;
      A[6][6] = 0.0;


//    setArray(A, tempM);
   }

   private void setYoVariables(double[][] A, DoubleYoVariable[][] ACopy)
   {
      for (int i = 0; i < A.length; i++)
      {
         for (int j = 0; j < A[i].length; j++)
         {
            ACopy[i][j].set(A[i][j]);
         }
      }
   }

   @SuppressWarnings("unused")
   private void setConstantKMatrix(double[][] K)
   {
      for (int i = 0; i < K.length; i++)
      {
         for (int j = 0; j < K[i].length; j++)
         {
            K[i][j] = 0.0;
         }
      }

      K[0][0] = k_qs.getDoubleValue();
      K[1][1] = k_qxyz.getDoubleValue();
      K[2][2] = k_qxyz.getDoubleValue();
      K[3][3] = k_qxyz.getDoubleValue();

   }

   private void Kalman(DoubleYoVariable[][] P, double[] X)
   {
      // E = C*P*Ct+R
      Mtranspose(C, Ct);
      Mmul(C, P, t1);
      Mmul(t1, Ct, t2);
      Madd(t2, R, E);

      // K = P*Ct*inv(E)
      Mmul(P, Ct, t3);
      Minverse(E, inverseE);
      Mmul(t3, inverseE, K);


//    setConstantKMatrix(K); // Use this if you want a constant K Matrix...
      setYoVariables(K, KCopy);

      // X += K*err;
      Mmul(K, quatError, t4);
      Madd(X, t4, X);

      // P -= K*C*P;
      Mmul(K, C, t5);
      Mmul(t5, P, t6);
      Msub(P, t6, P);
   }

   private void doKalman()
   {
      /*
       * Compute our C matrix, which relates the quaternion state
       * estimate to the quaternions measured by the accelerometers and
       * the compass.  The other states are all zero.
       */

      Midentity(C);

//    double err;
//    double[][] DCM = new double[3][3];
//    quatDC(DCM);
//    double DCM_0_2 = DCM[0][2];
//    double DCM_2_2 = DCM[2][2];
//    double DCM_1_2 = DCM[1][2];
//
//    // PHI section
//            err = 2.0 / ( DCM_2_2 * DCM_2_2 + DCM_1_2 * DCM_1_2 );
//            C[0][0] = err * ( q1 * DCM_2_2 );
//            C[0][1] = err * ( q0 * DCM_2_2 + 2.0 * q1 * DCM_1_2 );
//            C[0][2] = err * ( q3 * DCM_2_2 + 2.0 * q2 * DCM_1_2 );
//            C[0][3] = err * ( q2 * DCM_2_2 );
//
//            // THETA section
//            err = -1.0 / Math.sqrt(1.0 - DCM_0_2 * DCM_0_2 );
//
//            C[1][0] = -2.0 * q2 * err;
//            C[1][1] =  2.0 * q3 * err;
//            C[1][2] = -2.0 * q0 * err;
//            C[1][3] =  2.0 * q1 * err;



      X[0] = Quat[0].getDoubleValue();
      X[1] = Quat[1].getDoubleValue();
      X[2] = Quat[2].getDoubleValue();
      X[3] = Quat[3].getDoubleValue();
      X[4] = bias[0].getDoubleValue();
      X[5] = bias[1].getDoubleValue();
      X[6] = bias[2].getDoubleValue();

      Kalman(P, X);

      Quat[0].set(X[0]);
      Quat[1].set(X[1]);
      Quat[2].set(X[2]);
      Quat[3].set(X[3]);

      bias[0].set(X[4]);
      bias[1].set(X[5]);
      bias[2].set(X[6]);
      normalize(Quat);
   }

   private final double[] tempYawPitchRollAngles = new double[3];

   /**
    *  Convert accelerations to euler angles
    */
   public void accel2quaternions(double[] a, double heading, double[] quaternions)
   {
      // Accel to euler, then euler to quaternions:
      double g = mag(a);

//    double[] euler =
//        { -Math.atan2(a[1][0], -a[2][0]), -Math.asin(a[0][0] / -g), heading}; // Roll, Pitch, Yaw

      tempYawPitchRollAngles[0] = heading;    // yaw
      tempYawPitchRollAngles[1] = -Math.asin(a[0] / -g);    // pitch
      tempYawPitchRollAngles[2] = -Math.atan2(a[1], -a[2]);    // roll
      
      Quaternion quaternion = new Quaternion();
      quaternion.setYawPitchRoll(tempYawPitchRollAngles);
      quaternions[0] = quaternion.getS();
      quaternions[1] = quaternion.getX();
      quaternions[2] = quaternion.getY();
      quaternions[3] = quaternion.getZ();

      // return the closest one:
      double distanceSquared = 0.0;
      double distanceSquaredToNegative = 0.0;

      for (int i = 0; i < 4; i++)
      {
         distanceSquared += (quaternions[i] - Quat[i].getDoubleValue()) * (quaternions[i] - Quat[i].getDoubleValue());
         distanceSquaredToNegative += (-quaternions[i] - Quat[i].getDoubleValue()) * (-quaternions[i] - Quat[i].getDoubleValue());
      }

      if (distanceSquaredToNegative < distanceSquared)
      {
         quaternions[0] *= -1.0;
         quaternions[1] *= -1.0;
         quaternions[2] *= -1.0;
         quaternions[3] *= -1.0;
      }
   }

// /*
//  * This will convert from quaternions to euler angles
//  * q(4,1) -> euler[phi;theta;psi] (rad)
//  */
// public double[][] quat2euler(double[][] quat)
// {
//    double q0 = quat[0][0], q1 = quat[1][0], q2 = quat[2][0], q3 = quat[3][0];
//    double theta = -Math.asin(2 * (q1 * q3 - q0 * q2));
//    double phi = Math.atan2(2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1 * q1 + q2 * q2));
//    double psi = Math.atan2(2 * (q1 * q2 + q0 * q3), 1 - 2 * (q2 * q2 + q3 * q3));
//
//    double[][] m =
//        {
//        {phi},
//        {theta},
//        {psi}
//    };
//    return m;
// }
//
// public double[] euler2quat(double[] euler) {
//    double phi = euler[0] / 2.0; // Roll
//    double theta = euler[1] / 2.0; // Pitch
//    double psi = euler[2] / 2.0; // Yaw
//
//    double shphi0 = Math.sin(phi);
//    double chphi0 = Math.cos(phi);
//
//    double shtheta0 = Math.sin(theta);
//    double chtheta0 = Math.cos(theta);
//
//    double shpsi0 = Math.sin(psi);
//    double chpsi0 = Math.cos(psi);
//
//    double[] quaternions =
//        {chphi0 * chtheta0 * chpsi0 + shphi0 * shtheta0 * shpsi0, -chphi0 * shtheta0 * shpsi0 + shphi0 * chtheta0 * chpsi0,
//        chphi0 * shtheta0 * chpsi0 + shphi0 * chtheta0 * shpsi0, chphi0 * chtheta0 * shpsi0 - shphi0 * shtheta0 * chpsi0};
//
//    return quaternions;
// }

   /*
    * This will construct a direction cosine matrix from
    * quaternions in the standard rotation  sequence
    * [phi][theta][psi] from NED to body frame
    *
    * body = tBL(3,3)*NED
    * q(4,1)
    */
   @SuppressWarnings("unused")
   private void quatDC(double[][] DCM)
   {
      double q0 = Quat[0].getDoubleValue();
      double q1 = Quat[1].getDoubleValue();
      double q2 = Quat[2].getDoubleValue();
      double q3 = Quat[3].getDoubleValue();

//    double[][] m =
//        {
//        {1.0 - 2 * (q2 * q2 + q3 * q3), 2 * (q1 * q2 + q0 * q3), 2 * (q1 * q3 - q0 * q2)},
//        {2 * (q1 * q2 - q0 * q3), 1.0 - 2 * (q1 * q1 + q3 * q3), 2 * (q2 * q3 + q0 * q1)},
//        {2 * (q1 * q3 + q0 * q2), 2 * (q2 * q3 - q0 * q1), 1.0 - 2 * (q1 * q1 + q2 * q2)}
//    };
//    MatrixTools.setArray(DCM,m);

      DCM[0][0] = 1.0 - 2.0 * (q2 * q2 + q3 * q3);
      DCM[0][1] = 2.0 * (q1 * q2 + q0 * q3);
      DCM[0][2] = 2.0 * (q1 * q3 - q0 * q2);
      DCM[1][0] = 2.0 * (q1 * q2 - q0 * q3);
      DCM[1][1] = 1.0 - 2.0 * (q1 * q1 + q3 * q3);
      DCM[1][2] = 2.0 * (q2 * q3 + q0 * q1);
      DCM[2][0] = 2.0 * (q1 * q3 + q0 * q2);
      DCM[2][1] = 2.0 * (q2 * q3 - q0 * q1);
      DCM[2][2] = 1.0 - 2.0 * (q1 * q1 + q2 * q2);
   }

   private final double[] tempQuaternionM = new double[4];

   public void compassUpdate(double heading, double[] accel)
   {
      // Compute our measured and estimated quaternions

//    double[] quaternion_m = new double[4];
      accel2quaternions(accel, heading, tempQuaternionM);

      double q0 = Quat[0].getDoubleValue();
      double q1 = Quat[1].getDoubleValue();
      double q2 = Quat[2].getDoubleValue();
      double q3 = Quat[3].getDoubleValue();

      // Subtract to get the error in quaternions,
      quatError[0] = tempQuaternionM[0] - q0;
      quatError[1] = tempQuaternionM[1] - q1;
      quatError[2] = tempQuaternionM[2] - q2;
      quatError[3] = tempQuaternionM[3] - q3;

      doKalman();
   }

   /*
    * Our state update function for the IMU is:
    *
    * Qdot = Wxq * Q
    * bias_dot = [0,0,0]
    * Q += Qdot * dt
    */
   @SuppressWarnings("unused")
   private void propagateStateOldDoesntUpdateBiases(double[] pqr)
   {
      quatW(pqr);    // constructs quaternion W matrix in Wxq

      // q = q + W*q*dt;
      MmulScalarMul(Wxq, Quat, t9, dt);
      Madd(Quat, t9, Quat);
      normalize(Quat);

      // Keep copy up-to-date...
//    unpackQuaternion(Quat);
   }


   private void propagateState(double[] pqr)
   {
      // quatW(pqr); //                           constructs quaternion W matrix in Wxq
      // q = q + W*q*dt;

      // X_dot = A*X;

      MmulScalarMul(Wxq, Quat, t9, dt);
      Madd(Quat, t9, Quat);
      normalize(Quat);

      // Keep copy up-to-date...
//    unpackQuaternion(Quat);
   }


   private void propagateCovariance(double[][] a)
   {
      // Implements Pdot = A*P*At + Q...

      // New Code:
      Mmul(a, P, t7);    // t7 = A*P;
      Mtranspose(a, At);    // At = A-transpose;
      Mmul(t7, At, t8);    // t8 = A*P*At;
      Madd(t8, Q, Pdot);    // Pdot = A*P*At + Q;

      // P = P + Pdot * dt;
      Mmul(Pdot, dt, Pdot);
      Madd(P, Pdot, P);
      @SuppressWarnings("unused")
      double trace = Mtrace(P);


      // Old Code. Tim Hutchison found bug on Jan. 26, 2009:
      // Pdot = A*P+Q;

      /*
       * if (!USE_FIX)
       * {
       *  setArray(Pdot, Q);
       *
       *  Mmul(a, P, t7);
       *  Madd(Pdot, t7, Pdot);
       *  // Pdot = P*At + A*P+Q;
       *  Mtranspose(a, At);
       *  Mmul(P, At, t8);
       *  Madd(Pdot, t8, Pdot);
       *  // Pdot = (P*At + A*P+Q)*dt;
       *  Mmul(Pdot, dt, Pdot);
       *  // P = P + Pdot;
       *  Madd(P, Pdot, P);
       *  double trace = Mtrace(P);
       * }
       */

   }


   // Public User Methods:

   public void setNoiseParameters(double q_noise, double r_noise)
   {
      zero(Q);
      zero(R);

//    Q[0][0] = q_noise * q_noise;
//    Q[1][1] = q_noise * q_noise;
//    Q[2][2] = q_noise * q_noise;
//    Q[3][3] = q_noise * q_noise;


      Q[4][4] = q_noise * q_noise;
      Q[5][5] = q_noise * q_noise;
      Q[6][6] = q_noise * q_noise;

      R[0][0] = r_noise * r_noise;
      R[1][1] = r_noise * r_noise;
      R[2][2] = r_noise * r_noise;
      R[3][3] = r_noise * r_noise;
   }


   /**
    * Updates the IMU given the rate gyro inputs.
    *
    * @param pqr Matrix Gyro Rate values in order of qd_wy, qd_wx, qd_wz???
    */
   public void imuUpdate(double[] PQR)
   {
      Msub(PQR, bias, PQR);
      makeAMatrix(PQR);
      propagateState(PQR);
      propagateCovariance(A);
   }

   /*
    * We assume that the vehicle is still during the first sample
    * and use the values to help us determine the zero point for the
    * gyro bias and accelerometers.
    *
    * You must call this once you have the samples from the IMU
    * and compass.  Perhaps throw away the first few to let things
    * stabilize.
    */
   public void initialize(double[] Accel, double[] PQR, double heading)
   {
      setArray(bias, PQR);
      double[] quaternions = new double[4];
      accel2quaternions(Accel, heading, quaternions);

      Quat[0].set(quaternions[0]);
      Quat[1].set(quaternions[1]);
      Quat[2].set(quaternions[2]);
      Quat[3].set(quaternions[3]);

//    System.out.println("Debug: Quat0 = " + Quat[0][0]);

//    unpackQuaternion(Quat);
   }

   private void reset(DoubleYoVariable[][] P)
   {
      /*
       * The covariance matrix is probably initialized incorrectly.
       * It should be 1 for all diagonal elements of Q that are 0
       * and zero everywhere else.
       */
      double q_noise = 5.0;    // 5.0; //0.05; //1.0; //10.0; //250.0;
      double r_noise = 1.0;    // 100.0; //10.0; //25.0; //2.5; //25.0; //100.0; //10.0;

      Midentity(P);

      setNoiseParameters(q_noise, r_noise);
   }

// public void reset(double[][] P)
// {
//    /*
//     * The covariance matrix is probably initialized incorrectly.
//     * It should be 1 for all diagonal elements of Q that are 0
//     * and zero everywhere else.
//     */
//    double q_noise = 5.0; //5.0; //0.05; //1.0; //10.0; //250.0;
//    double r_noise = 1.0; //100.0; //10.0; //25.0; //2.5; //25.0; //100.0; //10.0;
//
//    Midentity(P);
//
//    setNoiseParameters(q_noise, r_noise);
// }

   public void initialize(DenseMatrix64F accel, DenseMatrix64F pqr, double heading)
   {
//    System.out.println("Initializing QuaternionBasedArrayFullIMUKalmanFilter. accel = " + QuaternionTools.format4(accel.get(0,0)) + ", " + QuaternionTools.format4(accel.get(1,0)) + ", " + QuaternionTools.format4(accel.get(2,0)));

//     accel.set(0, 0, -x_accel.val);
//     accel.set(1, 0, -y_accel.val);
//     accel.set(2, 0, -z_accel.val);
//
//     pqr.set(0, 0, 0.0); //x_gyro.val); // + x_gyro_bias.val);
//     pqr.set(1, 0, 0.0); //y_gyro.val); // + y_gyro_bias.val);
//     pqr.set(2, 0, 0.0); //z_gyro.val); // + z_gyro_bias.val);

      double[] accelArray = new double[3];
      double[] pqrArray = new double[3];
      accelArray[0] = accel.get(0, 0);
      accelArray[1] = accel.get(1, 0);
      accelArray[2] = accel.get(2, 0);

      pqrArray[0] = pqr.get(0, 0);
      pqrArray[1] = pqr.get(1, 0);
      pqrArray[2] = pqr.get(2, 0);

      initialize(accelArray, pqrArray, heading);

//    setArray(bias, pqrArray);
//      double[] quaternions = accel2quaternions(accelArray, heading);
//      Quat[0][0] = quaternions[0];
//      Quat[1][0] = quaternions[1];
//      Quat[2][0] = quaternions[2];
//      Quat[3][0] = quaternions[3];

      @SuppressWarnings("unused")
      double q0 = Quat[0].getDoubleValue();
      @SuppressWarnings("unused")
      double q1 = Quat[1].getDoubleValue();
      @SuppressWarnings("unused")
      double q2 = Quat[2].getDoubleValue();
      @SuppressWarnings("unused")
      double q3 = Quat[3].getDoubleValue();

//    System.out.println("Initializing QuaternionBasedArrayFullIMUKalmanFilter. q0 = " + QuaternionTools.format4(q0) + ", q1 = " + QuaternionTools.format4(q1) + ", q2 = " + QuaternionTools.format4(q2) + ", q3 = " + QuaternionTools.format4(q3));

   }

   public void reset()
   {
      reset(P);
   }

   public void accel2quaternions(DenseMatrix64F accel, double d, double[] quaternions)
   {
//    accel2quaternions(accel.getArray()[0], d, quaternions);
      accel2quaternions(accel.getData(), d, quaternions);
   }

   public void imuUpdate(DenseMatrix64F pqr)
   {
//    imuUpdate(pqr.getArray()[0]);
      imuUpdate(pqr.getData());
   }

   private double[] accelArrayTemp = new double[3];

   public void compassUpdate(double heading, DenseMatrix64F accel)
   {
      accelArrayTemp[0] = accel.get(0, 0);
      accelArrayTemp[1] = accel.get(1, 0);
      accelArrayTemp[2] = accel.get(2, 0);

      compassUpdate(heading, accelArrayTemp);

//    throw new RuntimeException("implement");
   }

   public double getBias(int i)
   {
      return bias[i].getDoubleValue();
   }

   public void getQuaternion(DenseMatrix64F qMatrix)
   {
//    Matrix qMatrix = new Matrix(4,1);

      double q0 = Quat[0].getDoubleValue();
      double q1 = Quat[1].getDoubleValue();
      double q2 = Quat[2].getDoubleValue();
      double q3 = Quat[3].getDoubleValue();

      qMatrix.set(0, 0, q0);
      qMatrix.set(1, 0, q1);
      qMatrix.set(2, 0, q2);
      qMatrix.set(3, 0, q3);

//    return qMatrix;
   }

}
