package us.ihmc.kalman.imu;

import us.ihmc.utilities.math.geometry.QuaternionTools;
import Jama.Matrix;

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
public class FullIMUKalmanFilter
{
   @SuppressWarnings("unused")
   private static final boolean verbose = true;
   private static final int N = 7;


   /*
    * Covariance matrix and covariance matrix derivative are updated
    * every other state step.  This is because the covariance should change
    * at a rate somewhat slower than the dynamics of the system.
    */
   private Matrix P = new Matrix(N, N);    // Covariance matrix

   /*
    * A represents the Jacobian of the derivative of the system with respect
    * its states.  We do not allocate the bottom three rows since we know that
    * the derivatives of bias_dot are all zero.
    */
   private Matrix A = new Matrix(N, N);

   /*
    * Q is our estimate noise variance.  It is supposed to be an NxN
    * matrix, but with elements only on the diagonals.  Additionally,
    * since the quaternion has no expected noise (we can't directly measure
    * it), those are zero.  For the gyro, we expect around 5 deg/sec noise,
    * which is 0.08 rad/sec.  The variance is then 0.08^2 ~= 0.0075.
    */
   private Matrix Q = new Matrix(N, N);    // Noise estimate

   /*
    * R is our measurement noise estimate.  Like Q, it is supposed to be
    * an NxN matrix with elements on the diagonals.  However, since we can
    * not directly measure the gyro bias, we have no estimate for it.
    * We only have an expected noise in the pitch and roll accelerometers
    * and in the compass.
    */
   private Matrix R = new Matrix(3, 3);    // State estimate for angles

   private Matrix DCM = new Matrix(3, 3);
   private Matrix Wxq = new Matrix(4, 4);

// public Matrix accel = new Matrix(3, 1); // Acceleration inputs. User must remove offsets first.
   public Matrix eulerAngles = new Matrix(3, 1);    // Estimated joint angles.
   public Matrix eulerError = new Matrix(3, 1);

// public Matrix pqr = new Matrix(3, 1);   // Rate gyro rates.
   public Matrix bias = new Matrix(3, 1);    // Rate gyro bias offset estimates. The Kalman filter adapts to these.
   public Matrix q = new Matrix(4, 1);    // Estimated orientation in quaternions.
   public double q0, q1, q2, q3;    // Redundant estimated orientation in quaternions.

   /*
    * The Direction Cosine Matrix is used to help rotate measurements
    * to and from the body frame.  We only need five elements from it,
    * so those are computed explicitly rather than the entire matrix
    */
   private double dcm00, dcm01, dcm02, dcm12, dcm22;

   private double dt = .001;
   @SuppressWarnings("unused")
   private double trace;
   private static final double PI = Math.PI;

   /*
    * C represents the Jacobian of the measurements of the attitude
    * with respect to the states of the filter.
    */
   private Matrix C = new Matrix(3, N);
   private Matrix Ct, E;
   @SuppressWarnings("unused")
   private static final java.text.DecimalFormat fmt = new java.text.DecimalFormat();

   public FullIMUKalmanFilter(double dt)
   {
      this.dt = dt;
      reset();
   }

   /** Format double with Fw.d. */

   public static String fixedWidthDoubletoString(double x, int w, int d)
   {
      java.text.DecimalFormat fmt = new java.text.DecimalFormat();
      fmt.setMaximumFractionDigits(d);
      fmt.setMinimumFractionDigits(d);
      fmt.setGroupingUsed(false);
      String s = fmt.format(x);
      while (s.length() < w)
      {
         s = " " + s;
      }

      return s;
   }

   /** Format integer with Iw. */

   public static String fixedWidthIntegertoString(int n, int w)
   {
      String s = Integer.toString(n);
      while (s.length() < w)
      {
         s = " " + s;
      }

      return s;
   }



   /*
    * This will construct a direction cosine matrix from
    * quaternions in the standard rotation  sequence
    * [phi][theta][psi] from NED to body frame
    *
    * body = tBL(3,3)*NED
    * q(4,1)
    */
   void quatDC(Matrix DCM)
   {
      double[][] m =
      {
         {1.0 - 2 * (q2 * q2 + q3 * q3), 2 * (q1 * q2 + q0 * q3), 2 * (q1 * q3 - q0 * q2)},
         {2 * (q1 * q2 - q0 * q3), 1.0 - 2 * (q1 * q1 + q3 * q3), 2 * (q2 * q3 + q0 * q1)},
         {2 * (q1 * q3 + q0 * q2), 2 * (q2 * q3 - q0 * q1), 1.0 - 2 * (q1 * q1 + q2 * q2)}
      };
      setArray(DCM, m);
   }

   /*
    * This will construct the quaternion omega matrix
    * W(4,4)
    * p, q, r (rad/sec)
    */
   void quatW(Matrix w_xyz)
   {
      double p = w_xyz.get(0, 0) / 2.0;
      double q = w_xyz.get(1, 0) / 2.0;
      double r = w_xyz.get(2, 0) / 2.0;

      double[][] m =
      {
         {0, -p, -q, -r}, {p, 0, r, -q}, {q, -r, 0, p}, {r, q, -p, 0}
      };
      setArray(Wxq, m);
   }


   /*
    * Functions to compute the partial derivative of the quaternion with
    * respect to the Euler angles.  These are used for computation of the
    * matrix C in the Kalman filter that represents the relationship of
    * the measurements to the states.
    */
   Matrix dphi_dq()
   {
      double err = 2 / (dcm22 * dcm22 + dcm12 * dcm12);

      double[][] m =
      {
         {q1 * dcm22}, {q0 * dcm22 + 2 * q1 * dcm12}, {q3 * dcm22 + 2 * q2 * dcm12}, {q2 * dcm22}
      };

      return new Matrix(m).times(err);    // Fix this to work in place without allocating...
   }

   Matrix dtheta_dq()
   {
      double err = -2 / Math.sqrt(1 - dcm02 * dcm02);
      double[][] m =
      {
         {-q2}, {q3}, {-q0}, {q1}
      };

      return new Matrix(m).times(err);    // Fix this to work in place without allocating...
   }

   Matrix dpsi_dq()
   {
      double err = 2 / (dcm00 * dcm00 + dcm01 * dcm01);

      double[][] m =
      {
         {q3 * dcm00}, {q2 * dcm00}, {q1 * dcm00 + 2 * q2 * dcm01}, {q0 * dcm00 + 2 * q3 * dcm01}
      };

      return new Matrix(m).times(err);    // Fix this to work in place without allocating...
   }

   void setArray(Matrix M, double[][] d)
   {
      int m, n;
      if ((m = M.getRowDimension()) == d.length && (n = M.getColumnDimension()) == d[0].length)
      {
         for (int i = 0; i < m; i++)
         {
            for (int j = 0; j < n; j++)
            {
               M.set(i, j, d[i][j]);
            }
         }
      }
      else
         System.err.println("setArray: incompatible dimensions.");
   }

   void setMatrix(Matrix M, Matrix d)
   {
      int m, n;
      if ((m = M.getRowDimension()) == d.getRowDimension() && (n = M.getColumnDimension()) == d.getColumnDimension())
      {
         for (int i = 0; i < m; i++)
         {
            for (int j = 0; j < n; j++)
            {
               M.set(i, j, d.get(i, j));
            }
         }
      }
      else
         System.err.println("setArray: incompatible dimensions.");
   }



   void makeAMatrix(Matrix pqr)
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
      double[][] wxq = Wxq.getArray();
      double[][] m =
      {
         {
            wxq[0][0], wxq[0][1], wxq[0][2], wxq[0][3], q1 / 2, q2 / 2, q3 / 2
         },
         {
            wxq[1][0], wxq[1][1], wxq[1][2], wxq[1][3], -q0 / 2, q3 / 2, -q2 / 2
         },
         {
            wxq[2][0], wxq[2][1], wxq[2][2], wxq[2][3], -q3 / 2, -q0 / 2, q1 / 2
         },
         {
            wxq[3][0], wxq[3][1], wxq[3][2], wxq[3][3], q2 / 2, -q1 / 2, -q0 / 2
         },
         {
            0, 0, 0, 0, 0, 0, 0
         },
         {
            0, 0, 0, 0, 0, 0, 0
         },
         {
            0, 0, 0, 0, 0, 0, 0
         }
      };

      setArray(A, m);
   }

   public static void normalize(Matrix M)
   {
      double mag = 0;
      double s;
      int m = M.getRowDimension(), n = M.getColumnDimension();
      for (int i = 0; i < m; i++)
      {
         for (int j = 0; j < n; j++)
         {
            s = M.get(i, j);
            mag += s * s;
         }
      }

      mag = Math.sqrt(mag);

      for (int i = 0; i < m; i++)
      {
         for (int j = 0; j < n; j++)
         {
            M.set(i, j, M.get(i, j) / mag);
         }
      }
   }

   void Kalman(Matrix P, Matrix X, Matrix C, Matrix R, Matrix err, Matrix K)
   {
      Ct = C.transpose();
      E = C.times(P).times(Ct).plus(R);    // E = C*P*Ct+R
      K = P.times(Ct).times(E.inverse());    // K = P*Ct*inv(E)
      X.plusEquals(K.times(err));    // X += K*err;
      P.minusEquals(K.times(C).times(P));    // P -= K*C*P;
   }

// void do_kalman(Matrix<3,N> C, Matrix<3,3> R, Matrix<1,3> error, int m=3) {
   static Matrix K = new Matrix(N, 3);

   void doKalman(Matrix C, Matrix R, Matrix error)
   {
      // We throw away the K result

      // Kalman() wants a vector, not an object.  Serialize the
      // state data into this vector, then extract it out again
      // once we're done with the loop.
      double[][] x_vect =
      {
         {q.get(0, 0)}, {q.get(1, 0)}, {q.get(2, 0)}, {q.get(3, 0)}, {bias.get(0, 0)}, {bias.get(1, 0)}, {bias.get(2, 0)}
      };
      Matrix X_vect = new Matrix(x_vect);

      Kalman(P, X_vect, C, R, error, K);

      q.set(0, 0, X_vect.get(0, 0));
      q.set(1, 0, X_vect.get(1, 0));
      q.set(2, 0, X_vect.get(2, 0));
      q.set(3, 0, X_vect.get(3, 0));

      bias.set(0, 0, X_vect.get(4, 0));
      bias.set(1, 0, X_vect.get(5, 0));
      bias.set(2, 0, X_vect.get(6, 0));
      normalize(q);

      QuaternionTools.quaternionsToRollPitchYaw(q, eulerAngles);
   }

   /*
    * Determine the shortest distance between two euler angles.
    * This might involve wrapping the other way around the circle.
    * You must have called compute_euler() to produce the euler
    * attitude estimate before calling this.
    */

// Matrix euler_diff(Matrix euler_m, Matrix euler) {
   void euler_diff(Matrix euler_m, Matrix euler)
   {
      double[][] m =
      {
         {euler_m.get(0, 0) - euler.get(0, 0)}, {euler_m.get(1, 0) - euler.get(1, 0)}, {euler_m.get(2, 0) - euler.get(2, 0)}
      };

      double d = m[0][0];    // Roll angles go from -180 degs to +180 degs
      if (d < -PI)
         m[0][0] = d + 2 * PI;
      else if (d > PI)
         m[0][0] = d - 2 * PI;

//    m[0][0] = (d < -PI) ? (d + 2 * PI) : ((d > PI) ? (d - 2 * PI) : d);

      // +++JEP. This seems very buggy to me. You can't just go around adding +- PI?
      d = m[1][0];    // Pitch angles only go +/- 90 degs

      if (d > PI / 2.0)
      {
         m[1][0] = d - PI;
      }
      else if (d < -PI / 2.0)
      {
         m[1][0] = d + PI;
      }

//    m[1][0] = (d > PI / 2) ? (d - PI) : ((d < -PI / 2) ? (d + PI) : d);

      d = m[2][0];    // Heading is +/- 180 degs

      if (d > PI)
      {
         m[2][0] = d + 2.0 * PI;
      }
      else if (d < -PI)
      {
         m[2][0] = d - 2.0 * PI;
      }

//    m[2][0] = (d > PI) ? (d - 2 * PI) : ((d < -PI) ? (d + 2 * PI) : d);

//    Matrix diff = new Matrix(m);
//    return diff;
      setArray(eulerError, m);
   }

   void zero(Matrix a)
   {
      for (int i = 0; i < a.getRowDimension(); i++)
      {
         for (int j = 0; j < a.getColumnDimension(); j++)
         {
            a.set(i, j, 0.0);
         }
      }
   }

   /**
    *  Convert accelerations to euler angles
    */
   double mag(Matrix a)
   {
      double ret = 0.0;
      for (int i = 0; i < a.getRowDimension(); i++)
      {
         for (int j = 0; j < a.getColumnDimension(); j++)
         {
            ret += a.get(i, j) * a.get(i, j);
         }
      }

      return Math.sqrt(ret);
   }

   public void accel2euler(Matrix a, double heading)
   {
      double g = mag(a);
      double[][] m =
      {
         {-Math.atan2(a.get(1, 0), -a.get(2, 0))},    // Roll
         {-Math.asin(a.get(0, 0) / -g)},    // Pitch
         {heading}    // Yaw
      };
      setArray(eulerAngles, m);

//    return new Matrix(m);
   }

   public void compassUpdate(double heading, Matrix accel)
   {
//    this.accel = accel;

      // Compute our measured and estimated angles
//    Matrix angles_m = accel2euler(accel, heading);
      accel2euler(accel, heading);
      Matrix angles_e = new Matrix(3, 1);
      QuaternionTools.quaternionsToRollPitchYaw(q, angles_e);

      // Compute the error between our measurement and our estimate
//    Matrix error = euler_diff(angles_m, angles_e);
      euler_diff(eulerAngles, angles_e);

      /*
       * Compute our C matrix, which relates the quaternion state
       * estimate to the angles measured by the accelerometers and
       * the compass.  The other states are all zero.
       */

      // Compute the DCM of our quaternion estimate
      quatDC(DCM);    // see Quat.h....
      double[][] dcm = DCM.getArray();
      dcm00 = dcm[0][0];
      dcm01 = dcm[0][1];
      dcm02 = dcm[0][2];
      dcm12 = dcm[1][2];
      dcm22 = dcm[2][2];

      Matrix dphi = dphi_dq();
      Matrix dtheta = dtheta_dq();
      Matrix dpsi = dpsi_dq();

      C.set(0, 0, dphi.get(0, 0));
      C.set(0, 1, dphi.get(1, 0));
      C.set(0, 2, dphi.get(2, 0));
      C.set(0, 3, dphi.get(3, 0));

      C.set(1, 0, dtheta.get(0, 0));
      C.set(1, 1, dtheta.get(1, 0));
      C.set(1, 2, dtheta.get(2, 0));
      C.set(1, 3, dtheta.get(3, 0));

      C.set(2, 0, dpsi.get(0, 0));
      C.set(2, 1, dpsi.get(1, 0));
      C.set(2, 2, dpsi.get(2, 0));
      C.set(2, 3, dpsi.get(3, 0));

//    if(verbose)
//       System.out.println("compass_update:" + " m = " + angles_m + " e = " + angles_e + " err = " + error);

//    doKalman(C, R, error);
      doKalman(C, R, eulerError);
   }

   void unpackQuaternion(Matrix quat)
   {
      double[][] q = quat.getArray();
      q0 = q[0][0];
      q1 = q[1][0];
      q2 = q[2][0];
      q3 = q[3][0];

   }

   /*
    * Our state update function for the IMU is:
    *
    * Qdot = Wxq * Q
    * bias_dot = [0,0,0]
    * Q += Qdot * dt
    */
   void propagateState(Matrix pqr)
   {
      quatW(pqr);    // construct the quaternion W matrix in Wxq
      Matrix Qdot = Wxq.times(q);    // Qdot = Wxq * q;
      q.plusEquals(Qdot.times(dt));    // q += Qdot * dt;
      normalize(q);

      // Keep copy up-to-date...
      unpackQuaternion(q);
   }

   void propagateCovariance(Matrix A)
   {
      Matrix Pdot = Q.copy();

//    System.out.print("propagateCovariance: Pdot = ");   Pdot.print(fmt, 10);
      Pdot.plusEquals(A.times(P));    // += A * this->P;

//    System.out.print("propagateCovariance: Pdot = ");   Pdot.print(fmt, 10);
      Pdot.plusEquals(P.times(A.transpose()));    // += this->P * A.transpose();

//    System.out.print("propagateCovariance: Pdot = ");   Pdot.print(fmt, 10);
      Pdot.timesEquals(dt);    // *= this->dt;

//    System.out.print("propagateCovariance: Pdot = ");   Pdot.print(fmt, 10);
      P.plusEquals(Pdot);    // += Pdot;

//    System.out.print("propagateCovariance: P = ");   P.print(fmt, 10);
      trace = P.trace();
   }

   /**
    * Updates the IMU given the rate gyro inputs.
    *
    * @param pqr Matrix Gyro Rate values in order of qd_wy, qd_wx, qd_wz???
    */
   public void imuUpdate(Matrix pqr)
   {
      pqr.minusEquals(bias);
      unpackQuaternion(q);
      makeAMatrix(pqr);
      propagateState(pqr);
      propagateCovariance(A);

      /* compute angles from quaternions */
      QuaternionTools.quaternionsToRollPitchYaw(q, eulerAngles);
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
   public void initialize(Matrix accel, Matrix pqr, double heading)
   {
      setMatrix(bias, pqr);

//    euler = accel2euler(accel, heading);
      accel2euler(accel, heading);
      QuaternionTools.rollPitchYawToQuaternions(eulerAngles, q);
   }

   public void reset()
   {
      /*
       * The covariance matrix is probably initialized incorrectly.
       * It should be 1 for all diagonal elements of Q that are 0
       * and zero everywhere else.
       */
      zero(P);
      zero(Q);
      zero(R);

      P.set(0, 0, 1);    // P = I
      P.set(1, 1, 1);
      P.set(2, 2, 1);
      P.set(3, 3, 1);

      // Quaternion attitude estimate noise
      // Since we have only one way to measure it, we leave it
      // set to zero.

      double q_noise = 0.05;    // 1.0; //10.0; //250.0;
      double r_noise = 25.0;    // 100.0; //10.0;

      Q.set(4, 4, q_noise * q_noise);
      Q.set(5, 5, q_noise * q_noise);
      Q.set(6, 6, q_noise * q_noise);

      R.set(0, 0, r_noise * r_noise);
      R.set(1, 1, r_noise * r_noise);
      R.set(2, 2, r_noise * r_noise);

      // Gyro bias estimate noise
//    Q.set(4, 4, 0.05 * 0.05);
//    Q.set(5, 5, 0.05 * 0.05);
//    Q.set(6, 6, 0.05 * 0.05);

//    Q.set(4, 4, 0.005 * 0.005);
//    Q.set(5, 5, 0.005 * 0.005);
//    Q.set(6, 6, 0.005 * 0.005);

//    Q.set(4, 4, 0.5 * 0.5);
//    Q.set(5, 5, 0.5 * 0.5);
//    Q.set(6, 6, 0.5 * 0.5);

//    Q.set(4, 4, 5.0*5.0);
//    Q.set(5, 5, 5.0*5.0);
//    Q.set(6, 6, 5.0*5.0);

//    Q.set(4, 4, 25.0*25.0);
//    Q.set(5, 5, 25.0*25.0);
//    Q.set(6, 6, 25.0*25.0);

//    Q.set(4, 4, 250.0*250.0);
//    Q.set(5, 5, 250.0*250.0);
//    Q.set(6, 6, 250.0*250.0);




      // Measurement estimate noise.  Our heading is likely
      // to have more noise than the pitch and roll angles.
//    R.set(0, 0, 25.3 * 25.3);
//    R.set(1, 1, 25.3 * 25.3);
//    R.set(2, 2, 28.5 * 28.5);

//    R.set(0, 0, 2.53 * 2.53);
//    R.set(1, 1, 2.53 * 2.53);
//    R.set(2, 2, 2.85 * 2.85);


//    R.set(0, 0, 253 * 253);
//    R.set(1, 1, 253 * 253);
//    R.set(2, 2, 285 * 285);


//    R.set(0, 0, 0.001 * 0.001);
//    R.set(1, 1, 0.001 * 0.001);
//    R.set(2, 2, 0.001 * 0.001);

   }

// public static void main(String[] args) {
//    try
//    {
//       FullIMUKalmanFilter ahrs = new FullIMUKalmanFilter(.001);
//       double x = -0.37727;
//       int t = 0;
//
//       ahrs.reset();
//       ahrs.accel.set(0, 0, x);
//       ahrs.initialize(ahrs.accel, ahrs.pqr, 0.0);
//       while(t++ < 3)
//       {
//          ahrs.accel.set(0, 0, x);
//          ahrs.imuUpdate(ahrs.pqr);
//          if(verbose) {
//             System.out.print(t + " accel = ");  ahrs.accel.print(fmt, 10);
//             System.out.print(t + " P = ");  ahrs.P.print(fmt, 10);
//             System.out.print(t + " Q = ");  ahrs.Q.print(fmt, 10);
//             System.out.print(t + " R = ");  ahrs.R.print(fmt, 10);
//             System.out.print(t + " pqr = ");  ahrs.pqr.print(fmt, 10);
//             System.out.print(t + " bias = ");  ahrs.bias.print(fmt, 10);
//             System.out.print(t + " q = ");  ahrs.q.print(fmt, 10);
//          }
//          ahrs.compassUpdate(0.0, ahrs.accel);
//          if(verbose) {
//             System.out.print(t + " accel = ");  ahrs.accel.print(fmt, 10);
//             System.out.print(t + " P = ");  ahrs.P.print(fmt, 10);
//             System.out.print(t + " Q = ");  ahrs.Q.print(fmt, 10);
//             System.out.print(t + " R = ");  ahrs.R.print(fmt, 10);
//             System.out.print(t + " pqr = ");  ahrs.pqr.print(fmt, 10);
//             System.out.print(t + " bias = ");  ahrs.bias.print(fmt, 10);
//             System.out.print(t + " q = ");  ahrs.q.print(fmt, 10);
//          }
//       }
//    }
//    catch(Throwable e)
//    {
//       e.printStackTrace();
//    }
// }


   /*
    * / Misc unused methods...
    * /    * Compute the shortest way from the current position to the
    * /    * commanded position.  Both are in radians.
    *  double turn_direction(double command, double current) {
    *     if(current > PI / 2 && command < -PI / 2)
    *        return 2 * PI + command - current;
    *     if(current < -PI / 2 && command > PI / 2)
    *        return -2 * PI + command - current;
    *     return command - current;
    *  }
    *
    *  Matrix euler_strapdown(Matrix euler) {
    *     double sphi = Math.sin(euler.get(0, 0));
    *     double cphi = Math.cos(euler.get(0, 0));
    *     double ctheta = Math.cos(euler.get(1, 0));
    *     double ttheta = Math.tan(euler.get(1, 0));
    *
    *     double[][] m =
    *                    {
    *                    {1, sphi * ttheta, cphi * ttheta},
    *                    {0, cphi, -sphi},
    *                    {0, sphi / ctheta, cphi / ctheta}
    *     };
    *     return new Matrix(m);
    *  }
    *
    * /    *  Add random noise to a vector
    *  void noise(Matrix v, double high, double low) {
    *     for(int i = 0; i < v.getRowDimension(); i++)
    *        for(int j = 0; j < v.getColumnDimension(); j++)
    *           v.set(i, j, v.get(i, j) + Math.random() * (high - low) + low);
    *  }
    *
    * /    * This will construct a direction cosine matrix from
    * /    * euler angles in the standard rotation sequence
    * /    * [phi][theta][psi] from NED to body frame
    * /
    * /    * body = tBL(3,3)*NED
    *  Matrix eulerDC(Matrix euler) {
    *     double phi = euler.get(0, 0);
    *     double theta = euler.get(0, 1);
    *     double psi = euler.get(0, 2);
    *
    *     double cpsi = Math.cos(psi);
    *     double cphi = Math.cos(phi);
    *     double ctheta = Math.cos(theta);
    *
    *     double spsi = Math.sin(psi);
    *     double sphi = Math.sin(phi);
    *     double stheta = Math.sin(theta);
    *
    *     double[][] m =
    *         {
    *         {cpsi * ctheta, spsi * ctheta, -stheta},
    *         { -spsi * cphi + cpsi * stheta * sphi, cpsi * cphi + spsi * stheta * sphi, ctheta * sphi},
    *         {spsi * sphi + cpsi * stheta * cphi, -cpsi * sphi + spsi * stheta * cphi, ctheta * cphi}
    *     };
    *     return new Matrix(m);
    *  }
    *
    * /    * This will construct the euler omega-cross matrix
    * /    * wx(3,3)
    * /    * p, q, r (rad/sec)
    *  Matrix eulerWx(Matrix euler) {
    *     double p = euler.get(0, 0);
    *     double q = euler.get(1, 0);
    *     double r = euler.get(2, 0);
    *
    *     double[][] m =
    *         {
    *         {0, -r, q},
    *         {r, 0, -p},
    *         { -q, p, 0}
    *     };
    *     return new Matrix(m);
    *  }
    *
    *  void normalize(double[] v) {
    *     double mag = 0;
    *     for(int i = 0; i < v.length; i++)
    *        mag += v[i] * v[i];
    *     mag = Math.sqrt(mag);
    *     for(int i = 0; i < v.length; i++)
    *        v[i] /= mag;
    *  }
    */

}
