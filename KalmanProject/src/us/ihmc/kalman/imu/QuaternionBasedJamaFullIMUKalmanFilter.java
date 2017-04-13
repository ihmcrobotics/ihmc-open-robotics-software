package us.ihmc.kalman.imu;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.RotationTools;

public class QuaternionBasedJamaFullIMUKalmanFilter implements QuaternionBasedFullIMUKalmanFilter
{
   @SuppressWarnings("unused")
   private static final boolean verbose = true;
   private static final int N = 7;

   public double trace;

   /*
    * Covariance matrix and covariance matrix derivative are updated
    * every other state step.  This is because the covariance should change
    * at a rate somewhat slower than the dynamics of the system.
    */
   private DenseMatrix64F P = new DenseMatrix64F(N, N);    // Covariance matrix

   /*
    * A represents the Jacobian of the derivative of the system with respect
    * its states.  We do not allocate the bottom three rows since we know that
    * the derivatives of bias_dot are all zero.
    */
   private DenseMatrix64F A = new DenseMatrix64F(N, N);

   /*
    * Q is our estimate noise variance.  It is supposed to be an NxN
    * matrix, but with elements only on the diagonals.  Additionally,
    * since the quaternion has no expected noise (we can't directly measure
    * it), those are zero.  For the gyro, we expect around 5 deg/sec noise,
    * which is 0.08 rad/sec.  The variance is then 0.08^2 ~= 0.0075.
    */
   private DenseMatrix64F Q = new DenseMatrix64F(N, N);    // Noise estimate

   /*
    * R is our measurement noise estimate.  Like Q, it is supposed to be
    * an NxN matrix with elements on the diagonals.  However, since we can
    * not directly measure the gyro bias, we have no estimate for it.
    * We only have an expected noise in the pitch and roll accelerometers
    * and in the compass.
    */
   private DenseMatrix64F R = new DenseMatrix64F(4, 4);    // State estimate for angles
   private DenseMatrix64F Wxq = new DenseMatrix64F(4, 4);
   public DenseMatrix64F bias = new DenseMatrix64F(3, 1);    // Rate gyro bias offset estimates. The Kalman filter adapts to these.
   public DenseMatrix64F q = new DenseMatrix64F(4, 1);    // Estimated orientation in quaternions.

   private double dt = .001;
   @SuppressWarnings("unused")
   private static final double PI = Math.PI;

   /*
    * C represents the Jacobian of the measurements of the attitude
    * with respect to the states of the filter.
    */
   private DenseMatrix64F C = new DenseMatrix64F(4, N);
   private DenseMatrix64F Ct, E;
   @SuppressWarnings("unused")
   private static final java.text.DecimalFormat fmt = new java.text.DecimalFormat();

   public QuaternionBasedJamaFullIMUKalmanFilter(double dt)
   {
      this.dt = dt;
      reset();
   }

   /*
    * This will construct the quaternion omega matrix
    * W(4,4)
    * p, q, r (rad/sec)
    */
   void quatW(DenseMatrix64F w_xyz)
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



   void setArray(DenseMatrix64F M, double[][] d)
   {
      int m, n;
      if ((m = M.getNumRows()) == d.length && (n = M.getNumCols()) == d[0].length)
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

   void setMatrix(DenseMatrix64F M, DenseMatrix64F d)
   {
      int m, n;
      if ((m = M.getNumRows()) == d.getNumRows() && (n = M.getNumCols()) == d.getNumCols())
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



   void makeAMatrix(DenseMatrix64F pqr)
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
      double q0 = q.get(0, 0);
      double q1 = q.get(1, 0);
      double q2 = q.get(2, 0);
      double q3 = q.get(3, 0);

      double[][] m =
      {
         {
            Wxq.get(0, 0), Wxq.get(0, 1), Wxq.get(0, 2), Wxq.get(0, 3), q1 / 2, q2 / 2, q3 / 2
         },
         {
            Wxq.get(1, 0), Wxq.get(1, 1), Wxq.get(1, 2), Wxq.get(1, 3), -q0 / 2, q3 / 2, -q2 / 2
         },
         {
            Wxq.get(2, 0), Wxq.get(2, 1), Wxq.get(2, 2), Wxq.get(2, 3), -q3 / 2, -q0 / 2, q1 / 2
         },
         {
            Wxq.get(3, 0), Wxq.get(3, 1), Wxq.get(3, 2), Wxq.get(3, 3), q2 / 2, -q1 / 2, -q0 / 2
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

   public static void normalizeQuaternion(DenseMatrix64F M)
   {
      double mag = 0;
      double s;
      int m = M.getNumRows(), n = M.getNumCols();
      for (int i = 0; i < m; i++)
      {
         for (int j = 0; j < n; j++)
         {
            s = M.get(i, j);
            mag += s * s;
         }
      }

      mag = Math.sqrt(mag);

      // If the quaternion is zero, make it no rotation (1,0,0,0) quaternion...
      if (mag < 1e-5)
      {
         M.set(0, 0, 1.0);
         M.set(1, 0, 0.0);
         M.set(2, 0, 0.0);
         M.set(3, 0, 0.0);
      }

      for (int i = 0; i < m; i++)
      {
         for (int j = 0; j < n; j++)
         {
            M.set(i, j, M.get(i, j) / mag);
         }
      }
   }

// int ticks = 0;

   int nonInvertibleTicks = 0;

   private void Kalman(DenseMatrix64F P, DenseMatrix64F X, DenseMatrix64F C, DenseMatrix64F R, DenseMatrix64F err, DenseMatrix64F K)
   {
      Ct = new DenseMatrix64F(C);
      CommonOps.transpose(Ct);
      E = new DenseMatrix64F(C.getNumRows(), Ct.getNumCols());
      DenseMatrix64F temp = new DenseMatrix64F(C.getNumRows(), P.getNumCols());
   // E = C*P*Ct+R
      CommonOps.mult(C, P, temp);
      CommonOps.mult(temp, Ct, E);
      CommonOps.add(E, R, E);

      double Edet = CommonOps.det(E);
      if (Math.abs(Edet) < 1e-6)
      {
         nonInvertibleTicks++;

         if (nonInvertibleTicks > 100)
         {
            nonInvertibleTicks = 0;
            System.out.println("QuaternionBasedFullIMUKalmanFilter::Kalman: E is not invertible! determinant = " + Edet);
         }

//       throw new RuntimeException("E is not invertible!!!");
      }
      else
      {
         DenseMatrix64F E_Inverse = new DenseMatrix64F(E.getNumCols(), E.getNumRows());
         CommonOps.invert(E, E_Inverse);

         temp = new DenseMatrix64F(P.getNumRows(), Ct.getNumCols());
         K = new DenseMatrix64F(P.getNumRows(), E_Inverse.getNumCols());
         // K = P*Ct*inv(E)
         CommonOps.mult(P, Ct, temp);
         CommonOps.mult(temp, E_Inverse, K);

         DenseMatrix64F KTimesErr = new DenseMatrix64F(K.getNumRows(), err.getNumCols());
         CommonOps.mult(K, err, KTimesErr);
         
         CommonOps.addEquals(X, KTimesErr); // X += K*err;
         
         temp.reshape(K.getNumRows(), C.getNumCols());
         CommonOps.mult(K, C, temp);
         DenseMatrix64F KCP = new DenseMatrix64F(K.getNumRows(), P.getNumCols());
         CommonOps.mult(temp, P, KCP);
         
         CommonOps.subtractEquals(P, KCP); // P -= K*C*P;
      }

//    ticks++;
//
//    if (ticks % 1000 == 0)
//       K.print(10, 10);
   }

// void do_kalman(Matrix<3,N> C, Matrix<3,3> R, Matrix<1,3> error, int m=3) {
   public DenseMatrix64F K = new DenseMatrix64F(N, 3);

   void doKalman(DenseMatrix64F C, DenseMatrix64F R, DenseMatrix64F error)
   {
      // We throw away the K result

      // Kalman() wants a vector, not an object.  Serialize the
      // state data into this vector, then extract it out again
      // once we're done with the loop.
      double[][] x_vect =
      {
         {q.get(0, 0)}, {q.get(1, 0)}, {q.get(2, 0)}, {q.get(3, 0)}, {bias.get(0, 0)}, {bias.get(1, 0)}, {bias.get(2, 0)}
      };
      DenseMatrix64F X_vect = new DenseMatrix64F(x_vect);

      Kalman(P, X_vect, C, R, error, K);

      q.set(0, 0, X_vect.get(0, 0));
      q.set(1, 0, X_vect.get(1, 0));
      q.set(2, 0, X_vect.get(2, 0));
      q.set(3, 0, X_vect.get(3, 0));

      bias.set(0, 0, X_vect.get(4, 0));
      bias.set(1, 0, X_vect.get(5, 0));
      bias.set(2, 0, X_vect.get(6, 0));
      normalizeQuaternion(q);

//    eulerAngles = QuaternionTools.quat2euler(q);
   }


   void zero(DenseMatrix64F a)
   {
      for (int i = 0; i < a.getNumRows(); i++)
      {
         for (int j = 0; j < a.getNumCols(); j++)
         {
            a.set(i, j, 0.0);
         }
      }
   }

   /**
    *  Convert accelerations to euler angles
    */
   double mag(DenseMatrix64F a)
   {
      double ret = 0.0;
      for (int i = 0; i < a.getNumRows(); i++)
      {
         for (int j = 0; j < a.getNumCols(); j++)
         {
            ret += a.get(i, j) * a.get(i, j);
         }
      }

      return Math.sqrt(ret);
   }


   public void accel2quaternions(DenseMatrix64F a, double heading, double[] quaternions)
   {
      // Accel to euler, then euler to quaternions:
      double g = mag(a);
      double[] yawPitchRoll =
      {
            heading    // Yaw
         -Math.asin(a.get(0, 0) / -g),    // Pitch
         -Math.atan2(a.get(1, 0), -a.get(2, 0)),    // Roll
      };

      Quaternion quaternion = new Quaternion();
      quaternion.setYawPitchRoll(yawPitchRoll);
      quaternions[0] = quaternion.getS();
      quaternions[1] = quaternion.getX();
      quaternions[2] = quaternion.getY();
      quaternions[3] = quaternion.getZ();

      // return the closest one:
      double distanceSquared = 0.0;
      double distanceSquaredToNegative = 0.0;

      for (int i = 0; i < 4; i++)
      {
         distanceSquared += (quaternions[i] - q.get(i, 0)) * (quaternions[i] - q.get(i, 0));
         distanceSquaredToNegative += (-quaternions[i] - q.get(i, 0)) * (-quaternions[i] - q.get(i, 0));
      }

      if (distanceSquaredToNegative < distanceSquared)
      {
         quaternions[0] *= -1.0;
         quaternions[1] *= -1.0;
         quaternions[2] *= -1.0;
         quaternions[3] *= -1.0;
      }
   }


   public void compassUpdate(double heading, DenseMatrix64F accel)
   {
      // Compute our measured and estimated quaternions

      double[] quaternion_m = new double[4];
      accel2quaternions(accel, heading, quaternion_m);

      double q0 = q.get(0, 0);
      double q1 = q.get(1, 0);
      double q2 = q.get(2, 0);
      double q3 = q.get(3, 0);

      // Subtract to get the error in quaternions,
      double[][] quaternion_error = new double[][]
      {
         {quaternion_m[0] - q0}, {quaternion_m[1] - q1}, {quaternion_m[2] - q2}, {quaternion_m[3] - q3}
      };

      DenseMatrix64F quatError = new DenseMatrix64F(quaternion_error);

      /*
       * Compute our C matrix, which relates the quaternion state
       * estimate to the quaternions measured by the accelerometers and
       * the compass.  The other states are all zero.
       */

      C.set(0, 0, 1.0);
      C.set(1, 1, 1.0);
      C.set(2, 2, 1.0);
      C.set(3, 3, 1.0);

      doKalman(C, R, quatError);
   }

// void unpackQuaternion(Matrix quat)
// {
//    double[][] q = quat.getArray();
//    q0 = q[0][0];
//    q1 = q[1][0];
//    q2 = q[2][0];
//    q3 = q[3][0];
// }

   /*
    * Our state update function for the IMU is:
    *
    * Qdot = Wxq * Q
    * bias_dot = [0,0,0]
    * Q += Qdot * dt
    */
   void propagateState(DenseMatrix64F pqr)
   {
      quatW(pqr);    // construct the quaternion W matrix in Wxq
      
      DenseMatrix64F Qdot = new DenseMatrix64F(Wxq.getNumRows(), q.getNumCols());
      CommonOps.mult(Wxq, q, Qdot);    // Qdot = Wxq * q;
      CommonOps.addEquals(q, dt, Qdot);    // q += Qdot * dt;
      normalizeQuaternion(q);

      // Keep copy up-to-date...
//    unpackQuaternion(q);
   }

   void propagateCovariance(DenseMatrix64F A)
   {
      DenseMatrix64F Pdot = new DenseMatrix64F(Q);    // Pdot = Q + A*P*At
      DenseMatrix64F A_transpose = new DenseMatrix64F(A);
      CommonOps.transpose(A_transpose);
      DenseMatrix64F temp = new DenseMatrix64F(A.getNumRows(), P.getNumCols());
      CommonOps.mult(A, P, temp);
      CommonOps.multAdd(temp, A_transpose, Pdot);   // Pdot = Q + A*P*At
      CommonOps.addEquals(Pdot, dt, Pdot);
      trace = CommonOps.trace(P);

      /*
       *  Old Wrong Code:
       * Matrix Pdot = Q.copy();
       * //      System.out.print("propagateCovariance: Pdot = ");   Pdot.print(fmt, 10);
       * Pdot.plusEquals(A.times(P)); // += A * this->P;
       * //      System.out.print("propagateCovariance: Pdot = ");   Pdot.print(fmt, 10);
       * Pdot.plusEquals(P.times(A.transpose())); // += this->P * A.transpose();
       * //      System.out.print("propagateCovariance: Pdot = ");   Pdot.print(fmt, 10);
       * Pdot.timesEquals(dt); // *= this->dt;
       * //      System.out.print("propagateCovariance: Pdot = ");   Pdot.print(fmt, 10);
       * P.plusEquals(Pdot); //+= Pdot;
       * //      System.out.print("propagateCovariance: P = ");   P.print(fmt, 10);
       * trace = P.trace();
       */
   }

   /**
    * Updates the IMU given the rate gyro inputs.
    *
    * @param pqr Matrix Gyro Rate values in order of qd_wy, qd_wx, qd_wz???
    */
   public void imuUpdate(DenseMatrix64F pqr)
   {
      CommonOps.subtractEquals(pqr, bias);

//    unpackQuaternion(q);
      makeAMatrix(pqr);
      propagateState(pqr);
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
   public void initialize(DenseMatrix64F accel, DenseMatrix64F pqr, double heading)
   {
//    System.out.println("Initializing QuaternionBasedJamaFullIMUKalmanFilter. accel = " + QuaternionTools.format4(accel.get(0,0)) + ", " + QuaternionTools.format4(accel.get(1,0)) + ", " + QuaternionTools.format4(accel.get(2,0)));

      setMatrix(bias, pqr);

//    euler = accel2euler(accel, heading);
      double[] quaternions = new double[4];
      accel2quaternions(accel, heading, quaternions);

      double[][] quat_redundant = new double[][]
      {
         {quaternions[0]}, {quaternions[1]}, {quaternions[2]}, {quaternions[3]}
      };

      this.setArray(q, quat_redundant);

//    unpackQuaternion(q);
      @SuppressWarnings("unused")
      double q0 = q.get(0, 0);
      @SuppressWarnings("unused")
      double q1 = q.get(1, 0);
      @SuppressWarnings("unused")
      double q2 = q.get(2, 0);
      @SuppressWarnings("unused")
      double q3 = q.get(3, 0);

//    System.out.println("Initializing QuaternionBasedJamaFullIMUKalmanFilter. q0 = " + QuaternionTools.format4(q0) + ", q1 = " + QuaternionTools.format4(q1) + ", q2 = " + QuaternionTools.format4(q2) + ", q3 = " + QuaternionTools.format4(q3));

//    q = QuaternionTools.euler2quat(eulerAngles);
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

      double q_noise = 5.0;    // 5.0; //0.05; //1.0; //10.0; //250.0;
      double r_noise = 1.0;    // 100.0; //10.0; //25.0; //2.5; //25.0; //100.0; //10.0;

      setNoiseParameters(q_noise, r_noise);

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


   public void setNoiseParameters(double q_noise, double r_noise)
   {
      zero(Q);
      zero(R);

      Q.set(4, 4, q_noise * q_noise);
      Q.set(5, 5, q_noise * q_noise);
      Q.set(6, 6, q_noise * q_noise);

      R.set(0, 0, r_noise * r_noise);
      R.set(1, 1, r_noise * r_noise);
      R.set(2, 2, r_noise * r_noise);
      R.set(3, 3, r_noise * r_noise);
   }

   /**
    * getBias
    *
    * @param i int
    * @return double
    */
   public double getBias(int i)
   {
      return bias.get(i, 0);
   }

   /**
    * getQuaternion
    *
    * @return Matrix
    */
   public void getQuaternion(DenseMatrix64F quaternionMatrix)
   {
      quaternionMatrix.set(0, 0, q.get(0, 0));
      quaternionMatrix.set(1, 0, q.get(1, 0));
      quaternionMatrix.set(2, 0, q.get(2, 0));
      quaternionMatrix.set(3, 0, q.get(3, 0));

//    return q;
   }

}
