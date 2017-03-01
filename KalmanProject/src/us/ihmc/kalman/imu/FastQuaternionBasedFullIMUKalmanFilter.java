package us.ihmc.kalman.imu;

import us.ihmc.euclid.tuple4D.Quaternion;


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
public class FastQuaternionBasedFullIMUKalmanFilter
{
   @SuppressWarnings("unused")
   private static final boolean verbose = true;

   private static final int N = 7;

   public double trace;

   // Pool of scratch arrays used by various routines
   private double[][] temp71 = new double[N][1];
   private double[][] temp77a = new double[N][N];
   private double[][] temp77b = new double[N][N];
   private double[][] temp47 = new double[4][N];
   private double[][] temp74 = new double[N][4];
   private double[][] temp44 = new double[4][4];
   private double[][] temp41 = new double[4][1];


   /*
    * Covariance matrix and covariance matrix derivative are updated
    * every other state step.  This is because the covariance should change
    * at a rate somewhat slower than the dynamics of the system.
    */
   public double[][] P = new double[N][N];    // Covariance matrix
   private double[][] Pdot = new double[N][N];

   /*
    * A represents the Jacobian of the derivative of the system with respect
    * its states.  We do not allocate the bottom three rows since we know that
    * the derivatives of bias_dot are all zero.
    */
   private double[][] A = new double[N][N];
   private double[][] At = new double[N][N];

   /*
    * Q is our estimate noise variance.  It is supposed to be an NxN
    * matrix, but with elements only on the diagonals.  Additionally,
    * since the quaternion has no expected noise (we can't directly measure
    * it), those are zero.  For the gyro, we expect around 5 deg/sec noise,
    * which is 0.08 rad/sec.  The variance is then 0.08^2 ~= 0.0075.
    */
   private double[][] Q = new double[N][N];    // Noise estimate

   /*
    * R is our measurement noise estimate.  Like Q, it is supposed to be
    * an NxN matrix with elements on the diagonals.  However, since we can
    * not directly measure the gyro bias, we have no estimate for it.
    * We only have an expected noise in the pitch and roll accelerometers
    * and in the compass.
    */
   private double[][] R = new double[4][4];    // State estimate for angles
   private double[][] K = new double[N][4];
   private double[][] Wxq = new double[4][4];
   private double[][] quatError = new double[4][1];

   public double[][] bias = new double[3][1];    // Rate gyro bias offset estimates. The Kalman filter adapts to these.
   public double[][] Quat = new double[4][1];    // Estimated orientation in quaternions.

// public Matrix Qdot = new Matrix(4, 1);
   public double q0, q1, q2, q3;    // Redundant estimated orientation in quaternions.
   private double[][] X = new double[N][1];

   private double dt = .001;
   @SuppressWarnings("unused")
   private static final double PI = Math.PI;

   /*
    * C represents the Jacobian of the measurements of the attitude
    * with respect to the states of the filter.
    */
   private double[][] C = new double[4][N];
   private double[][] Ct = new double[N][4];
   private double[][] E = new double[4][4];
   private double[][] inverseE = new double[4][4];

   @SuppressWarnings("unused")
   private static final java.text.DecimalFormat fmt = new java.text.DecimalFormat();

   public FastQuaternionBasedFullIMUKalmanFilter(double dt)
   {
      this.dt = dt;
      reset(P);
   }

   void unpackQuaternion(double[][] q)
   {
      q0 = q[0][0];
      q1 = q[1][0];
      q2 = q[2][0];
      q3 = q[3][0];
   }


   /*
    * This will construct the quaternion omega matrix
    * W(4,4)
    * p, q, r (rad/sec)
    */
   void quatW(double[][] w_xyz)
   {
      double p = w_xyz[0][0] / 2.0;
      double q = w_xyz[1][0] / 2.0;
      double r = w_xyz[2][0] / 2.0;

      double[][] m =
      {
         {0, -p, -q, -r}, {p, 0, r, -q}, {q, -r, 0, p}, {r, q, -p, 0}
      };
      KalmanMatrixTools.setArray(m, Wxq);
   }

   void makeAMatrix(double[][] pqr)
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
      double[][] m =
      {
         {
            Wxq[0][0], Wxq[0][1], Wxq[0][2], Wxq[0][3], q1 / 2, q2 / 2, q3 / 2
         },
         {
            Wxq[1][0], Wxq[1][1], Wxq[1][2], Wxq[1][3], -q0 / 2, q3 / 2, -q2 / 2
         },
         {
            Wxq[2][0], Wxq[2][1], Wxq[2][2], Wxq[2][3], -q3 / 2, -q0 / 2, q1 / 2
         },
         {
            Wxq[3][0], Wxq[3][1], Wxq[3][2], Wxq[3][3], q2 / 2, -q1 / 2, -q0 / 2
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

      KalmanMatrixTools.setArray(m, A);
   }

   void Kalman(double[][] P)
   {
      // E = C*P*Ct+R
      KalmanMatrixTools.transpose(C, Ct);
      KalmanMatrixTools.mul(C, P, temp47);
      KalmanMatrixTools.mul(temp47, Ct, temp44);
      KalmanMatrixTools.add(temp44, R, E);

      // K = P*Ct*inv(E)
      KalmanMatrixTools.mul(P, Ct, temp74);
      KalmanMatrixTools.inverse44(E, inverseE);
      KalmanMatrixTools.mul(temp74, inverseE, K);

      // X += K*err;
      KalmanMatrixTools.mul(K, quatError, temp71);
      KalmanMatrixTools.add(X, temp71, X);

      // P -= K*C*P;
      KalmanMatrixTools.mul(K, C, temp77a);
      KalmanMatrixTools.mul(temp77a, P, temp77b);
      KalmanMatrixTools.sub(P, temp77b, P);
   }

   void doKalman()
   {
      /*
       * Compute our C matrix, which relates the quaternion state
       * estimate to the quaternions measured by the accelerometers and
       * the compass.  The other states are all zero.
       */

      KalmanMatrixTools.identity(C);
      X[0][0] = Quat[0][0];
      X[1][0] = Quat[1][0];
      X[2][0] = Quat[2][0];
      X[3][0] = Quat[3][0];
      X[4][0] = bias[0][0];
      X[5][0] = bias[1][0];
      X[6][0] = bias[2][0];

      Kalman(P);

      Quat[0][0] = X[0][0];
      Quat[1][0] = X[1][0];
      Quat[2][0] = X[2][0];
      Quat[3][0] = X[3][0];

      bias[0][0] = X[4][0];
      bias[1][0] = X[5][0];
      bias[2][0] = X[6][0];
      KalmanMatrixTools.normalize(Quat);
   }

   // http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/index.htm

   /**
    *  Convert axis/angle directly to quaternion
    */
   public double[] axis2quat(double[][] axis, double heading)
   {
      @SuppressWarnings("unused")
      double mag = KalmanMatrixTools.mag(axis);
      double temp[][] = new double[3][1];
      KalmanMatrixTools.normalize(axis, temp);
      double s = Math.sin(heading / 2);
      double x = temp[0][0] * s;
      double y = temp[1][0] * s;
      double z = temp[2][0] * s;
      double w = Math.cos(heading / 2);

      return new double[] {x, y, z, w};
   }


   /**
    *  Convert accelerations and heading directly to quaternion
    */
   public double[] accelerometers2quat(double[][] accel, double heading)
   {
      double mag = KalmanMatrixTools.mag(accel);
      KalmanMatrixTools.normalize(accel);
      double x = accel[0][0];
      double y = accel[1][0];
      double z = accel[2][0];

      double shphi0 = -y / mag;
      double chphi0 = -z / mag;

      double shtheta0 = x / mag;    // Math.sin(theta);
      double chtheta0 = 1.0 - x / mag;    // Math.cos(theta);

      double shpsi0 = Math.sin(heading / 2.0);
      double chpsi0 = Math.cos(heading / 2.0);

//    System.out.println(shphi0 + " " + chphi0 + " " + shtheta0 + " " + chtheta0 + " " + shpsi0 + " " + chpsi0);
      double[] quaternions = {chphi0 * chtheta0 * chpsi0 + shphi0 * shtheta0 * shpsi0, -chphi0 * shtheta0 * shpsi0 + shphi0 * chtheta0 * chpsi0,
                              chphi0 * shtheta0 * chpsi0 + shphi0 * chtheta0 * shpsi0, chphi0 * chtheta0 * shpsi0 - shphi0 * shtheta0 * chpsi0};

      return quaternions;

   }

   /**
    *  Convert accelerations to euler angles
    */
   public double[] accel2quat(double[][] accel, double heading)
   {
      // Accel to euler, then euler to quaternions:
      double mag = KalmanMatrixTools.mag(accel);
      double[] yawPitchRoll = {heading, Math.asin(accel[0][0] / mag), -Math.atan2(accel[1][0], -accel[2][0])};
      double[] quaternions = new double[4];
      Quaternion quaternion = new Quaternion();
      quaternion.setYawPitchRoll(yawPitchRoll);
      quaternions[0] = quaternion.getS();
      quaternions[1] = quaternion.getX();
      quaternions[2] = quaternion.getY();
      quaternions[3] = quaternion.getZ();

//    System.out.println(quaternions[0] + " " + quaternions[1] + " " + quaternions[2] + " " + quaternions[3]);
//    double[] quats = accelerometers2quaternions(a,heading);
      @SuppressWarnings("unused")
      double[] quats = axis2quat(accel, heading);

//    System.out.println(quats[0] + " " + quats[1] + " " + quats[2] + " " + quats[3]);
//    quaternions = quats;

      // return the closest one:
      double distanceSquared = 0.0;
      double distanceSquaredToNegative = 0.0;

      for (int i = 0; i < 4; i++)
      {
         distanceSquared += (quaternions[i] - Quat[i][0]) * (quaternions[i] - Quat[i][0]);
         distanceSquaredToNegative += (-quaternions[i] - Quat[i][0]) * (-quaternions[i] - Quat[i][0]);
      }

      if (distanceSquaredToNegative < distanceSquared)
      {
         quaternions[0] *= -1.0;
         quaternions[1] *= -1.0;
         quaternions[2] *= -1.0;
         quaternions[3] *= -1.0;
      }

      return quaternions;
   }

// /*
//  * This will convert from quaternions to euler angles
//  * q(4,1) -> euler[phi;theta;psi] (rad)
//  */
// public double[][] quat2euler(double[][] quat) {
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

   public void setNoiseParameters(double q_noise, double r_noise)
   {
      KalmanMatrixTools.zero(Q);
      KalmanMatrixTools.zero(R);

      Q[4][4] = q_noise * q_noise;
      Q[5][5] = q_noise * q_noise;
      Q[6][6] = q_noise * q_noise;

      R[0][0] = r_noise * r_noise;
      R[1][1] = r_noise * r_noise;
      R[2][2] = r_noise * r_noise;
      R[3][3] = r_noise * r_noise;
   }


   public void compassUpdate(double heading, double[][] accel)
   {
      // Compute our measured and estimated quaternions

      double[] quaternion_m = accel2quat(accel, heading);

      // Subtract to get the error in quaternions,
      quatError[0][0] = quaternion_m[0] - q0;
      quatError[1][0] = quaternion_m[1] - q1;
      quatError[2][0] = quaternion_m[2] - q2;
      quatError[3][0] = quaternion_m[3] - q3;

      doKalman();
   }

   /*
    * Our state update function for the IMU is:
    *
    * Qdot = Wxq * Q
    * bias_dot = [0,0,0]
    * Q += Qdot * dt
    */
   void propagateState(double[][] pqr)
   {
      quatW(pqr);    // constructs quaternion W matrix in Wxq

      // q = q + W*q*dt;
      KalmanMatrixTools.mulScalarMul(Wxq, Quat, temp41, dt);
      KalmanMatrixTools.add(Quat, temp41, Quat);
      KalmanMatrixTools.normalize(Quat);

      // Keep copy up-to-date...
      unpackQuaternion(Quat);
   }

   void propagateCovariance(double[][] a)
   {
      // Pdot = A*P+Q;
      KalmanMatrixTools.setArray(Q, Pdot);
      KalmanMatrixTools.mul(a, P, temp77a);
      KalmanMatrixTools.add(Pdot, temp77a, Pdot);

      // Pdot = P*At + A*P+Q;
      KalmanMatrixTools.transpose(a, At);
      KalmanMatrixTools.mul(P, At, temp77a);
      KalmanMatrixTools.add(Pdot, temp77a, Pdot);

      // Pdot = (P*At + A*P+Q)*dt;
      KalmanMatrixTools.mul(Pdot, dt, Pdot);

      // P = P + Pdot;
      KalmanMatrixTools.add(P, Pdot, P);
      trace = KalmanMatrixTools.trace(P);
   }

   /**
    * Updates the IMU given the rate gyro inputs.
    *
    * @param pqr Matrix Gyro Rate values in order of qd_wy, qd_wx, qd_wz???
    */
   public void imuUpdate(double[][] PQR)
   {
      KalmanMatrixTools.sub(PQR, bias, PQR);
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
   public void initialize(double[][] Accel, double[][] PQR, double heading)
   {
      KalmanMatrixTools.setArray(PQR, bias);
      double[] quaternions = accel2quat(Accel, heading);
      Quat[0][0] = quaternions[0];
      Quat[1][0] = quaternions[1];
      Quat[2][0] = quaternions[2];
      Quat[3][0] = quaternions[3];
   }

   public void reset(double[][] P)
   {
      /*
       * The covariance matrix is probably initialized incorrectly.
       * It should be 1 for all diagonal elements of Q that are 0
       * and zero everywhere else.
       */
      double q_noise = 5.0;    // 5.0; //0.05; //1.0; //10.0; //250.0;
      double r_noise = 1.0;    // 100.0; //10.0; //25.0; //2.5; //25.0; //100.0; //10.0;

      KalmanMatrixTools.identity(P);
      KalmanMatrixTools.zero(Q);
      KalmanMatrixTools.zero(R);

      Q[4][4] = q_noise * q_noise;
      Q[5][5] = q_noise * q_noise;
      Q[6][6] = q_noise * q_noise;
      R[0][0] = r_noise * r_noise;
      R[1][1] = r_noise * r_noise;
      R[2][2] = r_noise * r_noise;
      R[3][3] = r_noise * r_noise;

   }

}
