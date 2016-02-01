package us.ihmc.kalman.imu;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;



public class SingleAngleIMUKalmanFilter
{
   private double angle;
   private double q_bias;
   private double rate;

   /*
    *  P is the covarianve matrix, updated every time step to determine how well
    *  the sensors are tracking the actual state.
    *  R represents the measurement covariance noise.  In this case, it is a 1x1
    *  matrix that says we expect 0.3 radian jitter from the accelerometer.
    *  Q is a 2x2 matrix that represents the process covariance noise.  In this case
    *  it indicates how much we trust the accelerometer relative to the gyros.
    */
   private double P[][] = new double[2][2];
   private double Pdot[] = new double[2 * 2];

   private double R_angle_default = 0.02;    // 0.02;    //6.0; //0.3;       // Modified from 6.0, by Shervin to use accelerometers less. 2/22/07.
   private double Q_angle_default = 0.001;    // 0.02; //
   private double Q_gyro_default = 0.003;    // 0.0001; //

   private double dt = .001;

   private double angle_m;
   private double angle_err;

   private double C_0;

   private double PCt_0;
   private double PCt_1;

   private double E;

   private double K_0;
   private double K_1;

   private double t_0;
   private double t_1;

   private DoubleYoVariable R_angle, Q_angle, Q_gyro;    // K1;

   public SingleAngleIMUKalmanFilter(YoVariableRegistry reg, YoVariableRegistry holder)
   {
//    K1 = new YoVariable("K1", reg);
      if (reg != null)
      {
         R_angle = (DoubleYoVariable)holder.getVariable("R_angle");
         Q_angle = (DoubleYoVariable)holder.getVariable("Q_angle");
         Q_gyro = (DoubleYoVariable)holder.getVariable("Q_gyro");

         if (R_angle == null)
         {
            R_angle = new DoubleYoVariable("R_angle", reg);
            Q_angle = new DoubleYoVariable("Q_angle", reg);
            Q_gyro = new DoubleYoVariable("Q_gyro", reg);
         }

         R_angle.set(R_angle_default);
         Q_angle.set(Q_angle_default);
         Q_gyro.set(Q_gyro_default);
      }

   }

   /*
    *   stateUpdate is called every dt with a biased gyro measurement by the user.
    *   It updates the current angle and rate estimate.
    *
    *   The pitch gyro measurement should be scaled into real units, but does not
    *   need any bias removal.  The filter will track the bias.
    *
    *   Our state vector is:
    *
    *       X = [ angle, gyro_bias ]
    *
    *   It runs the state estimation forward via the state functions:
    *
    *       Xdot = [ angle_dot, gyro_bias_dot ]
    *       angle_dot = gyro - gyro_bias
    *       gyro_bias_dot = 0
    *
    *   And updates the covariance matrix via the function:
    *
    *       Pdot = A*P + P*A' + Q
    *
    *   A is the jacobian of Xdot with respect to the states:
    *
    *       A = [ d(angle_dot)/d(angle)     d(angle_dot)/d(gyro_bias) ]
    *           [ d(gyro_bias_dot)/d(angle) d(gyro_bias_dot)/d(gyro_bias) ]
    *
    *         = [0 -1]
    *           [0  0]
    *
    */

   public void stateUpdate(double q_m)
   {
      double q = q_m - q_bias;

      /* Compute the derivative of the covariance matrix */
      Pdot[0] = (Q_angle == null) ? Q_angle_default : Q_angle.getDoubleValue() - P[0][1] - P[1][0];
      Pdot[1] = -P[1][1];
      Pdot[2] = -P[1][1];
      Pdot[3] = (Q_gyro == null) ? Q_gyro_default : Q_gyro.getDoubleValue();

      /* Store the unbiased gyro eztimate */
      rate = q;

      /*
       *  Update our angle estimate:
       *  angle += angle_dot * dt
       *        += (gyro - gyro_bias) * dt
       *        += q * dt
       */
      angle += q * dt;

      /* Update the covariance matrix */
      P[0][0] += Pdot[0] * dt;
      P[0][1] += Pdot[1] * dt;
      P[1][0] += Pdot[2] * dt;
      P[1][1] += Pdot[3] * dt;
   }

   /*
    *    kalmanUpdate is called when a new accelerometer measurement is available.
    *    ax_m and az_m do not need to be scaled into actual units but must be zeroed
    *    and have the same scale.
    *
    *    This does not have to be called every time step.  For a two-axis accelerometer
    *    mounted perpendicular to the rotation axis, we can compute the angle for
    *    the full 360 degree rotation with no lineariztion errors by using the
    *    arctangent of the two readings.
    *
    *    The C matrix is a 1x2 matrix that is the Jacobian matrix of the measurement
    *    value with respect to the states.  In this case, C is:
    *
    *         C = [ d(angle_m)/d(angle)     d(angle_m)/d(gyro_bias) ]
    *             [ 1 0 ]
    *
    *    because the angle measurement directly corresponds to the angle estimate
    *    and the angle measurement has no relation to the gyro bias.
    */
   public void kalmanUpdate(double ax_m, double az_m)
   {
//    **************Time Update ("Predict")********************
//    (1) Project state ahead
      // This step done only once in state_update...
      // Compute x = A*x + B*u

//    (2) Project the error covariance ahead
      // P = A*P*At + Q

      angle_m = Math.atan2(ax_m, az_m);
      angle_err = angle_m - angle;

      /*
       *   C_0 shows how the state measurement directly relates to the state estimate.
       *   C_1 shows that the state measurement does not relate to the gyro bias estimate.
       *   Since we don't actually use this, we drop all the terms related to it.
       */
      C_0 = 1;
      PCt_0 = C_0 * P[0][0];
      PCt_1 = C_0 * P[1][0];

//    *********************************************************

//    **************Measurement Update ("Correct")**************
//    (1) Compute the Kalman gain, K
      // K = P * Ht * inv(H*P*Ht + R)
      E = (R_angle == null) ? R_angle_default : R_angle.getDoubleValue() + C_0 * PCt_0;
      K_0 = PCt_0 / E;
      K_1 = PCt_1 / E;

//    (2) Update estimate with measurement zk
      // x = x + K*(z - H*x)
      angle += K_0 * angle_err;
      q_bias += K_1 * angle_err;

//    (3) Update the error covariance
      // P = (I - K*H)*P
      // -= K*H*P
      t_0 = PCt_0;
      t_1 = C_0 * P[0][1];
      P[0][0] -= K_0 * t_0;
      P[0][1] -= K_0 * t_1;
      P[1][0] -= K_1 * t_0;
      P[1][1] -= K_1 * t_1;

//    *********************************************************
   }

   public double getAngle()
   {
      return angle;
   }

   public double getAngle_M()
   {
      return angle_m;
   }

   public double getQbias()
   {
      return q_bias;
   }

   public void setQbias(double q_bias)
   {
      this.q_bias = q_bias;
   }

   public double getRate()
   {
      return rate;
   }

   public void setAngle(double angle)
   {
      this.angle = angle;
   }

   public void setR_angle(double angle)
   {
      R_angle.set(angle);
   }

   public void setQ_angle(double angle)
   {
      Q_angle.set(angle);
   }

   public void setQ_gyro(double angle)
   {
      Q_gyro.set(angle);
   }

   public void setDT(double dt)
   {
      this.dt = dt;
   }

   public void setAngleBasedOnAccelerometers()
   {
      this.angle = angle_m;
   }

}
