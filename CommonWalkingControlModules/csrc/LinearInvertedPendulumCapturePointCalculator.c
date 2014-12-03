#include <math.h>

/**
 * <p>LinearInvertedPendulumStaticCapturePointCalculator</p>
 *
 * <p>Calculates the position, velocity, and acceleration given a Linear Inverted Pendulum Model.
 * This is similar to LinearInvertedPendulumCapturePointCalculator, except that it is static and doesn't use
 * SimulationConstructionSet. It exists as the basis for a C version of this code.</p>
 *
 * <p>Copyright: Copyright (c) 2007</p>
 *
 * <p>Company: IHMC</p>
 *
 * @author Jerry Pratt
 * @version 1.0
 */

   /**
    * Calculates the position, velocity, and acceleration for the Linear Inverted Pendulum Model.
    *
    * @param x0 Initial position with respect to Center of Pressure.
    * @param v0 Initial velocity.
    * @param gravity Gravitational constant.
    * @param comHeight Center of Mass Height. Assumed constant.
    * @param t0 Initial time.
    * @param t Current time.
    *
    * @param returnedState double[] calculated trajectory at t in order of position, velocity, and acceleration.
    */
   void calculateTrajectory(double x0, double v0, double gravity, double comHeight, double t0,
                                                    double t, double *returnedState)
   {
      double S, C0, C1, D0, D1;
      double tau, est, est_minus;
      double pos, vel, acc;

      // Compute the parameters:
      S = sqrt(gravity / comHeight);
      C0 = (x0 + v0 / S) / 2.0;
      C1 = (x0 - v0 / S) / 2.0;
      D0 = (v0 + S * x0) / 2.0;
      D1 = (v0 - S * x0) / 2.0;

      // Compute the exponents:
      tau = t - t0;
      est = exp(S * tau);
      est_minus = exp( -S * tau);

      // Compute the position, velocity, and acceleration:
      pos = C0 * est + C1 * est_minus;
      vel = D0 * est + D1 * est_minus;
      acc = gravity / comHeight * pos;

      returnedState[0] = pos;
      returnedState[1] = vel;
      returnedState[2] = acc;
   }

   /**
    * Calculates the Capture Point with respect to the Center of Mass for the Linear Inverted Pendulum Model.
    *
    * @param gravity Gravitational constant.
    * @param comHeight Center of Mass Height. Assumed constant.
    * @param comVelocity Center of Mass Velocity.
    *
    * @return Predicted Capture Point location.
    */
   double calculateCapturePoint(double comVelocity, double gravity, double comHeight)
   {
      return sqrt(comHeight / gravity) * comVelocity;
   }

   /**
    * Calculates the predicted Center of Mass State at a given time using the Linear Inverted Pendulum Model.
    *
    * @param xCoM Center of Mass Position.
    * @param xCoP Center of Pressure Position.
    * @param vCoM Center of Mass Velocity.
    * @param gravity Gravitational constant.
    * @param comHeight Center of Mass Height. Assumed constant.
    * @param currentTime Current time.
    * @param predictionTime Time of predicted Center of Mass state.
    *
    * @param stateAtCaptureTime double[] in order of position, velocity, and acceleration of the Center of Mass.
    */
   void calculatePredictedCoMState(double xCoM, double xCoP, double vCoM, double gravity,
                                                           double comHeight, double currentTime, double predictionTime,
                                                           double *stateAtCaptureTime)
   {
     calculateTrajectory(xCoM - xCoP, vCoM, gravity, comHeight, currentTime,
                                                        predictionTime, stateAtCaptureTime);

      stateAtCaptureTime[0] = stateAtCaptureTime[0] + xCoP;
   }

   /**
    * Calculates the predicated Capture Point at a given captureTime using the Linear Inverted Pendulum Model.
    *
    * @param xCoM Center of Mass Position.
    * @param xCoP Center of Pressure Position.
    * @param vCoM Center of Mass Velocity.
    * @param gravity Gravitational constant.
    * @param comHeight Center of Mass Height. Assumed constant.
    * @param currentTime Current time.
    * @param captureTime Time of predicted Center of Mass state.
    *
    * @return Predicted Capture point at captureTime.
    */
   double calculatePredictedCapturePoint(double xCoM, double xCoP, double vCoM, double gravity,
                                                             double comHeight, double currentTime, double captureTime)
   {
      double comState[3];
      double capturePoint;

      calculatePredictedCoMState(xCoM, xCoP, vCoM, gravity, comHeight, currentTime, captureTime, comState);

      capturePoint = comState[0] + calculateCapturePoint(comState[1], gravity, comHeight);

      return capturePoint;
   }

