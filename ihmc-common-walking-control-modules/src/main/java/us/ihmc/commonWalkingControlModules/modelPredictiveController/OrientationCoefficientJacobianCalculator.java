package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrix;
import us.ihmc.commons.MathTools;

public class OrientationCoefficientJacobianCalculator
{
   public static void calculateAngularJacobian(int yawStart, int pitchStart, int rollStart, double omega, double time, DMatrix jacobianToPack, int derivative, double scale)
   {
      switch (derivative)
      {
         case 0:
            calculateOrientationJacobian(yawStart, pitchStart, rollStart, omega, time, jacobianToPack, scale);
            break;
         case 1:
            calculateAngularVelocityJacobian(yawStart, pitchStart, rollStart, omega, time, jacobianToPack, scale);
            break;
         case 2:
            calculateAngularAccelerationJacobian(yawStart, pitchStart, rollStart, omega, time, jacobianToPack, scale);
            break;
         case 3:
            calculateAngularJerkJacobian(yawStart, pitchStart, rollStart, omega, time, jacobianToPack, scale);
            break;
         default:
            throw new IllegalArgumentException("Derivative order must be less than 4.");
      }
   }


   public static void calculateOrientationJacobian(int yawStart, int pitchStart, int rollStart, double omega, double time, DMatrix orientationJacobianToPack, double scale)
   {
      double c0 = Math.exp(omega * time);
      double c1 = 1.0 / c0;
      double c5 = scale;

      add(orientationJacobianToPack, 0, rollStart, c0);
      add(orientationJacobianToPack, 1, pitchStart, c0);
      add(orientationJacobianToPack, 2, yawStart, c0);
      add(orientationJacobianToPack, 0, rollStart + 1, c1);
      add(orientationJacobianToPack, 1, pitchStart + 1, c1);
      add(orientationJacobianToPack, 2, yawStart + 1, c1);
      add(orientationJacobianToPack, 0, rollStart + 5, c5);
      add(orientationJacobianToPack, 1, pitchStart + 5, c5);
      add(orientationJacobianToPack, 2, yawStart + 5, c5);

      if (!MathTools.epsilonEquals(time, 0.0, 1e-5))
      {
         time = Math.min(MPCQPInputCalculator.sufficientlyLongTime, time);
         double c4 = time * c5;
         double c3 = time * c4;
         double c2 = time * c3;
         add(orientationJacobianToPack, 0, rollStart + 2, c2);
         add(orientationJacobianToPack, 0, rollStart + 3, c3);
         add(orientationJacobianToPack, 0, rollStart + 4, c4);

         add(orientationJacobianToPack, 1, pitchStart + 2, c2);
         add(orientationJacobianToPack, 1, pitchStart + 3, c3);
         add(orientationJacobianToPack, 1, pitchStart + 4, c4);

         add(orientationJacobianToPack, 2, yawStart + 2, c2);
         add(orientationJacobianToPack, 2, yawStart + 3, c3);
         add(orientationJacobianToPack, 2, yawStart + 4, c4);
      }
   }

   public static void calculateAngularVelocityJacobian(int yawStart, int pitchStart, int rollStart, double omega, double time, DMatrix velocityJacobianToPack, double scale)
   {
      double exponential = Math.exp(omega * time);
      double c0 = omega * exponential;
      double c1 = -omega / exponential;
      double c4 = scale;

      add(velocityJacobianToPack, 0, rollStart, c0);
      add(velocityJacobianToPack, 1, pitchStart, c0);
      add(velocityJacobianToPack, 2, yawStart, c0);
      add(velocityJacobianToPack, 0, rollStart + 1, c1);
      add(velocityJacobianToPack, 1, pitchStart + 1, c1);
      add(velocityJacobianToPack, 2, yawStart + 1, c1);
      add(velocityJacobianToPack, 0, rollStart + 4, c4);
      add(velocityJacobianToPack, 1, pitchStart + 4, c4);
      add(velocityJacobianToPack, 2, yawStart + 4, c4);

      if (!MathTools.epsilonEquals(time, 0.0, 1e-5))
      {
         time = Math.min(MPCQPInputCalculator.sufficientlyLongTime, time);
         double c2 = 3.0 * time * time * scale;
         double c3 = 2.0 * time * scale;
         add(velocityJacobianToPack, 0, rollStart + 2, c2);
         add(velocityJacobianToPack, 0, rollStart + 3, c3);
         add(velocityJacobianToPack, 1, pitchStart + 2, c2);
         add(velocityJacobianToPack, 1, pitchStart + 3, c3);
         add(velocityJacobianToPack, 2, yawStart + 2, c2);
         add(velocityJacobianToPack, 2, yawStart + 3, c3);

      }
   }

   public static void calculateAngularAccelerationJacobian(int yawStart, int pitchStart, int rollStart, double omega, double time, DMatrix accelerationJacobianToPack, double scale)
   {
      double exponential = Math.exp(omega * time);
      double omega2 = omega * omega;
      double c0 = omega2 * exponential;
      double c1 = omega2 / exponential;
      double c3 = 2.0 * scale;

      add(accelerationJacobianToPack, 0, rollStart, c0);
      add(accelerationJacobianToPack, 1, pitchStart, c0);
      add(accelerationJacobianToPack, 2, yawStart, c0);

      add(accelerationJacobianToPack, 0, rollStart + 1, c1);
      add(accelerationJacobianToPack, 1, pitchStart + 1, c1);
      add(accelerationJacobianToPack, 2, yawStart + 1, c1);

      add(accelerationJacobianToPack, 0, rollStart + 3, c3);
      add(accelerationJacobianToPack, 1, pitchStart + 3, c3);
      add(accelerationJacobianToPack, 2, yawStart + 3, c3);

      if (!MathTools.epsilonEquals(time, 0.0, 1e-5))
      {
         time = Math.min(MPCQPInputCalculator.sufficientlyLongTime, time);
         double c2 = 6.0 * time * scale;
         add(accelerationJacobianToPack, 0, rollStart + 2, c2);
         add(accelerationJacobianToPack, 1, pitchStart + 2, c2);
         add(accelerationJacobianToPack, 2, yawStart + 2, c2);
      }
   }

   public static void calculateAngularJerkJacobian(int yawStart, int pitchStart, int rollStart, double omega, double time, DMatrix jerkJacobianToPack, double scale)
   {
      double exponential = Math.exp(omega * time);
      double omega3 = omega * omega * omega;
      double c0 = omega3 * exponential;
      double c1 = -omega3 / exponential;
      double c2 = 6.0 * scale;

      add(jerkJacobianToPack, 0, rollStart, c0);
      add(jerkJacobianToPack, 1, pitchStart, c0);
      add(jerkJacobianToPack, 2, yawStart, c0);

      add(jerkJacobianToPack, 0, rollStart + 1, c1);
      add(jerkJacobianToPack, 1, pitchStart + 1, c1);
      add(jerkJacobianToPack, 2, yawStart + 1, c1);

      add(jerkJacobianToPack, 0, rollStart + 3, c2);
      add(jerkJacobianToPack, 1, pitchStart + 3, c2);
      add(jerkJacobianToPack, 2, yawStart + 3, c2);
   }

   private static void add(DMatrix matrixToPack, int row, int col, double value)
   {
      matrixToPack.set(row, col, value + matrixToPack.get(row, col));
   }
}
