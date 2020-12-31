package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrix;
import us.ihmc.commons.MathTools;

public class OrientationCoefficientJacobianCalculator
{
   public static void calculateAngularJacobian(int startIndex, double time, DMatrix jacobianToPack, int derivative, double scale)
   {
      switch (derivative)
      {
         case 0:
            calculateOrientationJacobian(startIndex, time, jacobianToPack, scale);
            break;
         case 1:
            calculateAngularVelocityJacobian(startIndex, time, jacobianToPack, scale);
            break;
         case 2:
            calculateAngularAccelerationJacobian(startIndex, time, jacobianToPack, scale);
            break;
         case 3:
            calculateAngularJerkJacobian(startIndex, jacobianToPack, scale);
            break;
         default:
            throw new IllegalArgumentException("Derivative order must be less than 4.");
      }
   }


   public static void calculateOrientationJacobian(int startIndex, double time, DMatrix orientationJacobianToPack, double scale)
   {
      double c3 = scale;

      add(orientationJacobianToPack, 2, startIndex + 3, c3);
      add(orientationJacobianToPack, 1, startIndex + 7, c3);
      add(orientationJacobianToPack, 0, startIndex + 11, c3);

      if (!MathTools.epsilonEquals(time, 0.0, 1e-5))
      {
         time = Math.min(MPCQPInputCalculator.sufficientlyLongTime, time);
         double c2 = time * c3;
         double c1 = time * c2;
         double c0 = time * c1;
         add(orientationJacobianToPack, 2, startIndex, c0);
         add(orientationJacobianToPack, 2, startIndex + 1, c1);
         add(orientationJacobianToPack, 2, startIndex + 2, c2);
         add(orientationJacobianToPack, 1, startIndex + 4, c0);
         add(orientationJacobianToPack, 1, startIndex + 5, c1);
         add(orientationJacobianToPack, 1, startIndex + 6, c2);
         add(orientationJacobianToPack, 0, startIndex + 8, c0);
         add(orientationJacobianToPack, 0, startIndex + 9, c1);
         add(orientationJacobianToPack, 0, startIndex + 10, c2);
      }
   }

   public static void calculateAngularVelocityJacobian(int startIndex, double time, DMatrix velocityJacobianToPack, double scale)
   {
      double c2 = scale;

      add(velocityJacobianToPack, 2, startIndex + 2, c2);
      add(velocityJacobianToPack, 1, startIndex + 6, c2);
      add(velocityJacobianToPack, 0, startIndex + 10, c2);

      if (!MathTools.epsilonEquals(time, 0.0, 1e-5))
      {
         time = Math.min(MPCQPInputCalculator.sufficientlyLongTime, time);
         double c0 = 3.0 * time * time * scale;
         double c1 = 2.0 * time * scale;
         add(velocityJacobianToPack, 2, startIndex, c0);
         add(velocityJacobianToPack, 2, startIndex + 1, c1);
         add(velocityJacobianToPack, 1, startIndex + 4, c0);
         add(velocityJacobianToPack, 1, startIndex + 5, c1);
         add(velocityJacobianToPack, 0, startIndex + 8, c0);
         add(velocityJacobianToPack, 0, startIndex + 9, c1);
      }
   }

   public static void calculateAngularAccelerationJacobian(int startIndex, double time, DMatrix accelerationJacobianToPack, double scale)
   {
      double c1 = 2.0 * scale;

      add(accelerationJacobianToPack, 2, startIndex + 1, c1);
      add(accelerationJacobianToPack, 1, startIndex + 5, c1);
      add(accelerationJacobianToPack, 0, startIndex + 9, c1);

      if (!MathTools.epsilonEquals(time, 0.0, 1e-5))
      {
         time = Math.min(MPCQPInputCalculator.sufficientlyLongTime, time);
         double c0 = 6.0 * time * scale;
         add(accelerationJacobianToPack, 2, startIndex, c0);
         add(accelerationJacobianToPack, 1, startIndex + 4, c0);
         add(accelerationJacobianToPack, 0, startIndex + 8, c0);
      }
   }

   public static void calculateAngularJerkJacobian(int startIndex, DMatrix jerkJacobianToPack, double scale)
   {
      double c0 = 6.0 * scale;

      add(jerkJacobianToPack, 2, startIndex, c0);
      add(jerkJacobianToPack, 1, startIndex + 4, c0);
      add(jerkJacobianToPack, 0, startIndex + 8, c0);
   }

   private static void add(DMatrix matrixToPack, int row, int col, double value)
   {
      matrixToPack.set(row, col, value + matrixToPack.get(row, col));
   }
}
