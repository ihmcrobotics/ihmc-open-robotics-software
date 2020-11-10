package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrix;
import us.ihmc.commons.MathTools;

public class CoMCoefficientJacobianCalculator
{
   public static void calculateJacobian(int segmentId, double time, DMatrix jacobianToPack, int derivative)
   {
      switch (derivative)
      {
         case 0:
            calculatePositionJacobian(segmentId, time, jacobianToPack);
            break;
         case 1:
            calculateVelocityJacobian(segmentId, time, jacobianToPack);
            break;
         case 2:
            calculateAccelerationJacobian(segmentId, time, jacobianToPack);
            break;
         default:
            throw new IllegalArgumentException("Derivative order must be less than 3.");
      }
   }

   public static void calculatePositionJacobian(int segmentId, double time, DMatrix positionJacobian)
   {
      if (!MathTools.epsilonEquals(time, 0.0, 1e-5))
      {
         int startIndex = MPCIndexHandler.comCoefficientsPerSegment * segmentId;
         double c1 = time * time;
         double c0 = time * c1;
         positionJacobian.set(0, startIndex, c0);
         positionJacobian.set(0, startIndex + 1, c1);
         positionJacobian.set(1, startIndex + 2, c0);
         positionJacobian.set(1, startIndex + 3, c1);
         positionJacobian.set(2, startIndex + 4, c0);
         positionJacobian.set(2, startIndex + 5, c1);
      }
   }

   public static void calculateVelocityJacobian(int segmentId, double time, DMatrix velocityJacobian)
   {
      if (!MathTools.epsilonEquals(time, 0.0, 1e-5))
      {
         int startIndex = MPCIndexHandler.comCoefficientsPerSegment * segmentId;
         double c0 = 3.0 * time * time;
         double c1 = 2.0 * time;
         velocityJacobian.set(0, startIndex, c0);
         velocityJacobian.set(0, startIndex + 1, c1);
         velocityJacobian.set(1, startIndex + 2, c0);
         velocityJacobian.set(1, startIndex + 3, c1);
         velocityJacobian.set(2, startIndex + 4, c0);
         velocityJacobian.set(2, startIndex + 5, c1);
      }
   }

   public static void calculateAccelerationJacobian(int segmentId, double time, DMatrix accelerationJacobian)
   {
      int startIndex = MPCIndexHandler.comCoefficientsPerSegment * segmentId;
      double c1 = 2.0;
      accelerationJacobian.set(0, startIndex + 1, c1);
      accelerationJacobian.set(1, startIndex + 3, c1);
      accelerationJacobian.set(2, startIndex + 5, c1);

      if (!MathTools.epsilonEquals(time, 0.0, 1e-5))
      {
         double c0 = 6.0 * time;
         accelerationJacobian.set(0, startIndex, c0);
         accelerationJacobian.set(1, startIndex + 2, c0);
         accelerationJacobian.set(2, startIndex + 4, c0);
      }
   }
}
