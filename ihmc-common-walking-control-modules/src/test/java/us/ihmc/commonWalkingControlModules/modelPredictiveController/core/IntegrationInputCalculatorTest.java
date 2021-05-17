package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPoint;
import us.ihmc.convexOptimization.quadraticProgram.SimpleEfficientActiveSetQPSolver;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.robotics.MatrixMissingTools;

import static us.ihmc.commonWalkingControlModules.modelPredictiveController.core.MPCQPInputCalculator.sufficientlyLargeValue;
import static us.ihmc.robotics.Assert.assertEquals;

public class IntegrationInputCalculatorTest
{

   @Test
   public void testComputeRhoAccelerationIntegration()
   {
      int numberOfBasisVectorsPerContactPoint = 4;
      Point2D point = new Point2D(2.5, 1.5);
      MPCContactPoint contactPoint = new MPCContactPoint(numberOfBasisVectorsPerContactPoint);
      double mu = 0.8;

      contactPoint.computeBasisVectors(point, new FramePose3D(), 0.0, mu);

      double omega = 3.0;
      double duration = 0.7;
      double goalValueForBasis = 0.2;

      DMatrixRMaj accelerationIntegrationHessian = new DMatrixRMaj(numberOfBasisVectorsPerContactPoint * LinearMPCIndexHandler.coefficientsPerRho,
                                                                   numberOfBasisVectorsPerContactPoint * LinearMPCIndexHandler.coefficientsPerRho);
      DMatrixRMaj accelerationIntegrationGradient = new DMatrixRMaj(numberOfBasisVectorsPerContactPoint * LinearMPCIndexHandler.coefficientsPerRho, 1);

      DMatrixRMaj calculatedHessian = new DMatrixRMaj(accelerationIntegrationHessian);
      DMatrixRMaj calculatedGradient = new DMatrixRMaj(accelerationIntegrationGradient);

      IntegrationInputCalculator.computeRhoAccelerationIntegrationMatrix(0, calculatedGradient, calculatedHessian, contactPoint, duration, omega, goalValueForBasis);

      double positiveExponential = Math.min(Math.exp(omega * duration), sufficientlyLargeValue);
      double positiveExponential2 = Math.min(positiveExponential * positiveExponential, sufficientlyLargeValue);
      double negativeExponential = 1.0 / positiveExponential;
      double negativeExponential2 = negativeExponential * negativeExponential;
      double duration2 = duration * duration;
      double duration3 = duration * duration2;
      double omega2 = omega * omega;
      double omega3 = omega * omega2;
      double omega4 = omega2 * omega2;
      double c00 = omega3 / 2.0 * (positiveExponential2 - 1.0);
      double c01 = omega4 * duration;
      double c02 = 6.0 * (positiveExponential * (omega * duration - 1.0) + 1.0);
      double c03 = 2.0 * omega * (positiveExponential - 1.0);
      double c11 = -omega3 / 2.0 * (negativeExponential2 - 1.0);
      double c12 = -6.0 * (negativeExponential * (omega * duration + 1.0) - 1.0);
      double c13 = -2.0 * omega * (negativeExponential - 1.0);
      double c22 = 12.0 * duration3;
      double c23 = 6.0 * duration2;
      double c33 = 4.0 * duration;

      double g0 = omega * (positiveExponential - 1.0) * goalValueForBasis;
      double g1 = -omega * (negativeExponential - 1.0) * goalValueForBasis;
      double g2 = 3.0 * duration2 * goalValueForBasis;
      double g3 = 2.0 * duration * goalValueForBasis;

      for (int basisVectorIndexI = 0; basisVectorIndexI < numberOfBasisVectorsPerContactPoint; basisVectorIndexI++)
      {
         int startIdxI = basisVectorIndexI * LinearMPCIndexHandler.coefficientsPerRho;

         accelerationIntegrationHessian.unsafe_set(startIdxI, startIdxI, c00);
         accelerationIntegrationHessian.unsafe_set(startIdxI, startIdxI + 1, c01);
         accelerationIntegrationHessian.unsafe_set(startIdxI, startIdxI + 2, c02);
         accelerationIntegrationHessian.unsafe_set(startIdxI, startIdxI + 2, c03);
         accelerationIntegrationHessian.unsafe_set(startIdxI + 1, startIdxI, c01);
         accelerationIntegrationHessian.unsafe_set(startIdxI + 1, startIdxI + 1, c11);
         accelerationIntegrationHessian.unsafe_set(startIdxI + 1, startIdxI + 2, c12);
         accelerationIntegrationHessian.unsafe_set(startIdxI + 1, startIdxI + 3, c13);
         accelerationIntegrationHessian.unsafe_set(startIdxI + 2, startIdxI, c02);
         accelerationIntegrationHessian.unsafe_set(startIdxI + 2, startIdxI + 1, c12);
         accelerationIntegrationHessian.unsafe_set(startIdxI + 2, startIdxI + 2, c22);
         accelerationIntegrationHessian.unsafe_set(startIdxI + 2, startIdxI + 3, c23);
         accelerationIntegrationHessian.unsafe_set(startIdxI + 3, startIdxI, c03);
         accelerationIntegrationHessian.unsafe_set(startIdxI + 3, startIdxI + 1, c13);
         accelerationIntegrationHessian.unsafe_set(startIdxI + 3, startIdxI + 2, c23);
         accelerationIntegrationHessian.unsafe_set(startIdxI + 3, startIdxI + 3, c33);

         accelerationIntegrationGradient.unsafe_set(startIdxI, 0, -g0);
         accelerationIntegrationGradient.unsafe_set(startIdxI + 1, 0, -g1);
         accelerationIntegrationGradient.unsafe_set(startIdxI + 2, 0, -g2);
         accelerationIntegrationGradient.unsafe_set(startIdxI + 3, 0, -g3);
      }

      MatrixTestTools.assertMatrixEquals(accelerationIntegrationHessian, calculatedHessian, 1e-5);
      MatrixTestTools.assertMatrixEquals(accelerationIntegrationGradient, calculatedGradient, 1e-5);

      SimpleEfficientActiveSetQPSolver solver = new SimpleEfficientActiveSetQPSolver();
      solver.setQuadraticCostFunction(accelerationIntegrationHessian, accelerationIntegrationGradient, 0.0);
      DMatrixRMaj solution = new DMatrixRMaj(numberOfBasisVectorsPerContactPoint * LinearMPCIndexHandler.coefficientsPerRho, 1);
      solver.solve(solution);

      contactPoint.computeContactForceCoefficientMatrix(solution, 0);

      for (double time = 0; time <= duration; time += 0.001)
      {
         contactPoint.computeContactForce(omega, time);

         double exponential = Math.min(Math.exp(omega * time), sufficientlyLargeValue);
         double a0 = omega2 * exponential;
         double a1 = omega2 / exponential;
         double a2 = 6.0 * time;
         double a3 = 2.0;

         for (int i = 0; i < numberOfBasisVectorsPerContactPoint; i++)
         {
            DMatrixRMaj basisCoefficients = contactPoint.getBasisCoefficients(i);

            double rhoValue = a0 * basisCoefficients.get(0, 0);
            rhoValue += a1 * basisCoefficients.get(0, 1);
            rhoValue += a2 * basisCoefficients.get(0, 2);
            rhoValue += a3 * basisCoefficients.get(0, 3);

            assertEquals(goalValueForBasis, rhoValue, 1e-3);
            assertEquals(goalValueForBasis, contactPoint.getBasisMagnitude(i).length(), 1e-3);
         }
      }
   }
}
