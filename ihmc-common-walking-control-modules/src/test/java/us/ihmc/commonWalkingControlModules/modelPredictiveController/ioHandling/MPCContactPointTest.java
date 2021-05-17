package us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.LinearMPCIndexHandler;
import us.ihmc.convexOptimization.quadraticProgram.SimpleEfficientActiveSetQPSolver;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.robotics.MatrixMissingTools;

import static us.ihmc.commonWalkingControlModules.modelPredictiveController.core.MPCQPInputCalculator.sufficientlyLargeValue;
import static us.ihmc.robotics.Assert.assertEquals;

public class MPCContactPointTest
{
   @Test
   public void testComputeAccelerationIntegration()
   {
      int numberOfBasisVectorsPerContactPoint = 4;
      Point2D point = new Point2D(2.5, 1.5);
      MPCContactPoint contactPoint = new MPCContactPoint(numberOfBasisVectorsPerContactPoint);
      double mu = 0.8;

      contactPoint.computeBasisVectors(point, new FramePose3D(), 0.0, mu);

      double omega = 3.0;
      double duration = 0.7;
      double goalValueForPoint = 0.2;
      double goalValueForBasis = goalValueForPoint / numberOfBasisVectorsPerContactPoint;

      DMatrixRMaj accelerationIntegrationHessian = new DMatrixRMaj(numberOfBasisVectorsPerContactPoint * LinearMPCIndexHandler.coefficientsPerRho,
                                                                   numberOfBasisVectorsPerContactPoint * LinearMPCIndexHandler.coefficientsPerRho);
      DMatrixRMaj accelerationIntegrationGradient = new DMatrixRMaj(numberOfBasisVectorsPerContactPoint * LinearMPCIndexHandler.coefficientsPerRho, 1);

      DMatrixRMaj calculatedHessian = new DMatrixRMaj(accelerationIntegrationHessian);
      DMatrixRMaj calculatedGradient = new DMatrixRMaj(accelerationIntegrationGradient);

      contactPoint.computeAccelerationIntegrationMatrix(0, calculatedGradient, calculatedHessian, duration, omega, goalValueForPoint);


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

      double g0 = omega * (positiveExponential - 1.0);
      double g1 = -omega * (negativeExponential - 1.0);
      double g2 = 3.0 * duration2;
      double g3 = 2.0 * duration;

      for (int basisVectorIndexI = 0; basisVectorIndexI < numberOfBasisVectorsPerContactPoint; basisVectorIndexI++)
      {
         int startIdxI = basisVectorIndexI * LinearMPCIndexHandler.coefficientsPerRho;

         FrameVector3DReadOnly basisVectorI = contactPoint.getBasisVector(basisVectorIndexI);

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

         for (int basisVectorIndexJ = basisVectorIndexI + 1; basisVectorIndexJ < numberOfBasisVectorsPerContactPoint; basisVectorIndexJ++)
         {
            FrameVector3DReadOnly basisVectorJ = contactPoint.getBasisVector(basisVectorIndexJ);

            double basisDot = basisVectorI.dot(basisVectorJ);

            int startIdxJ = basisVectorIndexJ * LinearMPCIndexHandler.coefficientsPerRho;

            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxI, startIdxJ, basisDot * c00);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxI, startIdxJ + 1, basisDot * c01);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxI, startIdxJ + 2, basisDot * c02);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxI, startIdxJ + 3, basisDot * c03);

            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxI + 1, startIdxJ, basisDot * c01);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxI + 1, startIdxJ + 1, basisDot * c11);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxI + 1, startIdxJ + 2, basisDot * c12);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxI + 1, startIdxJ + 3, basisDot * c13);

            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxI + 2, startIdxJ, basisDot * c02);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxI + 2, startIdxJ + 1, basisDot * c12);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxI + 2, startIdxJ + 2, basisDot * c22);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxI + 2, startIdxJ + 3, basisDot * c23);

            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxI + 3, startIdxJ, basisDot * c03);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxI + 3, startIdxJ + 1, basisDot * c13);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxI + 3, startIdxJ + 2, basisDot * c23);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxI + 3, startIdxJ + 3, basisDot * c33);

            // we know it's symmetric, and this way we can avoid iterating as much

            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxJ, startIdxI, basisDot * c00);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxJ, startIdxI + 1, basisDot * c01);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxJ, startIdxI + 2, basisDot * c02);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxJ, startIdxI + 3, basisDot * c03);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxJ + 1, startIdxI, basisDot * c01);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxJ + 1, startIdxI + 1, basisDot * c11);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxJ + 1, startIdxI + 2, basisDot * c12);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxJ + 1, startIdxI + 3, basisDot * c13);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxJ + 2, startIdxI, basisDot * c02);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxJ + 2, startIdxI + 1, basisDot * c12);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxJ + 2, startIdxI + 2, basisDot * c22);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxJ + 2, startIdxI + 3, basisDot * c23);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxJ + 3, startIdxI, basisDot * c03);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxJ + 3, startIdxI + 1, basisDot * c13);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxJ + 3, startIdxI + 2, basisDot * c23);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxJ + 3, startIdxI + 3, basisDot * c33);
         }

         double goal = basisVectorI.getZ() * goalValueForPoint;

         accelerationIntegrationGradient.unsafe_set(startIdxI, 0, g0 * goal);
         accelerationIntegrationGradient.unsafe_set(startIdxI + 1, 0, g1 * goal);
         accelerationIntegrationGradient.unsafe_set(startIdxI + 2, 0, g2 * goal);
         accelerationIntegrationGradient.unsafe_set(startIdxI + 3, 0, g3 * goal);
      }

//      MatrixTestTools.assertMatrixEquals(accelerationIntegrationGradient, calculatedGradient, 1e-5);
//      MatrixTestTools.assertMatrixEquals(accelerationIntegrationHessian, calculatedHessian, 1e-5);

      CommonOps_DDRM.scale(-1.0, accelerationIntegrationGradient);

      SimpleEfficientActiveSetQPSolver solver = new SimpleEfficientActiveSetQPSolver();
      solver.setQuadraticCostFunction(accelerationIntegrationHessian, accelerationIntegrationGradient, 0.0);
      DMatrixRMaj solution = new DMatrixRMaj(numberOfBasisVectorsPerContactPoint * LinearMPCIndexHandler.coefficientsPerRho, 1);
      solver.solve(solution);

      contactPoint.computeContactForceCoefficientMatrix(solution, 0);

      for (double time = 0; time <= duration; time += 0.001)
      {
         contactPoint.computeContactForce(omega, time);
         FrameVector3DReadOnly acceleration = contactPoint.getContactAcceleration();
         assertEquals(goalValueForPoint, acceleration.getZ(), 1e-3);
         assertEquals(goalValueForPoint, acceleration.length(), 1e-3);
      }
   }
}
