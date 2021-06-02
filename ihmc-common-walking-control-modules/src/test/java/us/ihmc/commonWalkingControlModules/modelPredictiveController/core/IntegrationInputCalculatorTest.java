package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPoint;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ZeroConeRotationCalculator;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.convexOptimization.quadraticProgram.SimpleEfficientActiveSetQPSolver;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.matrixlib.NativeCommonOps;

import java.util.Random;

import static us.ihmc.commonWalkingControlModules.modelPredictiveController.core.MPCQPInputCalculator.sufficientlyLargeValue;
import static us.ihmc.robotics.Assert.assertEquals;

public class IntegrationInputCalculatorTest
{
   private static final int iters = 500;

   @Test
   public void testComputeNormalAccelerationIntegration()
   {
      int numberOfBasisVectorsPerContactPoint = 2;
      MPCContactPlane contactPlane = new MPCContactPlane(1, numberOfBasisVectorsPerContactPoint, new ZeroConeRotationCalculator());
      double mu = 0.8;

      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(0.0, 0.0);
      polygon.update();
      FramePose3D pose = new FramePose3D();
      pose.getPosition().set(2.5, 1.5, 0.0);
      contactPlane.computeBasisVectors(polygon, pose, mu);

      double omega = 3.0;
      double duration = 0.7;
      double goalValueForPoint = 0.2;
      Vector3D goalForceForPoint = new Vector3D(0.0, 0.0, goalValueForPoint);

      DMatrixRMaj accelerationIntegrationHessian = new DMatrixRMaj(numberOfBasisVectorsPerContactPoint * LinearMPCIndexHandler.coefficientsPerRho,
                                                                   numberOfBasisVectorsPerContactPoint * LinearMPCIndexHandler.coefficientsPerRho);
      DMatrixRMaj accelerationIntegrationGradient = new DMatrixRMaj(numberOfBasisVectorsPerContactPoint * LinearMPCIndexHandler.coefficientsPerRho, 1);

      DMatrixRMaj calculatedHessian = new DMatrixRMaj(accelerationIntegrationHessian);
      DMatrixRMaj calculatedGradient = new DMatrixRMaj(accelerationIntegrationGradient);

      IntegrationInputCalculator.computeForceTrackingMatrix(0,
                                                            calculatedGradient,
                                                            calculatedHessian,
                                                            contactPlane,
                                                            duration,
                                                            omega,
                                                            goalValueForPoint);

      double tEnd = duration;
      double tStart = 0.0;

      double c00 = computeC00(omega, tEnd) - computeC00(omega, tStart);
      double c01 = computeC01(omega, tEnd) - computeC01(omega, tStart);
      double c02 = computeC02(omega, tEnd) - computeC02(omega, tStart);
      double c03 = computeC03(omega, tEnd) - computeC03(omega, tStart);
      double c11 = computeC11(omega, tEnd) - computeC11(omega, tStart);
      double c12 = computeC12(omega, tEnd) - computeC12(omega, tStart);
      double c13 = computeC13(omega, tEnd) - computeC13(omega, tStart);
      double c22 = computeC22(tEnd) - computeC22(tStart);
      double c23 = computeC23(tEnd) - computeC23(tStart);
      double c33 = computeC33(tEnd) - computeC33(tStart);

      double g0 = computeG0(omega, tEnd , 1.0) - computeG0(omega, tStart, 1.0);
      double g1 = computeG1(omega, tEnd , 1.0) - computeG1(omega, tStart, 1.0);
      double g2 = computeG2(tEnd , 1.0) - computeG2(tStart, 1.0);
      double g3 = computeG3(tEnd , 1.0) - computeG3(tStart, 1.0);

      for (int basisVectorIndexI = 0; basisVectorIndexI < numberOfBasisVectorsPerContactPoint; basisVectorIndexI++)
      {
         int startIdxI = basisVectorIndexI * LinearMPCIndexHandler.coefficientsPerRho;

         FrameVector3DReadOnly basisVectorI = contactPlane.getBasisVectorInPlaneFrame(basisVectorIndexI);

         accelerationIntegrationHessian.unsafe_set(startIdxI, startIdxI, c00);
         accelerationIntegrationHessian.unsafe_set(startIdxI, startIdxI + 1,  c01);
         accelerationIntegrationHessian.unsafe_set(startIdxI, startIdxI + 2,  c02);
         accelerationIntegrationHessian.unsafe_set(startIdxI, startIdxI + 3,  c03);
         accelerationIntegrationHessian.unsafe_set(startIdxI + 1, startIdxI,  c01);
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
            FrameVector3DReadOnly basisVectorJ = contactPlane.getBasisVectorInPlaneFrame(basisVectorIndexJ);

            double basisDot = basisVectorI.dot(basisVectorJ);

            int startIdxJ = basisVectorIndexJ * LinearMPCIndexHandler.coefficientsPerRho;

            accelerationIntegrationHessian.unsafe_set(startIdxI, startIdxJ, basisDot * c00);
            accelerationIntegrationHessian.unsafe_set(startIdxI, startIdxJ + 1, basisDot * c01);
            accelerationIntegrationHessian.unsafe_set(startIdxI, startIdxJ + 2, basisDot * c02);
            accelerationIntegrationHessian.unsafe_set(startIdxI, startIdxJ + 3, basisDot * c03);

            accelerationIntegrationHessian.unsafe_set(startIdxI + 1, startIdxJ, basisDot * c01);
            accelerationIntegrationHessian.unsafe_set(startIdxI + 1, startIdxJ + 1, basisDot * c11);
            accelerationIntegrationHessian.unsafe_set(startIdxI + 1, startIdxJ + 2, basisDot * c12);
            accelerationIntegrationHessian.unsafe_set(startIdxI + 1, startIdxJ + 3, basisDot * c13);

            accelerationIntegrationHessian.unsafe_set(startIdxI + 2, startIdxJ, basisDot * c02);
            accelerationIntegrationHessian.unsafe_set(startIdxI + 2, startIdxJ + 1, basisDot * c12);
            accelerationIntegrationHessian.unsafe_set(startIdxI + 2, startIdxJ + 2, basisDot * c22);
            accelerationIntegrationHessian.unsafe_set(startIdxI + 2, startIdxJ + 3, basisDot * c23);

            accelerationIntegrationHessian.unsafe_set(startIdxI + 3, startIdxJ, basisDot * c03);
            accelerationIntegrationHessian.unsafe_set(startIdxI + 3, startIdxJ + 1, basisDot * c13);
            accelerationIntegrationHessian.unsafe_set(startIdxI + 3, startIdxJ + 2, basisDot * c23);
            accelerationIntegrationHessian.unsafe_set(startIdxI + 3, startIdxJ + 3, basisDot * c33);

            // we know it's symmetric, and this way we can avoid iterating as much

            accelerationIntegrationHessian.unsafe_set(startIdxJ, startIdxI, basisDot * c00);
            accelerationIntegrationHessian.unsafe_set(startIdxJ, startIdxI + 1, basisDot * c01);
            accelerationIntegrationHessian.unsafe_set(startIdxJ, startIdxI + 2, basisDot * c02);
            accelerationIntegrationHessian.unsafe_set(startIdxJ, startIdxI + 3, basisDot * c03);
            accelerationIntegrationHessian.unsafe_set(startIdxJ + 1, startIdxI, basisDot * c01);
            accelerationIntegrationHessian.unsafe_set(startIdxJ + 1, startIdxI + 1, basisDot * c11);
            accelerationIntegrationHessian.unsafe_set(startIdxJ + 1, startIdxI + 2, basisDot * c12);
            accelerationIntegrationHessian.unsafe_set(startIdxJ + 1, startIdxI + 3, basisDot * c13);
            accelerationIntegrationHessian.unsafe_set(startIdxJ + 2, startIdxI, basisDot * c02);
            accelerationIntegrationHessian.unsafe_set(startIdxJ + 2, startIdxI + 1, basisDot * c12);
            accelerationIntegrationHessian.unsafe_set(startIdxJ + 2, startIdxI + 2, basisDot * c22);
            accelerationIntegrationHessian.unsafe_set(startIdxJ + 2, startIdxI + 3, basisDot * c23);
            accelerationIntegrationHessian.unsafe_set(startIdxJ + 3, startIdxI, basisDot * c03);
            accelerationIntegrationHessian.unsafe_set(startIdxJ + 3, startIdxI + 1, basisDot * c13);
            accelerationIntegrationHessian.unsafe_set(startIdxJ + 3, startIdxI + 2, basisDot * c23);
            accelerationIntegrationHessian.unsafe_set(startIdxJ + 3, startIdxI + 3, basisDot * c33);
         }

         double goal = basisVectorI.dot(goalForceForPoint);

         accelerationIntegrationGradient.unsafe_set(startIdxI, 0, g0 * goal);
         accelerationIntegrationGradient.unsafe_set(startIdxI + 1, 0, g1 * goal);
         accelerationIntegrationGradient.unsafe_set(startIdxI + 2, 0, g2 * goal);
         accelerationIntegrationGradient.unsafe_set(startIdxI + 3, 0, g3 * goal);
      }
      CommonOps_DDRM.scale(-1.0, accelerationIntegrationGradient);

      MatrixTestTools.assertMatrixEquals(accelerationIntegrationGradient, calculatedGradient, 1e-5);
      MatrixTestTools.assertMatrixEquals(accelerationIntegrationHessian, calculatedHessian, 1e-5);

      SimpleEfficientActiveSetQPSolver solver = new SimpleEfficientActiveSetQPSolver();
      solver.setQuadraticCostFunction(accelerationIntegrationHessian, accelerationIntegrationGradient, 0.0);
      DMatrixRMaj solution = new DMatrixRMaj(numberOfBasisVectorsPerContactPoint * LinearMPCIndexHandler.coefficientsPerRho, 1);
      solver.solve(solution);

      contactPlane.computeContactForceCoefficientMatrix(solution, 0);

      for (double time = 0; time <= duration; time += 0.05)
      {
         contactPlane.computeContactForce(omega, time);
         FrameVector3DReadOnly acceleration = contactPlane.getContactJerk();
         assertEquals(goalValueForPoint, acceleration.getZ(), 1e-3);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(goalForceForPoint, acceleration, 1e-3);
      }
   }

   @Test
   public void testComputeForceRateIntegration()
   {
      int numberOfBasisVectorsPerContactPoint = 2;
      MPCContactPlane contactPlane = new MPCContactPlane(1, numberOfBasisVectorsPerContactPoint, new ZeroConeRotationCalculator());
      double mu = 0.8;

      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(0.0, 0.0);
      polygon.update();
      FramePose3D pose = new FramePose3D();
      pose.getPosition().set(2.5, 1.5, 0.0);
      contactPlane.computeBasisVectors(polygon, pose, mu);

      double omega = 3.0;
      double duration = 0.7;
      double goalValueForPoint = 0.2;
      Vector3D goalForceForPoint = new Vector3D(0.0, 0.0, goalValueForPoint);

      DMatrixRMaj accelerationIntegrationHessian = new DMatrixRMaj(numberOfBasisVectorsPerContactPoint * LinearMPCIndexHandler.coefficientsPerRho,
                                                                   numberOfBasisVectorsPerContactPoint * LinearMPCIndexHandler.coefficientsPerRho);
      DMatrixRMaj accelerationIntegrationGradient = new DMatrixRMaj(numberOfBasisVectorsPerContactPoint * LinearMPCIndexHandler.coefficientsPerRho, 1);

      DMatrixRMaj calculatedHessian = new DMatrixRMaj(accelerationIntegrationHessian);
      DMatrixRMaj calculatedGradient = new DMatrixRMaj(accelerationIntegrationGradient);

      IntegrationInputCalculator.computeForceRateTrackingMatrix(0,
                                                            calculatedGradient,
                                                            calculatedHessian,
                                                            contactPlane,
                                                            duration,
                                                            omega,
                                                            goalValueForPoint);

      double tEnd = duration;
      double tStart = 0.0;

      double c00 = computeC00Rate(omega, tEnd) - computeC00Rate(omega, tStart);
      double c01 = computeC01Rate(omega, tEnd) - computeC01Rate(omega, tStart);
      double c02 = computeC02Rate(omega, tEnd) - computeC02Rate(omega, tStart);
      double c03 = 0.0;
      double c11 = computeC11Rate(omega, tEnd) - computeC11Rate(omega, tStart);
      double c12 = computeC12Rate(omega, tEnd) - computeC12Rate(omega, tStart);
      double c13 = 0.0;
      double c22 = computeC22Rate(tEnd) - computeC22Rate(tStart);
      double c23 = 0.0;
      double c33 = 0.0;

      double g0 = computeG0Rate(omega, tEnd , 1.0) - computeG0Rate(omega, tStart, 1.0);
      double g1 = computeG1Rate(omega, tEnd , 1.0) - computeG1Rate(omega, tStart, 1.0);
      double g2 = computeG2Rate(tEnd , 1.0) - computeG2Rate(tStart, 1.0);
      double g3 = 0.0;

      for (int basisVectorIndexI = 0; basisVectorIndexI < numberOfBasisVectorsPerContactPoint; basisVectorIndexI++)
      {
         int startIdxI = basisVectorIndexI * LinearMPCIndexHandler.coefficientsPerRho;

         FrameVector3DReadOnly basisVectorI = contactPlane.getBasisVectorInPlaneFrame(basisVectorIndexI);

         accelerationIntegrationHessian.unsafe_set(startIdxI, startIdxI, c00);
         accelerationIntegrationHessian.unsafe_set(startIdxI, startIdxI + 1,  c01);
         accelerationIntegrationHessian.unsafe_set(startIdxI, startIdxI + 2,  c02);
         accelerationIntegrationHessian.unsafe_set(startIdxI, startIdxI + 3,  c03);
         accelerationIntegrationHessian.unsafe_set(startIdxI + 1, startIdxI,  c01);
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
            FrameVector3DReadOnly basisVectorJ = contactPlane.getBasisVectorInPlaneFrame(basisVectorIndexJ);

            double basisDot = basisVectorI.dot(basisVectorJ);

            int startIdxJ = basisVectorIndexJ * LinearMPCIndexHandler.coefficientsPerRho;

            accelerationIntegrationHessian.unsafe_set(startIdxI, startIdxJ, basisDot * c00);
            accelerationIntegrationHessian.unsafe_set(startIdxI, startIdxJ + 1, basisDot * c01);
            accelerationIntegrationHessian.unsafe_set(startIdxI, startIdxJ + 2, basisDot * c02);
            accelerationIntegrationHessian.unsafe_set(startIdxI, startIdxJ + 3, basisDot * c03);

            accelerationIntegrationHessian.unsafe_set(startIdxI + 1, startIdxJ, basisDot * c01);
            accelerationIntegrationHessian.unsafe_set(startIdxI + 1, startIdxJ + 1, basisDot * c11);
            accelerationIntegrationHessian.unsafe_set(startIdxI + 1, startIdxJ + 2, basisDot * c12);
            accelerationIntegrationHessian.unsafe_set(startIdxI + 1, startIdxJ + 3, basisDot * c13);

            accelerationIntegrationHessian.unsafe_set(startIdxI + 2, startIdxJ, basisDot * c02);
            accelerationIntegrationHessian.unsafe_set(startIdxI + 2, startIdxJ + 1, basisDot * c12);
            accelerationIntegrationHessian.unsafe_set(startIdxI + 2, startIdxJ + 2, basisDot * c22);
            accelerationIntegrationHessian.unsafe_set(startIdxI + 2, startIdxJ + 3, basisDot * c23);

            accelerationIntegrationHessian.unsafe_set(startIdxI + 3, startIdxJ, basisDot * c03);
            accelerationIntegrationHessian.unsafe_set(startIdxI + 3, startIdxJ + 1, basisDot * c13);
            accelerationIntegrationHessian.unsafe_set(startIdxI + 3, startIdxJ + 2, basisDot * c23);
            accelerationIntegrationHessian.unsafe_set(startIdxI + 3, startIdxJ + 3, basisDot * c33);

            // we know it's symmetric, and this way we can avoid iterating as much

            accelerationIntegrationHessian.unsafe_set(startIdxJ, startIdxI, basisDot * c00);
            accelerationIntegrationHessian.unsafe_set(startIdxJ, startIdxI + 1, basisDot * c01);
            accelerationIntegrationHessian.unsafe_set(startIdxJ, startIdxI + 2, basisDot * c02);
            accelerationIntegrationHessian.unsafe_set(startIdxJ, startIdxI + 3, basisDot * c03);
            accelerationIntegrationHessian.unsafe_set(startIdxJ + 1, startIdxI, basisDot * c01);
            accelerationIntegrationHessian.unsafe_set(startIdxJ + 1, startIdxI + 1, basisDot * c11);
            accelerationIntegrationHessian.unsafe_set(startIdxJ + 1, startIdxI + 2, basisDot * c12);
            accelerationIntegrationHessian.unsafe_set(startIdxJ + 1, startIdxI + 3, basisDot * c13);
            accelerationIntegrationHessian.unsafe_set(startIdxJ + 2, startIdxI, basisDot * c02);
            accelerationIntegrationHessian.unsafe_set(startIdxJ + 2, startIdxI + 1, basisDot * c12);
            accelerationIntegrationHessian.unsafe_set(startIdxJ + 2, startIdxI + 2, basisDot * c22);
            accelerationIntegrationHessian.unsafe_set(startIdxJ + 2, startIdxI + 3, basisDot * c23);
            accelerationIntegrationHessian.unsafe_set(startIdxJ + 3, startIdxI, basisDot * c03);
            accelerationIntegrationHessian.unsafe_set(startIdxJ + 3, startIdxI + 1, basisDot * c13);
            accelerationIntegrationHessian.unsafe_set(startIdxJ + 3, startIdxI + 2, basisDot * c23);
            accelerationIntegrationHessian.unsafe_set(startIdxJ + 3, startIdxI + 3, basisDot * c33);
         }

         double goal = basisVectorI.dot(goalForceForPoint);

         accelerationIntegrationGradient.unsafe_set(startIdxI, 0, g0 * goal);
         accelerationIntegrationGradient.unsafe_set(startIdxI + 1, 0, g1 * goal);
         accelerationIntegrationGradient.unsafe_set(startIdxI + 2, 0, g2 * goal);
         accelerationIntegrationGradient.unsafe_set(startIdxI + 3, 0, g3 * goal);
      }
      CommonOps_DDRM.scale(-1.0, accelerationIntegrationGradient);


      MatrixTestTools.assertMatrixEquals(accelerationIntegrationHessian, calculatedHessian, 1e-5);
      MatrixTestTools.assertMatrixEquals(accelerationIntegrationGradient, calculatedGradient, 1e-5);

      for (int basisVectorIndexI = 0; basisVectorIndexI < numberOfBasisVectorsPerContactPoint; basisVectorIndexI++)
      {
         int startIdxI = basisVectorIndexI * LinearMPCIndexHandler.coefficientsPerRho;
         accelerationIntegrationHessian.unsafe_set(startIdxI + 3, startIdxI + 3, 1.0);
      }

      SimpleEfficientActiveSetQPSolver solver = new SimpleEfficientActiveSetQPSolver();
      solver.setQuadraticCostFunction(accelerationIntegrationHessian, accelerationIntegrationGradient, 0.0);
      DMatrixRMaj solution = new DMatrixRMaj(numberOfBasisVectorsPerContactPoint * LinearMPCIndexHandler.coefficientsPerRho, 1);
      solver.solve(solution);

      contactPlane.computeContactForceCoefficientMatrix(solution, 0);

      for (double time = 0; time <= duration; time += 0.05)
      {
         contactPlane.computeContactForce(omega, time);
         FrameVector3DReadOnly jerk = contactPlane.getContactJerk();
         assertEquals(goalValueForPoint, jerk.getZ(), 1e-3);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(goalForceForPoint, jerk, 1e-3);
      }
   }

   @Test
   public void testComputeRhoAccelerationIntegration()
   {
      int numberOfBasisVectorsPerContactPoint = 4;
      Point2D point = new Point2D(2.5, 1.5);
      MPCContactPoint contactPoint = new MPCContactPoint(numberOfBasisVectorsPerContactPoint);
      double mu = 0.8;

      SimpleEfficientActiveSetQPSolver solver = new SimpleEfficientActiveSetQPSolver();
      DMatrixRMaj solution = new DMatrixRMaj(numberOfBasisVectorsPerContactPoint * LinearMPCIndexHandler.coefficientsPerRho, 1);

      double omega = 3.0;

      double goalValueForBasis = 0.2;
      double duration = 0.7;

      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         FramePose3D pose = EuclidFrameRandomTools.nextFramePose3D(random, ReferenceFrame.getWorldFrame());

         contactPoint.computeBasisVectors(point, pose, 0.0, mu);

         DMatrixRMaj accelerationIntegrationHessian = new DMatrixRMaj(numberOfBasisVectorsPerContactPoint * LinearMPCIndexHandler.coefficientsPerRho,
                                                                      numberOfBasisVectorsPerContactPoint * LinearMPCIndexHandler.coefficientsPerRho);
         DMatrixRMaj accelerationIntegrationGradient = new DMatrixRMaj(numberOfBasisVectorsPerContactPoint * LinearMPCIndexHandler.coefficientsPerRho, 1);

         DMatrixRMaj calculatedHessian = new DMatrixRMaj(accelerationIntegrationHessian);
         DMatrixRMaj calculatedGradient = new DMatrixRMaj(accelerationIntegrationGradient);

         IntegrationInputCalculator.computeRhoAccelerationTrackingMatrix(0,
                                                                         calculatedGradient,
                                                                         calculatedHessian,
                                                                         contactPoint,
                                                                         duration,
                                                                         omega,
                                                                         goalValueForBasis);

         double tEnd = duration;
         double tStart = 0.0;

         double c00 = computeC00(omega, tEnd) - computeC00(omega, tStart);
         double c01 = computeC01(omega, tEnd) - computeC01(omega, tStart);
         double c02 = computeC02(omega, tEnd) - computeC02(omega, tStart);
         double c03 = computeC03(omega, tEnd) - computeC03(omega, tStart);
         double c11 = computeC11(omega, tEnd) - computeC11(omega, tStart);
         double c12 = computeC12(omega, tEnd) - computeC12(omega, tStart);
         double c13 = computeC13(omega, tEnd) - computeC13(omega, tStart);
         double c22 = computeC22(tEnd) - computeC22(tStart);
         double c23 = computeC23(tEnd) - computeC23(tStart);
         double c33 = computeC33(tEnd) - computeC33(tStart);

         double g0 = computeG0(omega, tEnd , goalValueForBasis) - computeG0(omega, tStart, goalValueForBasis);
         double g1 = computeG1(omega, tEnd , goalValueForBasis) - computeG1(omega, tStart, goalValueForBasis);
         double g2 = computeG2(tEnd , goalValueForBasis) - computeG2(tStart, goalValueForBasis);
         double g3 = computeG3(tEnd , goalValueForBasis) - computeG3(tStart, goalValueForBasis);

         for (int basisVectorIndexI = 0; basisVectorIndexI < numberOfBasisVectorsPerContactPoint; basisVectorIndexI++)
         {
            int startIdxI = basisVectorIndexI * LinearMPCIndexHandler.coefficientsPerRho;

            accelerationIntegrationHessian.unsafe_set(startIdxI, startIdxI, c00);
            accelerationIntegrationHessian.unsafe_set(startIdxI, startIdxI + 1, c01);
            accelerationIntegrationHessian.unsafe_set(startIdxI, startIdxI + 2, c02);
            accelerationIntegrationHessian.unsafe_set(startIdxI, startIdxI + 3, c03);
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

         MatrixTestTools.assertMatrixEquals("Hessian is incorrect", accelerationIntegrationHessian, calculatedHessian, 1e-5);
         MatrixTestTools.assertMatrixEquals("Gradient is incorrect", accelerationIntegrationGradient, calculatedGradient, 1e-5);

         solver.clear();
         solver.setQuadraticCostFunction(calculatedHessian, calculatedGradient, 0.0);
         solver.solve(solution);

         contactPoint.computeContactForceCoefficientMatrix(solution, 0);

         for (double time = 0; time <= duration; time += 0.05)
         {
            contactPoint.computeContactForce(omega, time);

            double exponential = Math.min(Math.exp(omega * time), sufficientlyLargeValue);
            double a0 = omega * omega * exponential;
            double a1 = omega * omega / exponential;
            double a2 = 6.0 * time;
            double a3 = 2.0;

            for (int i = 0; i < numberOfBasisVectorsPerContactPoint; i++)
            {
               DMatrixRMaj basisCoefficients = contactPoint.getBasisCoefficients(i);

               double rhoValue = a0 * basisCoefficients.get(0, 0);
               rhoValue += a1 * basisCoefficients.get(0, 1);
               rhoValue += a2 * basisCoefficients.get(0, 2);
               rhoValue += a3 * basisCoefficients.get(0, 3);

               assertEquals("Rho value at " + time + " is incorrect", goalValueForBasis, rhoValue, 1e-3);
               assertEquals("Rho value at " + time + " is incorrect", goalValueForBasis, contactPoint.getBasisMagnitude(i).length(), 1e-3);
            }
         }

         goalValueForBasis = RandomNumbers.nextDouble(random, 0.0, 5.0);
         duration = RandomNumbers.nextDouble(random, 0.1, 1.5);
      }
   }

   @Test
   public void testComputeRhoAccelerationIntegrationSingleRho()
   {
      SimpleEfficientActiveSetQPSolver solver = new SimpleEfficientActiveSetQPSolver();
      DMatrixRMaj solution = new DMatrixRMaj(LinearMPCIndexHandler.coefficientsPerRho, 1);

      double omega = 3.0;

      double goalValueForBasis = 0.2;
      double duration = 0.7;

      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         DMatrixRMaj accelerationIntegrationHessian = new DMatrixRMaj(LinearMPCIndexHandler.coefficientsPerRho, LinearMPCIndexHandler.coefficientsPerRho);
         DMatrixRMaj accelerationIntegrationGradient = new DMatrixRMaj(LinearMPCIndexHandler.coefficientsPerRho, 1);

         double tEnd = duration;
         double tStart = 0.0;

         double c00 = computeC00(omega, tEnd) - computeC00(omega, tStart);
         double c01 = computeC01(omega, tEnd) - computeC01(omega, tStart);
         double c02 = computeC02(omega, tEnd) - computeC02(omega, tStart);
         double c03 = computeC03(omega, tEnd) - computeC03(omega, tStart);
         double c11 = computeC11(omega, tEnd) - computeC11(omega, tStart);
         double c12 = computeC12(omega, tEnd) - computeC12(omega, tStart);
         double c13 = computeC13(omega, tEnd) - computeC13(omega, tStart);
         double c22 = computeC22(tEnd) - computeC22(tStart);
         double c23 = computeC23(tEnd) - computeC23(tStart);
         double c33 = computeC33(tEnd) - computeC33(tStart);

         double g0 = computeG0(omega, tEnd , goalValueForBasis) - computeG0(omega, tStart, goalValueForBasis);
         double g1 = computeG1(omega, tEnd , goalValueForBasis) - computeG1(omega, tStart, goalValueForBasis);
         double g2 = computeG2(tEnd , goalValueForBasis) - computeG2(tStart, goalValueForBasis);
         double g3 = computeG3(tEnd , goalValueForBasis) - computeG3(tStart, goalValueForBasis);

         accelerationIntegrationHessian.unsafe_set(0, 0, c00);
         accelerationIntegrationHessian.unsafe_set(0, 1, c01);
         accelerationIntegrationHessian.unsafe_set(0, 2, c02);
         accelerationIntegrationHessian.unsafe_set(0, 3, c03);
         accelerationIntegrationHessian.unsafe_set(1, 0, c01);
         accelerationIntegrationHessian.unsafe_set(1, 1, c11);
         accelerationIntegrationHessian.unsafe_set(1, 2, c12);
         accelerationIntegrationHessian.unsafe_set(1, 3, c13);
         accelerationIntegrationHessian.unsafe_set(2, 0, c02);
         accelerationIntegrationHessian.unsafe_set(2, 1, c12);
         accelerationIntegrationHessian.unsafe_set(2, 2, c22);
         accelerationIntegrationHessian.unsafe_set(2, 3, c23);
         accelerationIntegrationHessian.unsafe_set(3, 0, c03);
         accelerationIntegrationHessian.unsafe_set(3, 1, c13);
         accelerationIntegrationHessian.unsafe_set(3, 2, c23);
         accelerationIntegrationHessian.unsafe_set(3, 3, c33);

         accelerationIntegrationGradient.unsafe_set(0, 0, -g0);
         accelerationIntegrationGradient.unsafe_set(1, 0, -g1);
         accelerationIntegrationGradient.unsafe_set(2, 0, -g2);
         accelerationIntegrationGradient.unsafe_set(3, 0, -g3);

         double weight = 1e1;
         CommonOps_DDRM.scale(weight, accelerationIntegrationHessian);
         CommonOps_DDRM.scale(weight, accelerationIntegrationGradient);
         double expectedResidual = weight * 0.5 * duration * goalValueForBasis * goalValueForBasis;

         solver.clear();
         solver.setQuadraticCostFunction(accelerationIntegrationHessian, accelerationIntegrationGradient, expectedResidual);
         solver.solve(solution);
         double outputCost = solver.getObjectiveCost(solution);

         DMatrixRMaj cost = new DMatrixRMaj(1, 1);
         NativeCommonOps.multQuad(solution, accelerationIntegrationHessian, cost);
         CommonOps_DDRM.scale(0.5, cost);
         CommonOps_DDRM.multAddTransA(accelerationIntegrationGradient, solution, cost);

         double runningCost = 0.0;
         double residualCost = 0.0;
         double runningCostB = 0.0;
         double dt = 1e-4;
         for (double time = 0; time <= duration; time += dt)
         {
            double exponential = Math.exp(omega * time);
            double a0 = omega * omega * exponential;
            double a1 = omega * omega / exponential;
            double a2 = 6.0 * time;
            double a3 = 2.0;

            double rhoValue = a0 * solution.get(0, 0);
            rhoValue += a1 * solution.get(1, 0);
            rhoValue += a2 * solution.get(2, 0);
            rhoValue += a3 * solution.get(3, 0);

            residualCost += goalValueForBasis * goalValueForBasis * 0.5 * dt * weight;

            runningCost += 0.5 * (rhoValue * rhoValue - 2.0 * goalValueForBasis * rhoValue) * dt * weight;
            runningCostB += 0.5 * MathTools.pow(rhoValue - goalValueForBasis, 2) * dt * weight;
         }
         assertEquals(expectedResidual, residualCost, weight * 1e-2);
         assertEquals(runningCostB, runningCost + residualCost, weight * 1e-3);
         assertEquals(runningCost, cost.get(0, 0), weight * 1e-2);
         assertEquals(outputCost, cost.get(0, 0) + expectedResidual, weight * 1e-3);

         for (double time = 0; time <= duration; time += 0.05)
         {

            double exponential = Math.exp(omega * time);
            double a0 = omega * omega * exponential;
            double a1 = omega * omega / exponential;
            double a2 = 6.0 * time;
            double a3 = 2.0;

            double rhoValue = a0 * solution.get(0, 0);
            rhoValue += a1 * solution.get(1, 0);
            rhoValue += a2 * solution.get(2, 0);
            rhoValue += a3 * solution.get(3, 0);

            assertEquals("Rho value at " + time + " is incorrect", goalValueForBasis, rhoValue, 1e-3);
         }

         goalValueForBasis = RandomNumbers.nextDouble(random, 0.0, 5.0);
         duration = RandomNumbers.nextDouble(random, 0.1, 1.5);
      }
   }

   private static double computeC00(double omega, double time)
   {
      return MathTools.pow(omega, 3) / 2.0 * Math.exp(2.0 * omega * time);
   }

   private static double computeC01(double omega, double time)
   {
      return MathTools.pow(omega, 4) * time;
   }

   private static double computeC02(double omega, double time)
   {
      return 6.0 * Math.exp(omega * time) * (omega * time - 1.0);
   }

   private static double computeC03(double omega, double time)
   {
      return 2.0 * omega * Math.exp(omega * time);
   }

   private static double computeC11(double omega, double time)
   {
      return -MathTools.pow(omega, 3) / 2.0 * Math.exp(-2.0 * time * omega);
   }

   private static double computeC12(double omega, double time)
   {
      return -6.0 * Math.exp(-omega * time) * (omega * time + 1.0);
   }

   private static double computeC13(double omega, double time)
   {
      return -2.0 * omega * Math.exp(-omega * time);
   }

   private static double computeC22(double time)
   {
      return 12.0 * MathTools.pow(time, 3);
   }

   private static double computeC23(double time)
   {
      return 6.0 * time * time;
   }

   private static double computeC33(double time)
   {
      return 4.0 * time;
   }

   private static double computeG0(double omega, double time, double goal)
   {
      return omega * goal * Math.exp(omega * time);
   }

   private static double computeG1(double omega, double time, double goal)
   {
      return -omega * goal * Math.exp(-omega * time);
   }

   private static double computeG2(double time, double goal)
   {
      return 3.0 * goal * time * time;
   }

   private static double computeG3(double time, double goal)
   {
      return 2.0 * goal * time;
   }

   private static double computeC00Rate(double omega, double time)
   {
      return MathTools.pow(omega, 8) / 2.0 * Math.exp(2.0 * omega * time);
   }

   private static double computeC01Rate(double omega, double time)
   {
      return -MathTools.pow(omega, 9) * time;
   }

   private static double computeC02Rate(double omega, double time)
   {
      return 6.0 * omega * omega * Math.exp(omega * time);
   }

   private static double computeC11Rate(double omega, double time)
   {
      return -MathTools.pow(omega, 8) / 2.0 * Math.exp(-2.0 * time * omega);
   }

   private static double computeC12Rate(double omega, double time)
   {
      return 6.0 * omega * omega * Math.exp(-omega * time);
   }

   private static double computeC22Rate(double time)
   {
      return 36.0 * time;
   }

   private static double computeG0Rate(double omega, double time, double goal)
   {
      return omega * omega * goal * Math.exp(omega * time);
   }

   private static double computeG1Rate(double omega, double time, double goal)
   {
      return omega * omega * goal * Math.exp(-omega * time);
   }

   private static double computeG2Rate(double time, double goal)
   {
      return 6.0 * goal * time;
   }
}
