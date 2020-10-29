package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.matrixlib.MatrixTools;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

public class CoMTrajectoryPlannerToolsTest
{
   private static final double epsilon = 1e-4;

   @Test
   public void testTrajectoryConstruction()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < 1000; iter++)
      {
         double omega = RandomNumbers.nextDouble(random, 0.3, 2.0);
         double time = RandomNumbers.nextDouble(random, 0.0, 1.5);

         FramePoint3D c0 = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D c1 = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D c2 = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D c3 = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D c4 = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D c5 = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());

         FramePoint3D desiredCoMPosition = new FramePoint3D();
         FramePoint3D desiredDCMPosition = new FramePoint3D();
         FramePoint3D desiredDCMPosition2 = new FramePoint3D();
         FramePoint3D desiredVRPPosition = new FramePoint3D();
         FramePoint3D desiredVRPPosition2 = new FramePoint3D();
         FrameVector3D desiredCoMVelocity = new FrameVector3D();
         FrameVector3D desiredCoMAcceleration = new FrameVector3D();
         FrameVector3D desiredDCMVelocity = new FrameVector3D();

         CoMTrajectoryPlannerTools.constructDesiredCoMPosition(desiredCoMPosition, c0, c1, c2, c3, c4, c5, time, omega);
         CoMTrajectoryPlannerTools.constructDesiredCoMVelocity(desiredCoMVelocity, c0, c1, c2, c3, c4, c5, time, omega);
         CoMTrajectoryPlannerTools.constructDesiredCoMAcceleration(desiredCoMAcceleration, c0, c1, c2, c3, c4, c5, time, omega);
         CoMTrajectoryPlannerTools.constructDesiredVRPPosition(desiredVRPPosition2, c0, c1, c2, c3, c4, c5, time, omega);
         CoMTrajectoryPlannerTools.constructDesiredDCMPosition(desiredDCMPosition, c0, c1, c2, c3, c4, c5, time, omega);

         FramePoint3D desiredCoMPositionExpected = new FramePoint3D();
         FrameVector3D desiredCoMVelocityExpected = new FrameVector3D();
         FrameVector3D desiredCoMAccelerationExpected = new FrameVector3D();
         FramePoint3D desiredDCMPositionExpected = new FramePoint3D();
         FramePoint3D desiredDCMPositionExpected2 = new FramePoint3D();
         FrameVector3D desiredDCMVelocityExpected = new FrameVector3D();
         FramePoint3D desiredVRPPositionExpected = new FramePoint3D();
         FramePoint3D desiredVRPPositionExpected2 = new FramePoint3D();

         FramePoint3D temp = new FramePoint3D();

         // com position
         temp.set(c0);
         temp.scale(CoMTrajectoryPlannerTools.getCoMPositionFirstCoefficientTimeFunction(omega, time));

         desiredCoMPositionExpected.add(temp);

         temp.set(c1);
         temp.scale(CoMTrajectoryPlannerTools.getCoMPositionSecondCoefficientTimeFunction(omega, time));

         desiredCoMPositionExpected.add(temp);

         temp.set(c2);
         temp.scale(CoMTrajectoryPlannerTools.getCoMPositionThirdCoefficientTimeFunction(time));

         desiredCoMPositionExpected.add(temp);

         temp.set(c3);
         temp.scale(CoMTrajectoryPlannerTools.getCoMPositionFourthCoefficientTimeFunction(time));

         desiredCoMPositionExpected.add(temp);

         temp.set(c4);
         temp.scale(CoMTrajectoryPlannerTools.getCoMPositionFifthCoefficientTimeFunction(time));

         desiredCoMPositionExpected.add(temp);

         temp.set(c5);
         temp.scale(CoMTrajectoryPlannerTools.getCoMPositionSixthCoefficientTimeFunction());

         desiredCoMPositionExpected.add(temp);

         // com velocity
         temp.set(c0);
         temp.scale(CoMTrajectoryPlannerTools.getCoMVelocityFirstCoefficientTimeFunction(omega, time));

         desiredCoMVelocityExpected.add(temp);

         temp.set(c1);
         temp.scale(CoMTrajectoryPlannerTools.getCoMVelocitySecondCoefficientTimeFunction(omega, time));

         desiredCoMVelocityExpected.add(temp);

         temp.set(c2);
         temp.scale(CoMTrajectoryPlannerTools.getCoMVelocityThirdCoefficientTimeFunction(time));

         desiredCoMVelocityExpected.add(temp);

         temp.set(c3);
         temp.scale(CoMTrajectoryPlannerTools.getCoMVelocityFourthCoefficientTimeFunction(time));

         desiredCoMVelocityExpected.add(temp);

         temp.set(c4);
         temp.scale(CoMTrajectoryPlannerTools.getCoMVelocityFifthCoefficientTimeFunction());

         desiredCoMVelocityExpected.add(temp);

         temp.set(c5);
         temp.scale(CoMTrajectoryPlannerTools.getCoMVelocitySixthCoefficientTimeFunction());

         desiredCoMVelocityExpected.add(temp);

         // com acceleration
         temp.set(c0);
         temp.scale(CoMTrajectoryPlannerTools.getCoMAccelerationFirstCoefficientTimeFunction(omega, time));

         desiredCoMAccelerationExpected.add(temp);

         temp.set(c1);
         temp.scale(CoMTrajectoryPlannerTools.getCoMAccelerationSecondCoefficientTimeFunction(omega, time));

         desiredCoMAccelerationExpected.add(temp);

         temp.set(c2);
         temp.scale(CoMTrajectoryPlannerTools.getCoMAccelerationThirdCoefficientTimeFunction(time));

         desiredCoMAccelerationExpected.add(temp);

         temp.set(c3);
         temp.scale(CoMTrajectoryPlannerTools.getCoMAccelerationFourthCoefficientTimeFunction());

         desiredCoMAccelerationExpected.add(temp);

         temp.set(c4);
         temp.scale(CoMTrajectoryPlannerTools.getCoMAccelerationFifthCoefficientTimeFunction());

         desiredCoMAccelerationExpected.add(temp);

         temp.set(c5);
         temp.scale(CoMTrajectoryPlannerTools.getCoMAccelerationSixthCoefficientTimeFunction());

         desiredCoMAccelerationExpected.add(temp);

         // dcm position
         temp.set(c0);
         temp.scale(CoMTrajectoryPlannerTools.getDCMPositionFirstCoefficientTimeFunction(omega, time));

         desiredDCMPositionExpected.add(temp);

         temp.set(c1);
         temp.scale(CoMTrajectoryPlannerTools.getDCMPositionSecondCoefficientTimeFunction());

         desiredDCMPositionExpected.add(temp);

         temp.set(c2);
         temp.scale(CoMTrajectoryPlannerTools.getDCMPositionThirdCoefficientTimeFunction(omega, time));

         desiredDCMPositionExpected.add(temp);

         temp.set(c3);
         temp.scale(CoMTrajectoryPlannerTools.getDCMPositionFourthCoefficientTimeFunction(omega, time));

         desiredDCMPositionExpected.add(temp);

         temp.set(c4);
         temp.scale(CoMTrajectoryPlannerTools.getDCMPositionFifthCoefficientTimeFunction(omega, time));

         desiredDCMPositionExpected.add(temp);

         temp.set(c5);
         temp.scale(CoMTrajectoryPlannerTools.getDCMPositionSixthCoefficientTimeFunction());

         desiredDCMPositionExpected.add(temp);

         // vrp position
         temp.set(c0);
         temp.scale(CoMTrajectoryPlannerTools.getVRPPositionFirstCoefficientTimeFunction());

         desiredVRPPositionExpected.add(temp);

         temp.set(c1);
         temp.scale(CoMTrajectoryPlannerTools.getVRPPositionSecondCoefficientTimeFunction());

         desiredVRPPositionExpected.add(temp);

         temp.set(c2);
         temp.scale(CoMTrajectoryPlannerTools.getVRPPositionThirdCoefficientTimeFunction(omega, time));

         desiredVRPPositionExpected.add(temp);

         temp.set(c3);
         temp.scale(CoMTrajectoryPlannerTools.getVRPPositionFourthCoefficientTimeFunction(omega, time));

         desiredVRPPositionExpected.add(temp);

         temp.set(c4);
         temp.scale(CoMTrajectoryPlannerTools.getVRPPositionFifthCoefficientTimeFunction(time));

         desiredVRPPositionExpected.add(temp);

         temp.set(c5);
         temp.scale(CoMTrajectoryPlannerTools.getVRPPositionSixthCoefficientTimeFunction());

         desiredVRPPositionExpected.add(temp);

         CapturePointTools.computeCapturePointPosition(desiredCoMPositionExpected, desiredCoMVelocityExpected, omega, desiredDCMPositionExpected2);
         CapturePointTools.computeCapturePointPosition(desiredCoMPosition, desiredCoMVelocity, omega, desiredDCMPosition2);
         CapturePointTools.computeCapturePointVelocity(desiredCoMVelocity, desiredCoMAcceleration, omega, desiredDCMVelocity);
         CapturePointTools.computeCapturePointVelocity(desiredCoMVelocityExpected, desiredCoMAccelerationExpected, omega, desiredDCMVelocityExpected);
         CapturePointTools.computeCentroidalMomentumPivot(desiredDCMPosition, desiredDCMVelocity, omega, desiredVRPPosition);
         CapturePointTools.computeCentroidalMomentumPivot(desiredDCMPositionExpected, desiredDCMVelocityExpected, omega, desiredVRPPositionExpected2);

         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredCoMPositionExpected, desiredCoMPosition, epsilon);
         EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(desiredCoMVelocityExpected, desiredCoMVelocity, epsilon);
         EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(desiredCoMAccelerationExpected, desiredCoMAcceleration, epsilon);
         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCMPositionExpected, desiredDCMPosition, epsilon);
         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCMPositionExpected, desiredDCMPosition2, epsilon);
         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCMPositionExpected2, desiredDCMPosition, epsilon);
         EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(desiredDCMVelocityExpected, desiredDCMVelocity, epsilon);
         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredVRPPositionExpected2, desiredVRPPosition, epsilon);
         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredVRPPositionExpected2, desiredVRPPositionExpected, epsilon);
         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredVRPPositionExpected2, desiredVRPPosition2, epsilon);
      }
   }

   @Test
   public void testCoMPositionContinuityObjective()
   {
      double omega = 3.0;
      double previousDuration = 1.5;

      DMatrixRMaj jacobian = new DMatrixRMaj(12, 12);
      DMatrixRMaj hessian1 = new DMatrixRMaj(12, 12);
      DMatrixRMaj hessian2 = new DMatrixRMaj(12, 12);
      DMatrixRMaj hessian3 = new DMatrixRMaj(12, 12);

      double weight = 10.0;
      double weightSqrt = Math.sqrt(weight);

      CoMTrajectoryPlannerTools.addCoMPositionContinuityObjectiveDirectly(weight, 0, 1, omega, previousDuration, hessian3);

      CoMTrajectoryPlannerTools.addCoMPositionContinuityObjective(weightSqrt, 0, 1, 0, omega, previousDuration, jacobian);
      CoMTrajectoryPlannerTools.addCoMPositionContinuityObjective(weight, 0, 1, omega, previousDuration, hessian2);
      CommonOps_DDRM.multInner(jacobian, hessian1);

      EjmlUnitTests.assertEquals(hessian1, hessian2, 1e-5);
      EjmlUnitTests.assertEquals(hessian1, hessian3, 1e-5);
   }


   @Test
   public void testCoMPositionContinuityObjective2()
   {
      double omega = 3.0;
      double previousDuration = 1.5;

      DMatrixRMaj hessian = new DMatrixRMaj(12, 12);
      DMatrixRMaj xGradient = new DMatrixRMaj(12, 1);
      DMatrixRMaj yGradient = new DMatrixRMaj(12, 1);
      DMatrixRMaj zGradient = new DMatrixRMaj(12, 1);
      DMatrixRMaj xSolution = new DMatrixRMaj(12, 1);
      DMatrixRMaj ySolution = new DMatrixRMaj(12, 1);
      DMatrixRMaj zSolution = new DMatrixRMaj(12, 1);

      FramePoint3D desiredPosition = new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 1.5, 2.0);

      double weight = 10.0;

      CoMTrajectoryPlannerTools.addCoMPositionContinuityObjectiveDirectly(weight, 0, 1, omega, previousDuration, hessian);
      CoMTrajectoryPlannerTools.addCoMPositionObjective(weight, desiredPosition, omega, previousDuration,0, hessian, xGradient, yGradient, zGradient);

      MatrixTools.addDiagonal(hessian, 1e-7);

      solveProblem(hessian, xGradient, xSolution);
      solveProblem(hessian, yGradient, ySolution);
      solveProblem(hessian, zGradient, zSolution);

      double x0Value = 0.0;
      double y0Value = 0.0;
      double z0Value = 0.0;
      for (int i = 0; i < 6; i++)
      {
         x0Value += CoMTrajectoryPlannerTools.getCoMPositionCoefficientTimeFunction(i, omega, previousDuration) * xSolution.get(i);
         y0Value += CoMTrajectoryPlannerTools.getCoMPositionCoefficientTimeFunction(i, omega, previousDuration) * ySolution.get(i);
         z0Value += CoMTrajectoryPlannerTools.getCoMPositionCoefficientTimeFunction(i, omega, previousDuration) * zSolution.get(i);
      }

      double x1Value = 0.0;
      double y1Value = 0.0;
      double z1Value = 0.0;
      for (int i = 0; i < 6; i++)
      {
         x1Value += CoMTrajectoryPlannerTools.getCoMPositionCoefficientTimeFunction(i, omega, 0.0) * xSolution.get(i + 6);
         y1Value += CoMTrajectoryPlannerTools.getCoMPositionCoefficientTimeFunction(i, omega, 0.0) * ySolution.get(i + 6);
         z1Value += CoMTrajectoryPlannerTools.getCoMPositionCoefficientTimeFunction(i, omega, 0.0) * zSolution.get(i + 6);
      }

      assertEquals(desiredPosition.getX(), x0Value, 1e-4);
      assertEquals(desiredPosition.getY(), y0Value, 1e-4);
      assertEquals(desiredPosition.getZ(), z0Value, 1e-4);

      assertEquals(desiredPosition.getX(), x1Value, 1e-4);
      assertEquals(desiredPosition.getY(), y1Value, 1e-4);
      assertEquals(desiredPosition.getZ(), z1Value, 1e-4);

   }

   @Test
   public void testCoMVelocityContinuityObjective2()
   {
      double omega = 3.0;
      double previousDuration = 1.5;

      DMatrixRMaj hessian = new DMatrixRMaj(12, 12);
      DMatrixRMaj xGradient = new DMatrixRMaj(12, 1);
      DMatrixRMaj yGradient = new DMatrixRMaj(12, 1);
      DMatrixRMaj zGradient = new DMatrixRMaj(12, 1);
      DMatrixRMaj xSolution = new DMatrixRMaj(12, 1);
      DMatrixRMaj ySolution = new DMatrixRMaj(12, 1);
      DMatrixRMaj zSolution = new DMatrixRMaj(12, 1);

      FrameVector3D desiredVelocity = new FrameVector3D(ReferenceFrame.getWorldFrame(), 1.0, 1.5, 2.0);

      double weight = 10.0;

      CoMTrajectoryPlannerTools.addCoMVelocityContinuityObjectiveDirectly(weight, 0, 1, omega, previousDuration, hessian);
      CoMTrajectoryPlannerTools.addCoMVelocityObjective(weight, desiredVelocity, omega, previousDuration,0, hessian, xGradient, yGradient, zGradient);

      MatrixTools.addDiagonal(hessian, 1e-7);

      solveProblem(hessian, xGradient, xSolution);
      solveProblem(hessian, yGradient, ySolution);
      solveProblem(hessian, zGradient, zSolution);

      double x0Value = 0.0;
      double y0Value = 0.0;
      double z0Value = 0.0;
      for (int i = 0; i < 6; i++)
      {
         x0Value += CoMTrajectoryPlannerTools.getCoMVelocityCoefficientTimeFunction(i, omega, previousDuration) * xSolution.get(i);
         y0Value += CoMTrajectoryPlannerTools.getCoMVelocityCoefficientTimeFunction(i, omega, previousDuration) * ySolution.get(i);
         z0Value += CoMTrajectoryPlannerTools.getCoMVelocityCoefficientTimeFunction(i, omega, previousDuration) * zSolution.get(i);
      }

      double x1Value = 0.0;
      double y1Value = 0.0;
      double z1Value = 0.0;
      for (int i = 0; i < 6; i++)
      {
         x1Value += CoMTrajectoryPlannerTools.getCoMVelocityCoefficientTimeFunction(i, omega, 0.0) * xSolution.get(i + 6);
         y1Value += CoMTrajectoryPlannerTools.getCoMVelocityCoefficientTimeFunction(i, omega, 0.0) * ySolution.get(i + 6);
         z1Value += CoMTrajectoryPlannerTools.getCoMVelocityCoefficientTimeFunction(i, omega, 0.0) * zSolution.get(i + 6);
      }

      assertEquals(desiredVelocity.getX(), x0Value, 1e-4);
      assertEquals(desiredVelocity.getY(), y0Value, 1e-4);
      assertEquals(desiredVelocity.getZ(), z0Value, 1e-4);

      assertEquals(desiredVelocity.getX(), x1Value, 1e-4);
      assertEquals(desiredVelocity.getY(), y1Value, 1e-4);
      assertEquals(desiredVelocity.getZ(), z1Value, 1e-4);
   }


   @Test
   public void testCoMVelocityContinuityObjective()
   {
      double omega = 3.0;
      double previousDuration = 1.5;

      DMatrixRMaj jacobian = new DMatrixRMaj(12, 12);
      DMatrixRMaj hessian1 = new DMatrixRMaj(12, 12);
      DMatrixRMaj hessian2 = new DMatrixRMaj(12, 12);

      double weight = 10.0;
      double weightSqrt = Math.sqrt(weight);

      CoMTrajectoryPlannerTools.addCoMVelocityContinuityObjective(weightSqrt, 0, 1, 0, omega, previousDuration, jacobian);
      CoMTrajectoryPlannerTools.addCoMVelocityContinuityObjective(weight, 0, 1, omega, previousDuration, hessian2);
      CommonOps_DDRM.multInner(jacobian, hessian1);

      EjmlUnitTests.assertEquals(hessian1, hessian2, 1e-5);
   }

   @Test
   public void testCoMPositionObjective()
   {
      double omega = 3.0;
      double time = 1.5;
      double weight = 10.0;

      DMatrixRMaj hessian = new DMatrixRMaj(6, 6);
      DMatrixRMaj xGradient = new DMatrixRMaj(6, 1);
      DMatrixRMaj yGradient = new DMatrixRMaj(6, 1);
      DMatrixRMaj zGradient = new DMatrixRMaj(6, 1);

      Random random = new Random(1738L);
      for (int i = 0; i < 10; i++)
      {
         hessian.zero();
         xGradient.zero();
         yGradient.zero();
         zGradient.zero();

         MatrixTools.addDiagonal(hessian, 1e-6);

         FramePoint3D desiredPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame(), -10.0, 10.0);

         CoMTrajectoryPlannerTools.addCoMPositionObjective(weight, desiredPosition, omega, time, 0, hessian, xGradient, yGradient, zGradient);

         DMatrixRMaj xSolution = new DMatrixRMaj(6, 1);
         DMatrixRMaj ySolution = new DMatrixRMaj(6, 1);
         DMatrixRMaj zSolution = new DMatrixRMaj(6, 1);

         solveProblem(hessian, xGradient, xSolution);
         solveProblem(hessian, yGradient, ySolution);
         solveProblem(hessian, zGradient, zSolution);

         double xValue = 0.0, yValue = 0.0, zValue = 0.0;
         for (int j = 0; j < 6; j++)
         {
            xValue += CoMTrajectoryPlannerTools.getCoMPositionCoefficientTimeFunction(j, omega, time) * xSolution.get(j);
            yValue += CoMTrajectoryPlannerTools.getCoMPositionCoefficientTimeFunction(j, omega, time) * ySolution.get(j);
            zValue += CoMTrajectoryPlannerTools.getCoMPositionCoefficientTimeFunction(j, omega, time) * zSolution.get(j);
         }

         assertEquals(desiredPosition.getX(), xValue, 1e-6);
         assertEquals(desiredPosition.getY(), yValue, 1e-6);
         assertEquals(desiredPosition.getZ(), zValue, 1e-6);
      }
   }

   @Test
   public void testCoMVelocityObjective()
   {
      double omega = 3.0;
      double time = 1.5;
      double weight = 10.0;

      DMatrixRMaj hessian = new DMatrixRMaj(6, 6);
      DMatrixRMaj xGradient = new DMatrixRMaj(6, 1);
      DMatrixRMaj yGradient = new DMatrixRMaj(6, 1);
      DMatrixRMaj zGradient = new DMatrixRMaj(6, 1);


      MatrixTools.addDiagonal(hessian, 1e-6);

      FrameVector3DReadOnly desiredVelocity = new FrameVector3D(ReferenceFrame.getWorldFrame(), 1.5, 2.5, 3.5);

      CoMTrajectoryPlannerTools.addCoMVelocityObjective(weight, desiredVelocity, omega, time, 0, hessian, xGradient, yGradient, zGradient);

      DMatrixRMaj xSolution = new DMatrixRMaj(6, 1);
      DMatrixRMaj ySolution = new DMatrixRMaj(6, 1);
      DMatrixRMaj zSolution = new DMatrixRMaj(6, 1);

      solveProblem(hessian, xGradient, xSolution);
      solveProblem(hessian, yGradient, ySolution);
      solveProblem(hessian, zGradient, zSolution);

      double xValue = 0.0, yValue = 0.0, zValue = 0.0;
      for (int j = 0; j < 6; j++)
      {
         xValue += CoMTrajectoryPlannerTools.getCoMVelocityCoefficientTimeFunction(j, omega, time) * xSolution.get(j);
         yValue += CoMTrajectoryPlannerTools.getCoMVelocityCoefficientTimeFunction(j, omega, time) * ySolution.get(j);
         zValue += CoMTrajectoryPlannerTools.getCoMVelocityCoefficientTimeFunction(j, omega, time) * zSolution.get(j);
      }

      assertEquals(1.5, xValue, 1e-6);
      assertEquals(2.5, yValue, 1e-6);
      assertEquals(3.5, zValue, 1e-6);


      Random random = new Random(1738L);
      for (int i = 0; i < 10; i++)
      {
         hessian.zero();
         xGradient.zero();
         yGradient.zero();
         zGradient.zero();

         MatrixTools.addDiagonal(hessian, 1e-6);

         desiredVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame(), -10.0, 10.0);

         CoMTrajectoryPlannerTools.addCoMVelocityObjective(weight, desiredVelocity, omega, time, 0, hessian, xGradient, yGradient, zGradient);

         xSolution = new DMatrixRMaj(6, 1);
         ySolution = new DMatrixRMaj(6, 1);
         zSolution = new DMatrixRMaj(6, 1);

         solveProblem(hessian, xGradient, xSolution);
         solveProblem(hessian, yGradient, ySolution);
         solveProblem(hessian, zGradient, zSolution);

         xValue = 0.0;
         yValue = 0.0;
         zValue = 0.0;
         for (int j = 0; j < 6; j++)
         {
            xValue += CoMTrajectoryPlannerTools.getCoMVelocityCoefficientTimeFunction(j, omega, time) * xSolution.get(j);
            yValue += CoMTrajectoryPlannerTools.getCoMVelocityCoefficientTimeFunction(j, omega, time) * ySolution.get(j);
            zValue += CoMTrajectoryPlannerTools.getCoMVelocityCoefficientTimeFunction(j, omega, time) * zSolution.get(j);
         }

         assertEquals(desiredVelocity.getX(), xValue, 1e-6);
         assertEquals(desiredVelocity.getY(), yValue, 1e-6);
         assertEquals(desiredVelocity.getZ(), zValue, 1e-6);
      }
   }

   @Test
   public void testCoMAccelerationObjective()
   {
      double omega = 3.0;
      double time = 0.5;
      double weight = 1e2;

      Random random = new Random(1738L);

      FrameVector3D desiredAcceleration = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame(), -10, 10.0);

      DMatrixRMaj hessian = new DMatrixRMaj(6, 6);
      DMatrixRMaj xGradient = new DMatrixRMaj(6, 1);
      DMatrixRMaj yGradient = new DMatrixRMaj(6, 1);
      DMatrixRMaj zGradient = new DMatrixRMaj(6, 1);

      hessian.zero();
      zGradient.zero();

      MatrixTools.addDiagonal(hessian, 1e-10);

      CoMTrajectoryPlannerTools.addCoMAccelerationObjective(weight, 0, omega, time, desiredAcceleration, hessian, xGradient, yGradient, zGradient);

      DMatrixRMaj xSolution = new DMatrixRMaj(6, 1);
      DMatrixRMaj ySolution = new DMatrixRMaj(6, 1);
      DMatrixRMaj zSolution = new DMatrixRMaj(6, 1);

      solveProblem(hessian, xGradient, xSolution);
      solveProblem(hessian, yGradient, ySolution);
      solveProblem(hessian, zGradient, zSolution);

      double xValue = 0.0;
      double yValue = 0.0;
      double zValue = 0.0;
      for (int j = 0; j < 6; j++)
      {
         xValue += CoMTrajectoryPlannerTools.getCoMAccelerationCoefficientTimeFunction(j, omega, time) * xSolution.get(j);
         yValue += CoMTrajectoryPlannerTools.getCoMAccelerationCoefficientTimeFunction(j, omega, time) * ySolution.get(j);
         zValue += CoMTrajectoryPlannerTools.getCoMAccelerationCoefficientTimeFunction(j, omega, time) * zSolution.get(j);
      }

      assertEquals(desiredAcceleration.getX(), xValue, 1e-6);
      assertEquals(desiredAcceleration.getY(), yValue, 1e-6);
      assertEquals(desiredAcceleration.getZ(), zValue, 1e-6);
   }

   @Test
   public void testCoMAccelerationIsGravityObjective()
   {
      double omega = 3.0;
      double time = 1.5;
      double weight = 1.0;
      double gravity = -9.81;

      DMatrixRMaj hessian = new DMatrixRMaj(6, 6);
      DMatrixRMaj zGradient = new DMatrixRMaj(6, 1);

         hessian.zero();
         zGradient.zero();

         MatrixTools.addDiagonal(hessian, 1e-6);

         CoMTrajectoryPlannerTools.addCoMAccelerationIsGravityObjective(weight, 0, omega, time, gravity, hessian, zGradient);

         DMatrixRMaj zSolution = new DMatrixRMaj(6, 1);

         solveProblem(hessian, zGradient, zSolution);

         double zValue = 0.0;
         for (int j = 0; j < 6; j++)
         {
            zValue += CoMTrajectoryPlannerTools.getCoMAccelerationCoefficientTimeFunction(j, omega, time) * zSolution.get(j);
         }

         assertEquals(gravity, zValue, 1e-6);
   }

   @Test
   public void testCoMJerkObjective()
   {
      double omega = 3.0;
      double time = 1.5;
      double weight = 10.0;

      DMatrixRMaj hessian = new DMatrixRMaj(6, 6);
      DMatrixRMaj xGradient = new DMatrixRMaj(6, 1);
      DMatrixRMaj yGradient = new DMatrixRMaj(6, 1);
      DMatrixRMaj zGradient = new DMatrixRMaj(6, 1);

      Random random = new Random(1738L);
      for (int i = 0; i < 10; i++)
      {
         hessian.zero();
         xGradient.zero();
         yGradient.zero();
         zGradient.zero();

         MatrixTools.addDiagonal(hessian, 1e-6);

         FrameVector3DReadOnly desiredJerk = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

         CoMTrajectoryPlannerTools.addCoMJerkObjective(weight, desiredJerk, omega, time, 0, hessian, xGradient, yGradient, zGradient);

         DMatrixRMaj xSolution = new DMatrixRMaj(6, 1);
         DMatrixRMaj ySolution = new DMatrixRMaj(6, 1);
         DMatrixRMaj zSolution = new DMatrixRMaj(6, 1);

         solveProblem(hessian, xGradient, xSolution);
         solveProblem(hessian, yGradient, ySolution);
         solveProblem(hessian, zGradient, zSolution);

         double xValue = 0.0, yValue = 0.0, zValue = 0.0;
         for (int j = 0; j < 6; j++)
         {
            xValue += CoMTrajectoryPlannerTools.getCoMJerkCoefficientTimeFunction(j, omega, time) * xSolution.get(j);
            yValue += CoMTrajectoryPlannerTools.getCoMJerkCoefficientTimeFunction(j, omega, time) * ySolution.get(j);
            zValue += CoMTrajectoryPlannerTools.getCoMJerkCoefficientTimeFunction(j, omega, time) * zSolution.get(j);
         }

         assertEquals(desiredJerk.getX(), xValue, 1e-6);
         assertEquals(desiredJerk.getY(), yValue, 1e-6);
         assertEquals(desiredJerk.getZ(), zValue, 1e-6);
      }
   }

   @Test
   public void testDCMPositionObjective()
   {
      double omega = 3.0;
      double time = 1.5;
      double weight = 10.0;

      DMatrixRMaj hessian = new DMatrixRMaj(6, 6);
      DMatrixRMaj xGradient = new DMatrixRMaj(6, 1);
      DMatrixRMaj yGradient = new DMatrixRMaj(6, 1);
      DMatrixRMaj zGradient = new DMatrixRMaj(6, 1);

      Random random = new Random(1738L);
      for (int i = 0; i < 10; i++)
      {
         hessian.zero();
         xGradient.zero();
         yGradient.zero();
         zGradient.zero();

         MatrixTools.addDiagonal(hessian, 1e-6);

         FramePoint3D desiredPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());

         CoMTrajectoryPlannerTools.addDCMPositionObjective(weight, desiredPosition, omega, time, 0, hessian, xGradient, yGradient, zGradient);

         DMatrixRMaj xSolution = new DMatrixRMaj(6, 1);
         DMatrixRMaj ySolution = new DMatrixRMaj(6, 1);
         DMatrixRMaj zSolution = new DMatrixRMaj(6, 1);

         solveProblem(hessian, xGradient, xSolution);
         solveProblem(hessian, yGradient, ySolution);
         solveProblem(hessian, zGradient, zSolution);

         double xValue = 0.0, yValue = 0.0, zValue = 0.0;
         for (int j = 0; j < 6; j++)
         {
            xValue += CoMTrajectoryPlannerTools.getDCMPositionCoefficientTimeFunction(j, omega, time) * xSolution.get(j);
            yValue += CoMTrajectoryPlannerTools.getDCMPositionCoefficientTimeFunction(j, omega, time) * ySolution.get(j);
            zValue += CoMTrajectoryPlannerTools.getDCMPositionCoefficientTimeFunction(j, omega, time) * zSolution.get(j);
         }

         assertEquals(desiredPosition.getX(), xValue, 1e-6);
         assertEquals(desiredPosition.getY(), yValue, 1e-6);
         assertEquals(desiredPosition.getZ(), zValue, 1e-6);
      }
   }

   @Test
   public void testVRPPositionObjective()
   {
      double omega = 3.0;
      double time = 1.5;
      double weight = 10.0;

      DMatrixRMaj hessian = new DMatrixRMaj(6, 6);
      DMatrixRMaj xGradient = new DMatrixRMaj(6, 1);
      DMatrixRMaj yGradient = new DMatrixRMaj(6, 1);
      DMatrixRMaj zGradient = new DMatrixRMaj(6, 1);

      Random random = new Random(1738L);
      for (int i = 0; i < 10; i++)
      {
         hessian.zero();
         xGradient.zero();
         yGradient.zero();
         zGradient.zero();

         MatrixTools.addDiagonal(hessian, 1e-6);

         FramePoint3D desiredPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());

         CoMTrajectoryPlannerTools.addVRPPositionObjective(weight, desiredPosition, omega, time, 0, hessian, xGradient, yGradient, zGradient);

         DMatrixRMaj xSolution = new DMatrixRMaj(6, 1);
         DMatrixRMaj ySolution = new DMatrixRMaj(6, 1);
         DMatrixRMaj zSolution = new DMatrixRMaj(6, 1);

         solveProblem(hessian, xGradient, xSolution);
         solveProblem(hessian, yGradient, ySolution);
         solveProblem(hessian, zGradient, zSolution);

         double xValue = 0.0, yValue = 0.0, zValue = 0.0;
         for (int j = 0; j < 6; j++)
         {
            xValue += CoMTrajectoryPlannerTools.getVRPPositionCoefficientTimeFunction(j, omega, time) * xSolution.get(j);
            yValue += CoMTrajectoryPlannerTools.getVRPPositionCoefficientTimeFunction(j, omega, time) * ySolution.get(j);
            zValue += CoMTrajectoryPlannerTools.getVRPPositionCoefficientTimeFunction(j, omega, time) * zSolution.get(j);
         }

         assertEquals(desiredPosition.getX(), xValue, 1e-6);
         assertEquals(desiredPosition.getY(), yValue, 1e-6);
         assertEquals(desiredPosition.getZ(), zValue, 1e-6);
      }
   }

   @Test
   public void testVRPVelocityObjective()
   {
      double omega = 3.0;
      double time = 1.5;
      double weight = 10.0;

      DMatrixRMaj hessian = new DMatrixRMaj(6, 6);
      DMatrixRMaj xGradient = new DMatrixRMaj(6, 1);
      DMatrixRMaj yGradient = new DMatrixRMaj(6, 1);
      DMatrixRMaj zGradient = new DMatrixRMaj(6, 1);

      Random random = new Random(1738L);
      for (int i = 0; i < 10; i++)
      {
         hessian.zero();
         xGradient.zero();
         yGradient.zero();
         zGradient.zero();

         MatrixTools.addDiagonal(hessian, 1e-6);

         FrameVector3D desiredVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

         CoMTrajectoryPlannerTools.addVRPVelocityObjective(weight, desiredVelocity, omega, time, 0, hessian, xGradient, yGradient, zGradient);

         DMatrixRMaj xSolution = new DMatrixRMaj(6, 1);
         DMatrixRMaj ySolution = new DMatrixRMaj(6, 1);
         DMatrixRMaj zSolution = new DMatrixRMaj(6, 1);

         solveProblem(hessian, xGradient, xSolution);
         solveProblem(hessian, yGradient, ySolution);
         solveProblem(hessian, zGradient, zSolution);

         double xValue = 0.0, yValue = 0.0, zValue = 0.0;
         for (int j = 0; j < 6; j++)
         {
            xValue += CoMTrajectoryPlannerTools.getVRPVelocityCoefficientTimeFunction(j, omega, time) * xSolution.get(j);
            yValue += CoMTrajectoryPlannerTools.getVRPVelocityCoefficientTimeFunction(j, omega, time) * ySolution.get(j);
            zValue += CoMTrajectoryPlannerTools.getVRPVelocityCoefficientTimeFunction(j, omega, time) * zSolution.get(j);
         }

         assertEquals(desiredVelocity.getX(), xValue, 1e-6);
         assertEquals(desiredVelocity.getY(), yValue, 1e-6);
         assertEquals(desiredVelocity.getZ(), zValue, 1e-6);
      }
   }

   private static void solveProblem(DMatrixRMaj hessian, DMatrixRMaj gradient, DMatrixRMaj coefficientSolutionToPack)
   {
      LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.lu(6);
      solver.setA(hessian);

      DMatrixRMaj objective = new DMatrixRMaj(gradient);
      CommonOps_DDRM.scale(-0.5, objective);
      solver.solve(objective, coefficientSolutionToPack);
   }
}
