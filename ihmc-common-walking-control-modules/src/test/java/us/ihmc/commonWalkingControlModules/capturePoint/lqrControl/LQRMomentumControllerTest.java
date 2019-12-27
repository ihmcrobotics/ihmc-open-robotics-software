package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.apache.batik.ext.awt.geom.Linear;
import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import org.ejml.ops.EjmlUnitTests;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.matrixlib.NativeCommonOps;
import us.ihmc.robotics.linearAlgebra.MatrixExponentialCalculator;
import us.ihmc.robotics.math.trajectories.Trajectory3D;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public class LQRMomentumControllerTest
{
   private static final double epsilon = 1e-10;

   @Test
   public void testComputingS1()
   {
      LQRMomentumController controller = new LQRMomentumController();

      Point3D vrpStart = new Point3D(0.0, 0.0, 1.0);
      Point3D vrpEnd = new Point3D(1.0, 0.5, 1.0);
      Trajectory3D vrpTrajectory = new Trajectory3D(4);
      vrpTrajectory.setLinear(0.0, 1.0, vrpStart, vrpEnd);
      List<Trajectory3D> trajectories = new ArrayList<>();
      trajectories.add(vrpTrajectory);

      controller.setVrpTrajectory(trajectories);

      DenseMatrix64F AExpected = new DenseMatrix64F(6, 6);
      AExpected.set(0, 3, 1.0);
      AExpected.set(1, 4, 1.0);
      AExpected.set(2, 5, 1.0);

      DenseMatrix64F BExpected = new DenseMatrix64F(6, 3);
      BExpected.set(3, 0, 1.0);
      BExpected.set(4, 1, 1.0);
      BExpected.set(5, 2, 1.0);

      DenseMatrix64F CExpected = new DenseMatrix64F(3, 6);

      CExpected.set(0, 0, 1.0);
      CExpected.set(1, 1, 1.0);
      CExpected.set(2, 2, 1.0);

      DenseMatrix64F DExpected = new DenseMatrix64F(3, 3);

      DExpected.set(0, 0, -1.0 / MathTools.square(LQRMomentumController.omega));
      DExpected.set(1, 1, -1.0 / MathTools.square(LQRMomentumController.omega));
      DExpected.set(2, 2, -1.0 / MathTools.square(LQRMomentumController.omega));

      EjmlUnitTests.assertEquals(AExpected, controller.A, epsilon);
      EjmlUnitTests.assertEquals(BExpected, controller.B, epsilon);
      EjmlUnitTests.assertEquals(CExpected, controller.C, epsilon);
      EjmlUnitTests.assertEquals(DExpected, controller.D, epsilon);

      DenseMatrix64F QExpected = new DenseMatrix64F(3, 3);
      QExpected.set(0, 0, LQRMomentumController.defaultVrpTrackingWeight);
      QExpected.set(1, 1, LQRMomentumController.defaultVrpTrackingWeight);
      QExpected.set(2, 2, LQRMomentumController.defaultVrpTrackingWeight);

      DenseMatrix64F RExpected = new DenseMatrix64F(3, 3);
      RExpected.set(0, 0, LQRMomentumController.defaultMomentumRateWeight);
      RExpected.set(1, 1, LQRMomentumController.defaultMomentumRateWeight);
      RExpected.set(2, 2, LQRMomentumController.defaultMomentumRateWeight);

      EjmlUnitTests.assertEquals(QExpected, controller.Q, epsilon);
      EjmlUnitTests.assertEquals(RExpected, controller.R, epsilon);

      DenseMatrix64F Q1Expected = new DenseMatrix64F(3, 3);
      NativeCommonOps.multQuad(CExpected, QExpected, Q1Expected);

      DenseMatrix64F R1Expected = new DenseMatrix64F(3, 3);
      DenseMatrix64F R1InverseExpected = new DenseMatrix64F(3, 3);
      NativeCommonOps.multQuad(DExpected, QExpected, R1Expected);
      CommonOps.addEquals(R1Expected, RExpected);
      NativeCommonOps.invert(R1Expected, R1InverseExpected);

      EjmlUnitTests.assertEquals(Q1Expected, controller.Q1, epsilon);
      EjmlUnitTests.assertEquals(R1Expected, controller.R1, epsilon);
      EjmlUnitTests.assertEquals(R1InverseExpected, controller.R1Inverse, epsilon);

      DenseMatrix64F NExpected = new DenseMatrix64F(6, 3);
      DenseMatrix64F NTransposeExpected = new DenseMatrix64F(3, 6);
      DenseMatrix64F tempMatrix = new DenseMatrix64F(3, 3);
      CommonOps.mult(QExpected, DExpected, tempMatrix);
      CommonOps.multTransA(CExpected, tempMatrix, NExpected);
      CommonOps.transpose(NExpected, NTransposeExpected);

      EjmlUnitTests.assertEquals(NExpected, controller.N, epsilon);
      EjmlUnitTests.assertEquals(NTransposeExpected, controller.NTranspose, epsilon);


      controller.computeS1();

      EjmlUnitTests.assertEquals(AExpected, controller.A, epsilon);
      EjmlUnitTests.assertEquals(BExpected, controller.B, epsilon);
      EjmlUnitTests.assertEquals(CExpected, controller.C, epsilon);
      EjmlUnitTests.assertEquals(DExpected, controller.D, epsilon);

      EjmlUnitTests.assertEquals(QExpected, controller.Q, epsilon);
      EjmlUnitTests.assertEquals(RExpected, controller.R, epsilon);

      EjmlUnitTests.assertEquals(Q1Expected, controller.Q1, epsilon);
      EjmlUnitTests.assertEquals(R1Expected, controller.R1, epsilon);
      EjmlUnitTests.assertEquals(R1InverseExpected, controller.R1Inverse, epsilon);

      EjmlUnitTests.assertEquals(NExpected, controller.N, epsilon);

      DenseMatrix64F QRiccatiExpected = new DenseMatrix64F(3, 3);
      DenseMatrix64F ARiccatiExpected = new DenseMatrix64F(6, 6);

      NativeCommonOps.multQuad(NTransposeExpected, R1InverseExpected, QRiccatiExpected);
      CommonOps.scale(-1.0, QRiccatiExpected);
      CommonOps.addEquals(QRiccatiExpected, Q1Expected);

      tempMatrix = new DenseMatrix64F(3, 6);
      CommonOps.multTransB(R1InverseExpected, NExpected, tempMatrix);
      CommonOps.mult(-1.0, BExpected, tempMatrix, ARiccatiExpected);
      CommonOps.addEquals(ARiccatiExpected, AExpected);

      EjmlUnitTests.assertEquals(QRiccatiExpected, controller.QRiccati, epsilon);
      EjmlUnitTests.assertEquals(ARiccatiExpected, controller.ARiccati, epsilon);


      Random random = new Random(1738L);
      DenseMatrix64F S1 = controller.S1;
//      DenseMatrix64F S1Constructor = new DenseMatrix64F(6, 6);
//      S1Constructor.setData(RandomNumbers.nextDoubleArray(random, 36, 10.0));
//      CommonOps.transpose(S1Constructor, S1);
//      CommonOps.addEquals(S1, S1Constructor);
//
      DenseMatrix64F NB = new DenseMatrix64F(NExpected);
      CommonOps.transpose(NB);
      CommonOps.multAddTransA(BExpected, S1, NB);
      DenseMatrix64F S1DotExpected = new DenseMatrix64F(6, 6);
      NativeCommonOps.multQuad(NB, R1InverseExpected, S1DotExpected);
      CommonOps.addEquals(S1DotExpected, -1.0, Q1Expected);
      CommonOps.multAdd(-1.0, S1, AExpected, S1DotExpected);
      CommonOps.multAddTransA(-1.0, AExpected, S1, S1DotExpected);

      DenseMatrix64F H = new DenseMatrix64F( 4, 4);
      H.set(0, 1, 1.0);
      H.set(1, 0, 0.5);
      H.set(1, 3, -1.0);
      H.set(2, 0, -0.5);
      H.set(2, 3, -0.5);
      H.set(3, 2, -1.0);



      tempMatrix.reshape(6, 3);
      DenseMatrix64F S1Dot = new DenseMatrix64F(6, 6);
      DenseMatrix64F BTranspose = new DenseMatrix64F(controller.B);
      CommonOps.transpose(BTranspose);
      NativeCommonOps.multQuad(BTranspose, controller.R1Inverse, tempMatrix);
      NativeCommonOps.multQuad(S1, tempMatrix, S1Dot);
      CommonOps.addEquals(S1Dot, -1.0, controller.QRiccati);
      CommonOps.multAdd(-1.0, S1, controller.ARiccati, S1Dot);
      CommonOps.multAddTransA(-1.0, controller.ARiccati, S1, S1Dot);

      EjmlUnitTests.assertEquals(S1DotExpected, S1Dot, epsilon);
      EjmlUnitTests.assertEquals(new DenseMatrix64F(6, 6), S1Dot, epsilon);
   }

   @Test
   public void testComputingS2FromSingleLinearTrajectory()
   {
      LQRMomentumController controller = new LQRMomentumController();

      Point3D vrpStart = new Point3D(0.0, 0.0, 1.0);
      Point3D vrpEnd = new Point3D(1.0, 0.5, 1.0);
      Trajectory3D vrpTrajectory = new Trajectory3D(4);
      double finalTime = 1.5;
      vrpTrajectory.setLinear(0.0, finalTime, vrpStart, vrpEnd);
      List<Trajectory3D> trajectories = new ArrayList<>();
      trajectories.add(vrpTrajectory);

      controller.setVrpTrajectory(trajectories);

      controller.computeS1();
      controller.computeS2Parameters();

      DenseMatrix64F NExpected = new DenseMatrix64F(6, 3);
      DenseMatrix64F NBExpected = new DenseMatrix64F(3, 6);
      DenseMatrix64F NTransposeExpected = new DenseMatrix64F(3, 6);
      DenseMatrix64F tempMatrix = new DenseMatrix64F(3, 3);
      CommonOps.mult(controller.Q, controller.D, tempMatrix);
      CommonOps.multTransA(controller.C, tempMatrix, NExpected);
      CommonOps.transpose(NExpected, NTransposeExpected);

      CommonOps.multTransA(controller.B, controller.getCostHessian(), NBExpected);
      CommonOps.addEquals(NBExpected, NTransposeExpected);

      LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.general(0, 0);

      DenseMatrix64F A2Expected = new DenseMatrix64F(6, 6);
      DenseMatrix64F A2InverseExpected = new DenseMatrix64F(6, 6);
      DenseMatrix64F B2Expected = new DenseMatrix64F(6, 3);

      CommonOps.transpose(controller.A, A2Expected);
      CommonOps.scale(-1.0, A2Expected);

      tempMatrix.reshape(3, 6);
      CommonOps.multTransB(controller.R1Inverse, controller.B, tempMatrix);
      CommonOps.multAddTransA(NBExpected, tempMatrix, A2Expected);

      DenseMatrix64F tempMatrix2 = new DenseMatrix64F(3, 3);
      tempMatrix.reshape(6, 3);
      CommonOps.multTransA(NBExpected, controller.R1Inverse, tempMatrix);
      CommonOps.mult(-1.0, controller.D, controller.Q, tempMatrix2);
      CommonOps.mult(tempMatrix, tempMatrix2, B2Expected);
      CommonOps.multAddTransA(2.0, controller.C, controller.Q, B2Expected);

      solver.setA(A2Expected);
      solver.invert(A2InverseExpected);

      EjmlUnitTests.assertEquals(NBExpected, controller.NB, epsilon);
      EjmlUnitTests.assertEquals(A2Expected, controller.A2, epsilon);
      EjmlUnitTests.assertEquals(B2Expected, controller.B2, epsilon);
      EjmlUnitTests.assertEquals(A2InverseExpected, controller.A2Inverse, epsilon);


      DenseMatrix64F A2InverseB2 = new DenseMatrix64F(6, 3);
      DenseMatrix64F R1InvDQ = new DenseMatrix64F(3, 3);
      DenseMatrix64F DQ = new DenseMatrix64F(3, 3);
      DenseMatrix64F R1InvBTrans = new DenseMatrix64F(3, 6);

      CommonOps.mult(A2InverseExpected, B2Expected, A2InverseB2);
      CommonOps.mult(controller.D, controller.Q, DQ);
      CommonOps.mult(controller.R1Inverse, DQ, R1InvDQ);
      CommonOps.multTransB(controller.R1Inverse, controller.B, R1InvBTrans);

      assertEquals(1, controller.alphas.size());
      assertEquals(1, controller.betas.size());
      assertEquals(1, controller.gammas.size());
      assertEquals(2, controller.betas.get(0).size());
      assertEquals(2, controller.gammas.get(0).size());

      DenseMatrix64F beta1Expected = new DenseMatrix64F(6, 1);
      DenseMatrix64F beta2Expected = new DenseMatrix64F(6, 1);
      DenseMatrix64F gamma1Expected = new DenseMatrix64F(3, 1);
      DenseMatrix64F gamma2Expected = new DenseMatrix64F(3, 1);
      DenseMatrix64F coefficients = new DenseMatrix64F(3, 1);

      vrpTrajectory.getCoefficients(1, coefficients);
      CommonOps.mult(-1, A2InverseB2, coefficients, beta2Expected);

      CommonOps.mult(R1InvDQ, coefficients, gamma2Expected);
      CommonOps.multAdd(-0.5, R1InvBTrans, beta2Expected, gamma2Expected);

      vrpTrajectory.getCoefficients(0, coefficients);
      CommonOps.mult(A2InverseExpected, beta2Expected, beta1Expected);
      CommonOps.multAdd(-1.0, A2InverseB2, coefficients, beta1Expected);

      CommonOps.mult(R1InvDQ, coefficients, gamma1Expected);
      CommonOps.multAdd(-0.5, R1InvBTrans, beta1Expected, gamma1Expected);

      EjmlUnitTests.assertEquals(beta2Expected, controller.betas.get(0).get(1), epsilon);
      EjmlUnitTests.assertEquals(gamma2Expected, controller.gammas.get(0).get(1), epsilon);
      EjmlUnitTests.assertEquals(beta1Expected, controller.betas.get(0).get(0), epsilon);
      EjmlUnitTests.assertEquals(gamma1Expected, controller.gammas.get(0).get(0), epsilon);

      MatrixExponentialCalculator matrixExponentialCalculator = new MatrixExponentialCalculator(6);

      DenseMatrix64F alphaExpected = new DenseMatrix64F(6, 1);

      DenseMatrix64F timeScaledDynamics = new DenseMatrix64F(6, 6);
      DenseMatrix64F matrixExponential = new DenseMatrix64F(6, 6);
      DenseMatrix64F betaSum = new DenseMatrix64F(6, 1);

      CommonOps.scale(finalTime, A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      CommonOps.addEquals(betaSum, -1.0 * MathTools.pow(finalTime, 0), beta1Expected);
      CommonOps.addEquals(betaSum, -1.0 * MathTools.pow(finalTime, 1), beta2Expected);

      solver.setA(matrixExponential);
      solver.solve(betaSum, alphaExpected);

      EjmlUnitTests.assertEquals(timeScaledDynamics, controller.timeScaledDynamics, epsilon);
      EjmlUnitTests.assertEquals(matrixExponential, controller.exponential, epsilon);
      EjmlUnitTests.assertEquals(betaSum, controller.summedBetas, epsilon);
      EjmlUnitTests.assertEquals(alphaExpected, controller.alphas.get(0), epsilon);

      for (double time = 0.0; time <= finalTime; time += 0.01)
      {
         controller.computeS2(time);

         CommonOps.scale(time, A2Expected, timeScaledDynamics);
         matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);

         DenseMatrix64F s2Expected = new DenseMatrix64F(6, 1);
         CommonOps.mult(matrixExponential, alphaExpected, s2Expected);
         CommonOps.addEquals(s2Expected, MathTools.pow(time, 0), beta1Expected);
         CommonOps.addEquals(s2Expected, MathTools.pow(time, 1), beta2Expected);

         EjmlUnitTests.assertEquals(timeScaledDynamics, controller.timeScaledDynamics, epsilon);
         EjmlUnitTests.assertEquals(matrixExponential, controller.exponential, epsilon);
         EjmlUnitTests.assertEquals(betaSum, controller.summedBetas, epsilon);
         EjmlUnitTests.assertEquals(alphaExpected, controller.alphas.get(0), epsilon);

         EjmlUnitTests.assertEquals(s2Expected, controller.getCostJacobian(), epsilon);
      }

      controller.computeS2(finalTime);

      // should be zero at tf
      DenseMatrix64F zeroMatrix = new DenseMatrix64F(6, 1);
      EjmlUnitTests.assertEquals(zeroMatrix, controller.getCostJacobian(), epsilon);
   }

   @Test
   public void testComputingS2FromSingleCubicTrajectory()
   {
      LQRMomentumController controller = new LQRMomentumController();

      Point3D vrpStart = new Point3D(0.0, 0.0, 1.0);
      Point3D vrpEnd = new Point3D(1.0, 0.5, 1.0);
      Trajectory3D vrpTrajectory = new Trajectory3D(4);
      double finalTime = 1.5;
      vrpTrajectory.setCubic(0.0, finalTime, vrpStart, vrpEnd);
      List<Trajectory3D> trajectories = new ArrayList<>();
      trajectories.add(vrpTrajectory);

      controller.setVrpTrajectory(trajectories);

      controller.computeS1();
      controller.computeS2Parameters();

      DenseMatrix64F NExpected = new DenseMatrix64F(6, 3);
      DenseMatrix64F NBExpected = new DenseMatrix64F(3, 6);
      DenseMatrix64F NTransposeExpected = new DenseMatrix64F(3, 6);
      DenseMatrix64F tempMatrix = new DenseMatrix64F(3, 3);
      CommonOps.mult(controller.Q, controller.D, tempMatrix);
      CommonOps.multTransA(controller.C, tempMatrix, NExpected);
      CommonOps.transpose(NExpected, NTransposeExpected);

      CommonOps.multTransA(controller.B, controller.getCostHessian(), NBExpected);
      CommonOps.addEquals(NBExpected, NTransposeExpected);

      LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.general(0, 0);

      DenseMatrix64F A2Expected = new DenseMatrix64F(6, 6);
      DenseMatrix64F A2InverseExpected = new DenseMatrix64F(6, 6);
      DenseMatrix64F B2Expected = new DenseMatrix64F(6, 3);

      CommonOps.transpose(controller.A, A2Expected);
      CommonOps.scale(-1.0, A2Expected);

      tempMatrix.reshape(3, 6);
      CommonOps.multTransB(controller.R1Inverse, controller.B, tempMatrix);
      CommonOps.multAddTransA(NBExpected, tempMatrix, A2Expected);

      DenseMatrix64F tempMatrix2 = new DenseMatrix64F(3, 3);
      tempMatrix.reshape(6, 3);
      CommonOps.multTransA(NBExpected, controller.R1Inverse, tempMatrix);
      CommonOps.mult(-1.0, controller.D, controller.Q, tempMatrix2);
      CommonOps.mult(tempMatrix, tempMatrix2, B2Expected);
      CommonOps.multAddTransA(2.0, controller.C, controller.Q, B2Expected);

      solver.setA(A2Expected);
      solver.invert(A2InverseExpected);

      EjmlUnitTests.assertEquals(NBExpected, controller.NB, epsilon);
      EjmlUnitTests.assertEquals(A2Expected, controller.A2, epsilon);
      EjmlUnitTests.assertEquals(B2Expected, controller.B2, epsilon);
      EjmlUnitTests.assertEquals(A2InverseExpected, controller.A2Inverse, epsilon);


      DenseMatrix64F A2InverseB2 = new DenseMatrix64F(6, 3);
      DenseMatrix64F R1InvDQ = new DenseMatrix64F(3, 3);
      DenseMatrix64F DQ = new DenseMatrix64F(3, 3);
      DenseMatrix64F R1InvBTrans = new DenseMatrix64F(3, 6);

      CommonOps.mult(A2InverseExpected, B2Expected, A2InverseB2);
      CommonOps.mult(controller.D, controller.Q, DQ);
      CommonOps.mult(controller.R1Inverse, DQ, R1InvDQ);
      CommonOps.multTransB(controller.R1Inverse, controller.B, R1InvBTrans);

      assertEquals(1, controller.alphas.size());
      assertEquals(1, controller.betas.size());
      assertEquals(1, controller.gammas.size());
      assertEquals(4, controller.betas.get(0).size());
      assertEquals(4, controller.gammas.get(0).size());

      DenseMatrix64F beta1Expected = new DenseMatrix64F(6, 1);
      DenseMatrix64F beta2Expected = new DenseMatrix64F(6, 1);
      DenseMatrix64F beta3Expected = new DenseMatrix64F(6, 1);
      DenseMatrix64F beta4Expected = new DenseMatrix64F(6, 1);
      DenseMatrix64F gamma1Expected = new DenseMatrix64F(3, 1);
      DenseMatrix64F gamma2Expected = new DenseMatrix64F(3, 1);
      DenseMatrix64F gamma3Expected = new DenseMatrix64F(3, 1);
      DenseMatrix64F gamma4Expected = new DenseMatrix64F(3, 1);
      DenseMatrix64F coefficients = new DenseMatrix64F(3, 1);

      vrpTrajectory.getCoefficients(3, coefficients);
      CommonOps.mult(-1, A2InverseB2, coefficients, beta4Expected);

      CommonOps.mult(R1InvDQ, coefficients, gamma4Expected);
      CommonOps.multAdd(-0.5, R1InvBTrans, beta4Expected, gamma4Expected);

      vrpTrajectory.getCoefficients(2, coefficients);
      CommonOps.mult(3, A2InverseExpected, beta4Expected, beta3Expected);
      CommonOps.multAdd(-1.0, A2InverseB2, coefficients, beta3Expected);

      CommonOps.mult(R1InvDQ, coefficients, gamma3Expected);
      CommonOps.multAdd(-0.5, R1InvBTrans, beta3Expected, gamma3Expected);

      vrpTrajectory.getCoefficients(1, coefficients);
      CommonOps.mult(2, A2InverseExpected, beta3Expected, beta2Expected);
      CommonOps.multAdd(-1.0, A2InverseB2, coefficients, beta2Expected);

      CommonOps.mult(R1InvDQ, coefficients, gamma2Expected);
      CommonOps.multAdd(-0.5, R1InvBTrans, beta2Expected, gamma2Expected);

      vrpTrajectory.getCoefficients(0, coefficients);
      CommonOps.mult(A2InverseExpected, beta2Expected, beta1Expected);
      CommonOps.multAdd(-1.0, A2InverseB2, coefficients, beta1Expected);

      CommonOps.mult(R1InvDQ, coefficients, gamma1Expected);
      CommonOps.multAdd(-0.5, R1InvBTrans, beta1Expected, gamma1Expected);

      EjmlUnitTests.assertEquals(beta4Expected, controller.betas.get(0).get(3), epsilon);
      EjmlUnitTests.assertEquals(beta3Expected, controller.betas.get(0).get(2), epsilon);
      EjmlUnitTests.assertEquals(beta2Expected, controller.betas.get(0).get(1), epsilon);
      EjmlUnitTests.assertEquals(beta1Expected, controller.betas.get(0).get(0), epsilon);
      EjmlUnitTests.assertEquals(gamma4Expected, controller.gammas.get(0).get(3), epsilon);
      EjmlUnitTests.assertEquals(gamma3Expected, controller.gammas.get(0).get(2), epsilon);
      EjmlUnitTests.assertEquals(gamma2Expected, controller.gammas.get(0).get(1), epsilon);
      EjmlUnitTests.assertEquals(gamma1Expected, controller.gammas.get(0).get(0), epsilon);

      MatrixExponentialCalculator matrixExponentialCalculator = new MatrixExponentialCalculator(6);

      DenseMatrix64F alphaExpected = new DenseMatrix64F(6, 1);

      DenseMatrix64F timeScaledDynamics = new DenseMatrix64F(6, 6);
      DenseMatrix64F matrixExponential = new DenseMatrix64F(6, 6);
      DenseMatrix64F betaSum = new DenseMatrix64F(6, 1);

      CommonOps.scale(finalTime, A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      CommonOps.addEquals(betaSum, -1.0 * MathTools.pow(finalTime, 0), beta1Expected);
      CommonOps.addEquals(betaSum, -1.0 * MathTools.pow(finalTime, 1), beta2Expected);
      CommonOps.addEquals(betaSum, -1.0 * MathTools.pow(finalTime, 2), beta3Expected);
      CommonOps.addEquals(betaSum, -1.0 * MathTools.pow(finalTime, 3), beta4Expected);

      solver.setA(matrixExponential);
      solver.solve(betaSum, alphaExpected);

      EjmlUnitTests.assertEquals(timeScaledDynamics, controller.timeScaledDynamics, epsilon);
      EjmlUnitTests.assertEquals(matrixExponential, controller.exponential, epsilon);
      EjmlUnitTests.assertEquals(betaSum, controller.summedBetas, epsilon);
      EjmlUnitTests.assertEquals(alphaExpected, controller.alphas.get(0), epsilon);

      for (double time = 0.0; time <= finalTime; time += 0.01)
      {
         controller.computeS2(time);

         CommonOps.scale(time, A2Expected, timeScaledDynamics);
         matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);

         DenseMatrix64F s2Expected = new DenseMatrix64F(6, 1);
         CommonOps.mult(matrixExponential, alphaExpected, s2Expected);
         CommonOps.addEquals(s2Expected, MathTools.pow(time, 0), beta1Expected);
         CommonOps.addEquals(s2Expected, MathTools.pow(time, 1), beta2Expected);
         CommonOps.addEquals(s2Expected, MathTools.pow(time, 2), beta3Expected);
         CommonOps.addEquals(s2Expected, MathTools.pow(time, 3), beta4Expected);

         EjmlUnitTests.assertEquals(timeScaledDynamics, controller.timeScaledDynamics, epsilon);
         EjmlUnitTests.assertEquals(matrixExponential, controller.exponential, epsilon);
         EjmlUnitTests.assertEquals(betaSum, controller.summedBetas, epsilon);
         EjmlUnitTests.assertEquals(alphaExpected, controller.alphas.get(0), epsilon);

         EjmlUnitTests.assertEquals(s2Expected, controller.getCostJacobian(), epsilon);
      }

      controller.computeS2(finalTime);

      // should be zero at tf
      DenseMatrix64F zeroMatrix = new DenseMatrix64F(6, 1);
      EjmlUnitTests.assertEquals(zeroMatrix, controller.getCostJacobian(), epsilon);
   }

   @Test
   public void testComputingS2FromTwoLinearTrajectories()
   {
      LQRMomentumController controller = new LQRMomentumController();

      Point3D vrpStart = new Point3D(0.0, 0.0, 1.0);
      Point3D vrpMiddle = new Point3D(0.6, 0.75, 1.0);
      Point3D vrpEnd = new Point3D(1.0, 0.5, 1.0);
      Trajectory3D vrpTrajectory1 = new Trajectory3D(4);
      Trajectory3D vrpTrajectory2 = new Trajectory3D(4);
      double finalTime1 = 1.5;
      double finalTime2 = 3.1;
      vrpTrajectory1.setLinear(0.0, finalTime1, vrpStart, vrpMiddle);
      vrpTrajectory2.setLinear(finalTime1, finalTime2, vrpMiddle, vrpEnd);
      List<Trajectory3D> trajectories = new ArrayList<>();
      trajectories.add(vrpTrajectory1);
      trajectories.add(vrpTrajectory2);

      controller.setVrpTrajectory(trajectories);

      controller.computeS1();
      controller.computeS2Parameters();

      DenseMatrix64F NExpected = new DenseMatrix64F(6, 3);
      DenseMatrix64F NBExpected = new DenseMatrix64F(3, 6);
      DenseMatrix64F NTransposeExpected = new DenseMatrix64F(3, 6);
      DenseMatrix64F tempMatrix = new DenseMatrix64F(3, 3);
      CommonOps.mult(controller.Q, controller.D, tempMatrix);
      CommonOps.multTransA(controller.C, tempMatrix, NExpected);
      CommonOps.transpose(NExpected, NTransposeExpected);

      CommonOps.multTransA(controller.B, controller.getCostHessian(), NBExpected);
      CommonOps.addEquals(NBExpected, NTransposeExpected);

      LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.general(0, 0);

      DenseMatrix64F A2Expected = new DenseMatrix64F(6, 6);
      DenseMatrix64F A2InverseExpected = new DenseMatrix64F(6, 6);
      DenseMatrix64F B2Expected = new DenseMatrix64F(6, 3);

      CommonOps.transpose(controller.A, A2Expected);
      CommonOps.scale(-1.0, A2Expected);

      tempMatrix.reshape(3, 6);
      CommonOps.multTransB(controller.R1Inverse, controller.B, tempMatrix);
      CommonOps.multAddTransA(NBExpected, tempMatrix, A2Expected);

      DenseMatrix64F tempMatrix2 = new DenseMatrix64F(3, 3);
      tempMatrix.reshape(6, 3);
      CommonOps.multTransA(NBExpected, controller.R1Inverse, tempMatrix);
      CommonOps.mult(-1.0, controller.D, controller.Q, tempMatrix2);
      CommonOps.mult(tempMatrix, tempMatrix2, B2Expected);
      CommonOps.multAddTransA(2.0, controller.C, controller.Q, B2Expected);

      solver.setA(A2Expected);
      solver.invert(A2InverseExpected);

      EjmlUnitTests.assertEquals(NBExpected, controller.NB, epsilon);
      EjmlUnitTests.assertEquals(A2Expected, controller.A2, epsilon);
      EjmlUnitTests.assertEquals(B2Expected, controller.B2, epsilon);
      EjmlUnitTests.assertEquals(A2InverseExpected, controller.A2Inverse, epsilon);


      DenseMatrix64F A2InverseB2 = new DenseMatrix64F(6, 3);
      DenseMatrix64F R1InvDQ = new DenseMatrix64F(3, 3);
      DenseMatrix64F DQ = new DenseMatrix64F(3, 3);
      DenseMatrix64F R1InvBTrans = new DenseMatrix64F(3, 6);

      CommonOps.mult(A2InverseExpected, B2Expected, A2InverseB2);
      CommonOps.mult(controller.D, controller.Q, DQ);
      CommonOps.mult(controller.R1Inverse, DQ, R1InvDQ);
      CommonOps.multTransB(controller.R1Inverse, controller.B, R1InvBTrans);

      assertEquals(2, controller.alphas.size());
      assertEquals(2, controller.betas.size());
      assertEquals(2, controller.gammas.size());
      assertEquals(2, controller.betas.get(0).size());
      assertEquals(2, controller.betas.get(1).size());
      assertEquals(2, controller.gammas.get(0).size());
      assertEquals(2, controller.gammas.get(1).size());

      DenseMatrix64F beta11Expected = new DenseMatrix64F(6, 1);
      DenseMatrix64F beta12Expected = new DenseMatrix64F(6, 1);
      DenseMatrix64F beta21Expected = new DenseMatrix64F(6, 1);
      DenseMatrix64F beta22Expected = new DenseMatrix64F(6, 1);
      DenseMatrix64F gamma11Expected = new DenseMatrix64F(3, 1);
      DenseMatrix64F gamma12Expected = new DenseMatrix64F(3, 1);
      DenseMatrix64F gamma21Expected = new DenseMatrix64F(3, 1);
      DenseMatrix64F gamma22Expected = new DenseMatrix64F(3, 1);
      DenseMatrix64F coefficients = new DenseMatrix64F(3, 1);

      vrpTrajectory2.getCoefficients(1, coefficients);
      CommonOps.mult(-1, A2InverseB2, coefficients, beta22Expected);

      CommonOps.mult(R1InvDQ, coefficients, gamma22Expected);
      CommonOps.multAdd(-0.5, R1InvBTrans, beta22Expected, gamma22Expected);

      vrpTrajectory2.getCoefficients(0, coefficients);
      CommonOps.mult(A2InverseExpected, beta22Expected, beta21Expected);
      CommonOps.multAdd(-1.0, A2InverseB2, coefficients, beta21Expected);

      CommonOps.mult(R1InvDQ, coefficients, gamma21Expected);
      CommonOps.multAdd(-0.5, R1InvBTrans, beta21Expected, gamma21Expected);

      EjmlUnitTests.assertEquals(beta21Expected, controller.betas.get(1).get(0), epsilon);
      EjmlUnitTests.assertEquals(beta22Expected, controller.betas.get(1).get(1), epsilon);
      EjmlUnitTests.assertEquals(gamma21Expected, controller.gammas.get(1).get(0), epsilon);
      EjmlUnitTests.assertEquals(gamma22Expected, controller.gammas.get(1).get(1), epsilon);


      vrpTrajectory1.getCoefficients(1, coefficients);
      CommonOps.mult(-1, A2InverseB2, coefficients, beta12Expected);

      CommonOps.mult(R1InvDQ, coefficients, gamma12Expected);
      CommonOps.multAdd(-0.5, R1InvBTrans, beta12Expected, gamma12Expected);


      vrpTrajectory1.getCoefficients(0, coefficients);
      CommonOps.mult(A2InverseExpected, beta12Expected, beta11Expected);
      CommonOps.multAdd(-1.0, A2InverseB2, coefficients, beta11Expected);

      CommonOps.mult(R1InvDQ, coefficients, gamma11Expected);
      CommonOps.multAdd(-0.5, R1InvBTrans, beta11Expected, gamma11Expected);


      EjmlUnitTests.assertEquals(beta11Expected, controller.betas.get(0).get(0), epsilon);
      EjmlUnitTests.assertEquals(beta12Expected, controller.betas.get(0).get(1), epsilon);
      EjmlUnitTests.assertEquals(gamma11Expected, controller.gammas.get(0).get(0), epsilon);
      EjmlUnitTests.assertEquals(gamma12Expected, controller.gammas.get(0).get(1), epsilon);


      MatrixExponentialCalculator matrixExponentialCalculator = new MatrixExponentialCalculator(6);

      DenseMatrix64F alpha2Expected = new DenseMatrix64F(6, 1);
      DenseMatrix64F alpha1Expected = new DenseMatrix64F(6, 1);

      DenseMatrix64F timeScaledDynamics = new DenseMatrix64F(6, 6);
      DenseMatrix64F matrixExponential = new DenseMatrix64F(6, 6);
      DenseMatrix64F betaSum = new DenseMatrix64F(6, 1);

      CommonOps.scale((finalTime2 - finalTime1), A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      CommonOps.addEquals(betaSum, -1.0 * MathTools.pow(finalTime2 - finalTime1, 0), beta21Expected);
      CommonOps.addEquals(betaSum, -1.0 * MathTools.pow(finalTime2 - finalTime1, 1), beta22Expected);

      solver.setA(matrixExponential);
      solver.solve(betaSum, alpha2Expected);

      CommonOps.scale(finalTime1, A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      betaSum.zero();
      CommonOps.addEquals(betaSum, alpha2Expected);
      CommonOps.addEquals(betaSum, beta21Expected);
      CommonOps.addEquals(betaSum, -1.0 * MathTools.pow(finalTime1, 0), beta11Expected);
      CommonOps.addEquals(betaSum, -1.0 * MathTools.pow(finalTime1, 1), beta12Expected);

      solver.setA(matrixExponential);
      solver.solve(betaSum, alpha1Expected);

      EjmlUnitTests.assertEquals(alpha2Expected, controller.alphas.get(1), epsilon);
      EjmlUnitTests.assertEquals(alpha1Expected, controller.alphas.get(0), epsilon);



      // check for continuity

      DenseMatrix64F s2EndOf1 = new DenseMatrix64F(6, 1);
      DenseMatrix64F s2StartOf2 = new DenseMatrix64F(6, 1);

      CommonOps.scale(finalTime1, A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      CommonOps.mult(matrixExponential, controller.alphas.get(0), s2EndOf1);
      CommonOps.addEquals(s2EndOf1, MathTools.pow(finalTime1, 0), controller.betas.get(0).get(0));
      CommonOps.addEquals(s2EndOf1, MathTools.pow(finalTime1, 1), controller.betas.get(0).get(1));

      CommonOps.scale(0, A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      CommonOps.mult(matrixExponential, controller.alphas.get(1), s2StartOf2);
      CommonOps.addEquals(s2StartOf2, MathTools.pow(0, 0), controller.betas.get(1).get(0));
      CommonOps.addEquals(s2StartOf2, MathTools.pow(0, 1), controller.betas.get(1).get(1));

      EjmlUnitTests.assertEquals(s2EndOf1, s2StartOf2, epsilon);

      for (double time = 0; time <= finalTime1; time += 0.01)
      {
         controller.computeS2(time);

         CommonOps.scale(time, A2Expected, timeScaledDynamics);
         matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);

         DenseMatrix64F s2Expected = new DenseMatrix64F(6, 1);
         CommonOps.mult(matrixExponential, alpha1Expected, s2Expected);
         CommonOps.addEquals(s2Expected, MathTools.pow(time, 0), beta11Expected);
         CommonOps.addEquals(s2Expected, MathTools.pow(time, 1), beta12Expected);

         EjmlUnitTests.assertEquals(timeScaledDynamics, controller.timeScaledDynamics, epsilon);
         EjmlUnitTests.assertEquals(matrixExponential, controller.exponential, epsilon);
         EjmlUnitTests.assertEquals(betaSum, controller.summedBetas, epsilon);
         EjmlUnitTests.assertEquals(alpha1Expected, controller.alphas.get(0), epsilon);

         EjmlUnitTests.assertEquals(s2Expected, controller.getCostJacobian(), epsilon);
      }

      for (double time = finalTime1; time <= finalTime2; time += 0.01)
      {
         double localTime = time - finalTime1;
         controller.computeS2(time);

         CommonOps.scale(localTime, A2Expected, timeScaledDynamics);
         matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);

         DenseMatrix64F s2Expected = new DenseMatrix64F(6, 1);
         CommonOps.mult(matrixExponential, alpha2Expected, s2Expected);
         CommonOps.addEquals(s2Expected, MathTools.pow(localTime, 0), beta21Expected);
         CommonOps.addEquals(s2Expected, MathTools.pow(localTime, 1), beta22Expected);

         EjmlUnitTests.assertEquals(alpha2Expected, controller.alphas.get(1), epsilon);

         EjmlUnitTests.assertEquals(s2Expected, controller.getCostJacobian(), epsilon);
      }

      controller.computeS2(finalTime2);

      // should be zero at tf
      DenseMatrix64F zeroMatrix = new DenseMatrix64F(6, 1);
      EjmlUnitTests.assertEquals(zeroMatrix, controller.getCostJacobian(), epsilon);
   }
}
