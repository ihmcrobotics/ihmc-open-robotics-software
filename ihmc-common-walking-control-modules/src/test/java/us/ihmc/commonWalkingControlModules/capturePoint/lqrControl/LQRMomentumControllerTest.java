package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import org.ejml.ops.EjmlUnitTests;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SimpleCoMTrajectoryPlanner;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.matrixlib.NativeCommonOps;
import us.ihmc.robotics.linearAlgebra.MatrixExponentialCalculator;
import us.ihmc.robotics.math.trajectories.Trajectory3D;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public class LQRMomentumControllerTest
{
   private static final double omega = 3.0;
   private static final double epsilon = 1e-10;

   @Test
   public void testComputingS1()
   {
      LQRMomentumController controller = new LQRMomentumController(() -> omega);

      Point3D vrpStart = new Point3D(0.0, 0.0, 1.0);
      Point3D vrpEnd = new Point3D(1.0, 0.5, 1.0);
      Trajectory3D vrpTrajectory = new Trajectory3D(4);
      vrpTrajectory.setLinear(0.0, 1.0, vrpStart, vrpEnd);
      List<Trajectory3D> trajectories = new ArrayList<>();
      trajectories.add(vrpTrajectory);

      controller.setVRPTrajectory(trajectories);

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

      DExpected.set(0, 0, -1.0 / MathTools.square(omega));
      DExpected.set(1, 1, -1.0 / MathTools.square(omega));
      DExpected.set(2, 2, -1.0 / MathTools.square(omega));

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
      LQRMomentumController controller = new LQRMomentumController(() -> omega);
      controller.computeS1();

      Point3D vrpStart = new Point3D(0.0, 0.0, 1.0);
      Point3D vrpEnd = new Point3D(1.0, 0.5, 1.0);
      Trajectory3D vrpTrajectory = new Trajectory3D(4);
      double finalTime = 1.5;
      vrpTrajectory.setLinear(0.0, finalTime, vrpStart, vrpEnd);
      List<Trajectory3D> trajectories = new ArrayList<>();
      trajectories.add(vrpTrajectory);

      DenseMatrix64F finalPosition = new DenseMatrix64F(3, 1);
      vrpEnd.get(finalPosition);

      controller.setVRPTrajectory(trajectories);

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
      DenseMatrix64F k2Method1 = new DenseMatrix64F(3, 1);
      DenseMatrix64F k2Method2 = new DenseMatrix64F(3, 1);
      DenseMatrix64F yd = new DenseMatrix64F(3, 1);

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
      CommonOps.subtractEquals(coefficients, finalPosition);
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

      DenseMatrix64F finalY = new DenseMatrix64F(3, 1);
      vrpEnd.get(finalY);

      for (double time = 0.0; time <= finalTime; time += 0.01)
      {
         controller.computeS2(time);

         CommonOps.scale(time, A2Expected, timeScaledDynamics);
         matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);

         DenseMatrix64F s2Expected = new DenseMatrix64F(6, 1);
         CommonOps.mult(matrixExponential, alphaExpected, s2Expected);
         CommonOps.addEquals(s2Expected, MathTools.pow(time, 0), beta1Expected);
         CommonOps.addEquals(s2Expected, MathTools.pow(time, 1), beta2Expected);

         DenseMatrix64F gamma1ExpectedAlt = new DenseMatrix64F(3, 1);
         DenseMatrix64F gamma2ExpectedAlt = new DenseMatrix64F(3, 1);

         vrpTrajectory.getCoefficients(0, coefficients);
         CommonOps.subtractEquals(coefficients, finalPosition);
         CommonOps.mult(R1InvDQ, coefficients, gamma1ExpectedAlt);
         CommonOps.multAdd(-0.5, R1InvBTrans, beta1Expected, gamma1ExpectedAlt);

         vrpTrajectory.getCoefficients(1, coefficients);
         CommonOps.mult(R1InvDQ, coefficients, gamma2ExpectedAlt);
         CommonOps.multAdd(-0.5, R1InvBTrans, beta2Expected, gamma2ExpectedAlt);

         EjmlUnitTests.assertEquals(timeScaledDynamics, controller.timeScaledDynamics, epsilon);
         EjmlUnitTests.assertEquals(matrixExponential, controller.exponential, epsilon);
         EjmlUnitTests.assertEquals(betaSum, controller.summedBetas, epsilon);
         EjmlUnitTests.assertEquals(alphaExpected, controller.alphas.get(0), epsilon);
         EjmlUnitTests.assertEquals(beta1Expected, controller.betas.get(0).get(0), epsilon);
         EjmlUnitTests.assertEquals(beta2Expected, controller.betas.get(0).get(1), epsilon);
         EjmlUnitTests.assertEquals(gamma1ExpectedAlt, gamma1Expected, epsilon);
         EjmlUnitTests.assertEquals(gamma2ExpectedAlt, gamma2Expected, epsilon);
         EjmlUnitTests.assertEquals(gamma1Expected, controller.gammas.get(0).get(0), epsilon);
         EjmlUnitTests.assertEquals(gamma2Expected, controller.gammas.get(0).get(1), epsilon);

         EjmlUnitTests.assertEquals(s2Expected, controller.getCostJacobian(), epsilon);

         vrpTrajectory.compute(time);
         vrpTrajectory.getPosition().get(yd);
         CommonOps.subtractEquals(yd, finalY);

         CommonOps.mult(-0.5, R1InvBTrans, s2Expected, k2Method1);
         CommonOps.multAdd(R1InvDQ, yd, k2Method1);

         tempMatrix.reshape(3, 6);
         CommonOps.mult(-0.5, R1InvBTrans, matrixExponential, tempMatrix);
         CommonOps.mult(tempMatrix, controller.alphas.get(0), k2Method2);

         CommonOps.addEquals(k2Method2, MathTools.pow(time, 0), controller.gammas.get(0).get(0));
         CommonOps.addEquals(k2Method2, MathTools.pow(time, 1), controller.gammas.get(0).get(1));

         DenseMatrix64F K1Expected = new DenseMatrix64F(3, 6);
         CommonOps.mult(-1.0, controller.R1Inverse, NBExpected, K1Expected);

         EjmlUnitTests.assertEquals(k2Method1, k2Method2, epsilon);
         MatrixTestTools.assertMatrixEquals(k2Method1, controller.k2, epsilon);
         MatrixTestTools.assertMatrixEquals(K1Expected, controller.K1, epsilon);
      }

      controller.computeS2(finalTime);

      // should be zero at tf
      DenseMatrix64F zeroMatrix = new DenseMatrix64F(6, 1);
      EjmlUnitTests.assertEquals(zeroMatrix, controller.getCostJacobian(), epsilon);
   }

   @Test
   public void testComputingS2FromSingleCubicTrajectory()
   {
      LQRMomentumController controller = new LQRMomentumController(() -> omega);

      Point3D vrpStart = new Point3D(0.0, 0.0, 1.0);
      Point3D vrpEnd = new Point3D(1.0, 0.5, 1.0);
      Trajectory3D vrpTrajectory = new Trajectory3D(4);
      double finalTime = 1.5;
      vrpTrajectory.setCubic(0.0, finalTime, vrpStart, vrpEnd);
      List<Trajectory3D> trajectories = new ArrayList<>();
      trajectories.add(vrpTrajectory);

      DenseMatrix64F finalPosition = new DenseMatrix64F(3, 1);
      vrpEnd.get(finalPosition);

      controller.setVRPTrajectory(trajectories);

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

      DenseMatrix64F K1Expected = new DenseMatrix64F(3, 6);
      CommonOps.mult(-1.0, controller.R1Inverse, NBExpected, K1Expected);

      MatrixTestTools.assertMatrixEquals(K1Expected, controller.K1, epsilon);


      DenseMatrix64F A2InverseB2 = new DenseMatrix64F(6, 3);
      DenseMatrix64F R1InvDQ = new DenseMatrix64F(3, 3);
      DenseMatrix64F DQ = new DenseMatrix64F(3, 3);
      DenseMatrix64F R1InvBTrans = new DenseMatrix64F(3, 6);
      DenseMatrix64F k2Method1 = new DenseMatrix64F(3, 1);
      DenseMatrix64F k2Method2 = new DenseMatrix64F(3, 1);
      DenseMatrix64F yd = new DenseMatrix64F(3, 1);

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
      CommonOps.subtractEquals(coefficients, finalPosition);
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

      DenseMatrix64F finalY = new DenseMatrix64F(3, 1);
      vrpEnd.get(finalY);

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

         vrpTrajectory.compute(time);
         vrpTrajectory.getPosition().get(yd);
         CommonOps.subtractEquals(yd, finalY);

         CommonOps.mult(-0.5, R1InvBTrans, s2Expected, k2Method1);
         CommonOps.multAdd(R1InvDQ, yd, k2Method1);

         tempMatrix.reshape(3, 6);
         CommonOps.mult(-0.5, R1InvBTrans, matrixExponential, tempMatrix);
         CommonOps.mult(tempMatrix, controller.alphas.get(0), k2Method2);

         CommonOps.addEquals(k2Method2, MathTools.pow(time, 0), controller.gammas.get(0).get(0));
         CommonOps.addEquals(k2Method2, MathTools.pow(time, 1), controller.gammas.get(0).get(1));
         CommonOps.addEquals(k2Method2, MathTools.pow(time, 2), controller.gammas.get(0).get(2));
         CommonOps.addEquals(k2Method2, MathTools.pow(time, 3), controller.gammas.get(0).get(3));

         K1Expected = new DenseMatrix64F(3, 6);
         CommonOps.mult(-1.0, controller.R1Inverse, NBExpected, K1Expected);

         EjmlUnitTests.assertEquals(k2Method1, k2Method2, epsilon);
         MatrixTestTools.assertMatrixEquals(k2Method2, controller.k2, epsilon);
         MatrixTestTools.assertMatrixEquals(k2Method1, controller.k2, epsilon);
         MatrixTestTools.assertMatrixEquals(K1Expected, controller.K1, epsilon);
      }

      controller.computeS2(finalTime);

      // should be zero at tf
      DenseMatrix64F zeroMatrix = new DenseMatrix64F(6, 1);
      EjmlUnitTests.assertEquals(zeroMatrix, controller.getCostJacobian(), epsilon);
   }

   @Test
   public void testComputingS2FromTwoLinearTrajectories()
   {
      LQRMomentumController controller = new LQRMomentumController(() -> omega);

      Point3D vrpStart = new Point3D(0.0, 0.0, 1.0);
      Point3D vrpMiddle = new Point3D(0.6, 0.75, 1.0);
      Point3D vrpEnd = new Point3D(1.0, 0.5, 1.0);
      Trajectory3D vrpTrajectory1 = new Trajectory3D(4);
      Trajectory3D vrpTrajectory2 = new Trajectory3D(4);
      double finalTime1 = 1.5;
      double finalTime2 = 3.1;
      vrpTrajectory1.setLinear(0.0, finalTime1, vrpStart, vrpMiddle);
      vrpTrajectory2.setLinear(0.0, finalTime2 - finalTime1, vrpMiddle, vrpEnd);
      List<Trajectory3D> trajectories = new ArrayList<>();
      trajectories.add(vrpTrajectory1);
      trajectories.add(vrpTrajectory2);

      DenseMatrix64F finalPosition = new DenseMatrix64F(3, 1);
      vrpEnd.get(finalPosition);

      controller.setVRPTrajectory(trajectories);

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
      DenseMatrix64F R1InvDQInverse = new DenseMatrix64F(3, 3);
      DenseMatrix64F DQ = new DenseMatrix64F(3, 3);
      DenseMatrix64F R1InvBTrans = new DenseMatrix64F(3, 6);
      DenseMatrix64F k2Method1 = new DenseMatrix64F(3, 1);
      DenseMatrix64F k2Method2 = new DenseMatrix64F(3, 1);
      DenseMatrix64F yd = new DenseMatrix64F(3, 1);

      CommonOps.mult(A2InverseExpected, B2Expected, A2InverseB2);
      CommonOps.mult(controller.D, controller.Q, DQ);
      CommonOps.mult(controller.R1Inverse, DQ, R1InvDQ);
      CommonOps.multTransB(controller.R1Inverse, controller.B, R1InvBTrans);
      solver.setA(R1InvDQ);
      solver.invert(R1InvDQInverse);

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
      DenseMatrix64F constructedCoefficient11 = new DenseMatrix64F(3, 1);
      DenseMatrix64F constructedCoefficient12 = new DenseMatrix64F(3, 1);
      DenseMatrix64F constructedCoefficient21 = new DenseMatrix64F(3, 1);
      DenseMatrix64F constructedCoefficient22 = new DenseMatrix64F(3, 1);


      vrpTrajectory2.getCoefficients(1, coefficients);
      CommonOps.mult(-1, A2InverseB2, coefficients, beta22Expected);

      CommonOps.mult(R1InvDQ, coefficients, gamma22Expected);
      CommonOps.multAdd(-0.5, R1InvBTrans, beta22Expected, gamma22Expected);

      vrpTrajectory2.getCoefficients(0, coefficients);
      CommonOps.subtractEquals(coefficients, finalPosition);
      CommonOps.mult(A2InverseExpected, beta22Expected, beta21Expected);
      CommonOps.multAdd(-1.0, A2InverseB2, coefficients, beta21Expected);

      CommonOps.mult(R1InvDQ, coefficients, gamma21Expected);
      CommonOps.multAdd(-0.5, R1InvBTrans, beta21Expected, gamma21Expected);

      tempMatrix.set(gamma21Expected);
      CommonOps.multAdd(0.5, R1InvBTrans, beta21Expected, tempMatrix);
      CommonOps.mult(R1InvDQInverse, tempMatrix, constructedCoefficient21);

      tempMatrix.set(gamma22Expected);
      CommonOps.multAdd(0.5, R1InvBTrans, beta22Expected, tempMatrix);
      CommonOps.mult(R1InvDQInverse, tempMatrix, constructedCoefficient22);

      EjmlUnitTests.assertEquals(beta21Expected, controller.betas.get(1).get(0), epsilon);
      EjmlUnitTests.assertEquals(beta22Expected, controller.betas.get(1).get(1), epsilon);
      EjmlUnitTests.assertEquals(gamma21Expected, controller.gammas.get(1).get(0), epsilon);
      EjmlUnitTests.assertEquals(gamma22Expected, controller.gammas.get(1).get(1), epsilon);

      controller.relativeVRPTrajectories.get(1).getCoefficients(0, coefficients);
      MatrixTestTools.assertMatrixEquals(constructedCoefficient21, coefficients, epsilon);
      controller.relativeVRPTrajectories.get(1).getCoefficients(1, coefficients);
      MatrixTestTools.assertMatrixEquals(constructedCoefficient22, coefficients, epsilon);



      vrpTrajectory1.getCoefficients(1, coefficients);
      CommonOps.mult(-1, A2InverseB2, coefficients, beta12Expected);

      CommonOps.mult(R1InvDQ, coefficients, gamma12Expected);
      CommonOps.multAdd(-0.5, R1InvBTrans, beta12Expected, gamma12Expected);


      vrpTrajectory1.getCoefficients(0, coefficients);
      CommonOps.subtractEquals(coefficients, finalPosition);
      CommonOps.mult(A2InverseExpected, beta12Expected, beta11Expected);
      CommonOps.multAdd(-1.0, A2InverseB2, coefficients, beta11Expected);

      CommonOps.mult(R1InvDQ, coefficients, gamma11Expected);
      CommonOps.multAdd(-0.5, R1InvBTrans, beta11Expected, gamma11Expected);


      tempMatrix.set(gamma11Expected);
      CommonOps.multAdd(0.5, R1InvBTrans, beta11Expected, tempMatrix);
      CommonOps.mult(R1InvDQInverse, tempMatrix, constructedCoefficient11);

      tempMatrix.set(gamma12Expected);
      CommonOps.multAdd(0.5, R1InvBTrans, beta12Expected, tempMatrix);
      CommonOps.mult(R1InvDQInverse, tempMatrix, constructedCoefficient12);


      EjmlUnitTests.assertEquals(beta11Expected, controller.betas.get(0).get(0), epsilon);
      EjmlUnitTests.assertEquals(beta12Expected, controller.betas.get(0).get(1), epsilon);
      EjmlUnitTests.assertEquals(gamma11Expected, controller.gammas.get(0).get(0), epsilon);
      EjmlUnitTests.assertEquals(gamma12Expected, controller.gammas.get(0).get(1), epsilon);

      controller.relativeVRPTrajectories.get(0).getCoefficients(0, coefficients);
      MatrixTestTools.assertMatrixEquals(constructedCoefficient11, coefficients, epsilon);
      controller.relativeVRPTrajectories.get(0).getCoefficients(1, coefficients);
      MatrixTestTools.assertMatrixEquals(constructedCoefficient12, coefficients, epsilon);


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



      /*
      // check for continuity
       */
      DenseMatrix64F relativeMiddleVRP = new DenseMatrix64F(3, 1);
      vrpMiddle.get(relativeMiddleVRP);
      CommonOps.subtractEquals(relativeMiddleVRP, finalPosition);

      DenseMatrix64F constructedPositionEndOf1 = new DenseMatrix64F(3, 1);
      DenseMatrix64F constructedPositionStartOf2 = new DenseMatrix64F(3, 1);
      DenseMatrix64F positionEndOf1 = new DenseMatrix64F(3, 1);
      DenseMatrix64F positionStartOf2 = new DenseMatrix64F(3, 1);

      controller.relativeVRPTrajectories.get(0).getCoefficients(0, coefficients);
      CommonOps.addEquals(constructedPositionEndOf1, MathTools.pow(finalTime1, 0), coefficients);
      controller.relativeVRPTrajectories.get(0).getCoefficients(1, coefficients);
      CommonOps.addEquals(constructedPositionEndOf1, MathTools.pow(finalTime1, 1), coefficients);

      controller.relativeVRPTrajectories.get(0).compute(finalTime1);
      controller.relativeVRPTrajectories.get(1).compute(0.0);
      controller.relativeVRPTrajectories.get(0).getPosition().get(positionEndOf1);
      controller.relativeVRPTrajectories.get(1).getPosition().get(positionStartOf2);


      controller.relativeVRPTrajectories.get(1).getCoefficients(0, coefficients);
      CommonOps.addEquals(constructedPositionStartOf2, MathTools.pow(0, 0), coefficients);
      controller.relativeVRPTrajectories.get(1).getCoefficients(1, coefficients);
      CommonOps.addEquals(constructedPositionStartOf2, MathTools.pow(0, 1), coefficients);

      DenseMatrix64F s2EndOf1 = new DenseMatrix64F(6, 1);
      DenseMatrix64F s2StartOf2 = new DenseMatrix64F(6, 1);

      DenseMatrix64F k2EndOf1Method1 = new DenseMatrix64F(3, 1);
      DenseMatrix64F k2EndOf1Method2 = new DenseMatrix64F(3, 1);
      DenseMatrix64F k2StartOf2Method1 = new DenseMatrix64F(3, 1);
      DenseMatrix64F k2StartOf2Method2 = new DenseMatrix64F(3, 1);

      DenseMatrix64F actualS2 = new DenseMatrix64F(6, 1);
      DenseMatrix64F actualk2 = new DenseMatrix64F(6, 1);

      controller.computeS2(finalTime1);
      actualk2.set(controller.k2);
      actualS2.set(controller.getCostJacobian());

      // s2 at end of 1
      CommonOps.scale(finalTime1, A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      CommonOps.mult(matrixExponential, controller.alphas.get(0), s2EndOf1);
      CommonOps.addEquals(s2EndOf1, MathTools.pow(finalTime1, 0), controller.betas.get(0).get(0));
      CommonOps.addEquals(s2EndOf1, MathTools.pow(finalTime1, 1), controller.betas.get(0).get(1));

      // k2 method 1 end of 1
      CommonOps.mult(-0.5, R1InvBTrans, s2EndOf1, k2EndOf1Method1);
      CommonOps.multAdd(R1InvDQ, relativeMiddleVRP, k2EndOf1Method1);

      // k2 method 2 end of 1
      CommonOps.scale(finalTime1, A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      tempMatrix.reshape(3, 6);
      CommonOps.mult(-0.5, R1InvBTrans, matrixExponential, tempMatrix);
      CommonOps.mult(tempMatrix, controller.alphas.get(0), k2EndOf1Method2);
      CommonOps.addEquals(k2EndOf1Method2, MathTools.pow(finalTime1, 0), controller.gammas.get(0).get(0));
      CommonOps.addEquals(k2EndOf1Method2, MathTools.pow(finalTime1, 1), controller.gammas.get(0).get(1));

      // s2 at start of 1
      CommonOps.scale(0, A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      CommonOps.mult(matrixExponential, controller.alphas.get(1), s2StartOf2);
      CommonOps.addEquals(s2StartOf2, MathTools.pow(0, 0), controller.betas.get(1).get(0));
      CommonOps.addEquals(s2StartOf2, MathTools.pow(0, 1), controller.betas.get(1).get(1));

      // k2 method 1 start of 2
      CommonOps.mult(-0.5, R1InvBTrans, s2StartOf2, k2StartOf2Method1);
      CommonOps.multAdd(R1InvDQ, relativeMiddleVRP, k2StartOf2Method1);

      // k2 method 2 start of 2
      CommonOps.scale(0.0, A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      CommonOps.mult(-0.5, R1InvBTrans, matrixExponential, tempMatrix);
      CommonOps.mult(tempMatrix, controller.alphas.get(1), k2StartOf2Method2);
      CommonOps.addEquals(k2StartOf2Method2, MathTools.pow(0, 0), controller.gammas.get(1).get(0));
      CommonOps.addEquals(k2StartOf2Method2, MathTools.pow(0, 1), controller.gammas.get(1).get(1));


      EjmlUnitTests.assertEquals(s2EndOf1, s2StartOf2, epsilon);
      EjmlUnitTests.assertEquals(actualS2, s2StartOf2, epsilon);
      MatrixTestTools.assertMatrixEquals(relativeMiddleVRP, positionEndOf1, epsilon);
      MatrixTestTools.assertMatrixEquals(relativeMiddleVRP, positionStartOf2, epsilon);
      MatrixTestTools.assertMatrixEquals(relativeMiddleVRP, constructedPositionEndOf1, epsilon);
      MatrixTestTools.assertMatrixEquals(relativeMiddleVRP, constructedPositionStartOf2, epsilon);
      MatrixTestTools.assertMatrixEquals(k2StartOf2Method1, actualk2, epsilon);
      MatrixTestTools.assertMatrixEquals(k2StartOf2Method1, k2StartOf2Method2, epsilon);
      MatrixTestTools.assertMatrixEquals(k2EndOf1Method1, k2EndOf1Method2, epsilon);
      MatrixTestTools.assertMatrixEquals(k2EndOf1Method1, k2StartOf2Method1, epsilon);
      MatrixTestTools.assertMatrixEquals(k2EndOf1Method2, k2StartOf2Method2, epsilon);

      // check the actual trajectory for every time step of the first trajectory
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

         vrpTrajectory1.compute(time);
         vrpTrajectory1.getPosition().get(yd);
         CommonOps.subtractEquals(yd, finalPosition);

         CommonOps.mult(-0.5, R1InvBTrans, s2Expected, k2Method1);
         CommonOps.multAdd(R1InvDQ, yd, k2Method1);

         tempMatrix.reshape(3, 6);
         CommonOps.mult(-0.5, R1InvBTrans, matrixExponential, tempMatrix);
         CommonOps.mult(tempMatrix, controller.alphas.get(0), k2Method2);

         CommonOps.addEquals(k2Method2, MathTools.pow(time, 0), controller.gammas.get(0).get(0));
         CommonOps.addEquals(k2Method2, MathTools.pow(time, 1), controller.gammas.get(0).get(1));


         DenseMatrix64F K1Expected = new DenseMatrix64F(3, 6);
         CommonOps.mult(-1.0, controller.R1Inverse, NBExpected, K1Expected);

         EjmlUnitTests.assertEquals(k2Method1, k2Method2, epsilon);
         MatrixTestTools.assertMatrixEquals(k2Method1, controller.k2, epsilon);
         MatrixTestTools.assertMatrixEquals(k2Method2, controller.k2, epsilon);
         MatrixTestTools.assertMatrixEquals(K1Expected, controller.K1, epsilon);
      }

      // check the actual trajectory for every time step of the second trajectory
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

         MatrixTestTools.assertMatrixEquals("Beta 1 calculation failed at time " + time, beta21Expected, controller.betas.get(1).get(0), epsilon);
         MatrixTestTools.assertMatrixEquals("Beta 2 calculation failed at time " + time, beta22Expected, controller.betas.get(1).get(1), epsilon);
         MatrixTestTools.assertMatrixEquals("S2 calculation failed at time " + time, s2Expected, controller.getCostJacobian(), epsilon);

         // validate the gammas
         DenseMatrix64F gamma21ExpectedAlt = new DenseMatrix64F(3, 1);
         DenseMatrix64F gamma22ExpectedAlt = new DenseMatrix64F(3, 1);

         vrpTrajectory2.getCoefficients(0, coefficients);
         CommonOps.subtractEquals(coefficients, finalPosition);
         CommonOps.mult(R1InvDQ, coefficients, gamma21ExpectedAlt);
         CommonOps.multAdd(-0.5, R1InvBTrans, beta21Expected, gamma21ExpectedAlt);

         vrpTrajectory2.getCoefficients(1, coefficients);
         CommonOps.mult(R1InvDQ, coefficients, gamma22ExpectedAlt);
         CommonOps.multAdd(-0.5, R1InvBTrans, beta22Expected, gamma22ExpectedAlt);

         MatrixTestTools.assertMatrixEquals(gamma21ExpectedAlt, controller.gammas.get(1).get(0), epsilon);
         MatrixTestTools.assertMatrixEquals(gamma22ExpectedAlt, controller.gammas.get(1).get(1), epsilon);

         MatrixTestTools.assertMatrixEquals(gamma21ExpectedAlt, gamma21Expected, epsilon);
         MatrixTestTools.assertMatrixEquals(gamma22ExpectedAlt, gamma22Expected, epsilon);

         // validate the feedback solution
         vrpTrajectory2.compute(localTime);
         vrpTrajectory2.getPosition().get(yd);
         CommonOps.subtractEquals(yd, finalPosition);

         CommonOps.mult(-0.5, R1InvBTrans, s2Expected, k2Method1);
         CommonOps.multAdd(R1InvDQ, yd, k2Method1);


         tempMatrix.reshape(3, 6);
         CommonOps.scale(localTime, A2Expected, timeScaledDynamics);
         matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
         CommonOps.mult(-0.5, R1InvBTrans, matrixExponential, tempMatrix);
         CommonOps.mult(tempMatrix, controller.alphas.get(1), k2Method2);
//         MatrixTestTools.assertMatrixEquals(matrixExponential, controller.exponential, epsilon);

         CommonOps.addEquals(k2Method2, MathTools.pow(localTime, 0), controller.gammas.get(1).get(0));
         CommonOps.addEquals(k2Method2, MathTools.pow(localTime, 1), controller.gammas.get(1).get(1));

         DenseMatrix64F K1Expected = new DenseMatrix64F(3, 6);
         CommonOps.mult(-1.0, controller.R1Inverse, NBExpected, K1Expected);

         MatrixTestTools.assertMatrixEquals(k2Method1, k2Method2, epsilon);
         MatrixTestTools.assertMatrixEquals(k2Method1, controller.k2, epsilon);
         MatrixTestTools.assertMatrixEquals(K1Expected, controller.K1, epsilon);
      }

      controller.computeS2(finalTime2);

      // should be zero at tf
      DenseMatrix64F zeroMatrix = new DenseMatrix64F(6, 1);
      EjmlUnitTests.assertEquals(zeroMatrix, controller.getCostJacobian(), epsilon);
   }

   @Test
   public void testComputingS2FromThreeCubicTrajectories()
   {
      LQRMomentumController controller = new LQRMomentumController(() -> omega);

      Point3D vrpStart = new Point3D(0.0, 0.0, 1.0);
      Point3D vrpMiddle = new Point3D(0.6, 0.75, 0.87);
      Point3D vrpMiddle2 = new Point3D(0.79, 0.88, 0.95);
      Point3D vrpEnd = new Point3D(1.0, 0.5, 1.0);
      Trajectory3D vrpTrajectory1 = new Trajectory3D(4);
      Trajectory3D vrpTrajectory2 = new Trajectory3D(4);
      Trajectory3D vrpTrajectory3 = new Trajectory3D(4);
      double finalTime1 = 1.5;
      double finalTime2 = 3.1;
      double finalTime3 = 3.97;
      vrpTrajectory1.setCubic(0.0, finalTime1, vrpStart, vrpMiddle);
      vrpTrajectory2.setCubic(0.0, finalTime2 - finalTime1, vrpMiddle, vrpMiddle2);
      vrpTrajectory3.setCubic(0.0, finalTime3 - finalTime2, vrpMiddle2, vrpEnd);
      List<Trajectory3D> trajectories = new ArrayList<>();
      trajectories.add(vrpTrajectory1);
      trajectories.add(vrpTrajectory2);
      trajectories.add(vrpTrajectory3);

      DenseMatrix64F finalPosition = new DenseMatrix64F(3, 1);
      vrpEnd.get(finalPosition);

      controller.setVRPTrajectory(trajectories);

      // double up on the computation to make sure things get zeroed properly between calls
      controller.computeS1();
      controller.computeS1();
      controller.computeS2Parameters();
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
      DenseMatrix64F k2Method1 = new DenseMatrix64F(3, 1);
      DenseMatrix64F k2Method2 = new DenseMatrix64F(3, 1);
      DenseMatrix64F yd = new DenseMatrix64F(3, 1);

      CommonOps.mult(A2InverseExpected, B2Expected, A2InverseB2);
      CommonOps.mult(controller.D, controller.Q, DQ);
      CommonOps.mult(controller.R1Inverse, DQ, R1InvDQ);
      CommonOps.multTransB(controller.R1Inverse, controller.B, R1InvBTrans);

      assertEquals(3, controller.alphas.size());
      assertEquals(3, controller.betas.size());
      assertEquals(3, controller.gammas.size());
      assertEquals(4, controller.betas.get(0).size());
      assertEquals(4, controller.betas.get(1).size());
      assertEquals(4, controller.betas.get(2).size());
      assertEquals(4, controller.gammas.get(0).size());
      assertEquals(4, controller.gammas.get(1).size());
      assertEquals(4, controller.gammas.get(2).size());

      DenseMatrix64F beta11Expected = new DenseMatrix64F(6, 1);
      DenseMatrix64F beta12Expected = new DenseMatrix64F(6, 1);
      DenseMatrix64F beta13Expected = new DenseMatrix64F(6, 1);
      DenseMatrix64F beta14Expected = new DenseMatrix64F(6, 1);
      DenseMatrix64F beta21Expected = new DenseMatrix64F(6, 1);
      DenseMatrix64F beta22Expected = new DenseMatrix64F(6, 1);
      DenseMatrix64F beta23Expected = new DenseMatrix64F(6, 1);
      DenseMatrix64F beta24Expected = new DenseMatrix64F(6, 1);
      DenseMatrix64F beta31Expected = new DenseMatrix64F(6, 1);
      DenseMatrix64F beta32Expected = new DenseMatrix64F(6, 1);
      DenseMatrix64F beta33Expected = new DenseMatrix64F(6, 1);
      DenseMatrix64F beta34Expected = new DenseMatrix64F(6, 1);
      DenseMatrix64F gamma11Expected = new DenseMatrix64F(3, 1);
      DenseMatrix64F gamma12Expected = new DenseMatrix64F(3, 1);
      DenseMatrix64F gamma13Expected = new DenseMatrix64F(3, 1);
      DenseMatrix64F gamma14Expected = new DenseMatrix64F(3, 1);
      DenseMatrix64F gamma21Expected = new DenseMatrix64F(3, 1);
      DenseMatrix64F gamma22Expected = new DenseMatrix64F(3, 1);
      DenseMatrix64F gamma23Expected = new DenseMatrix64F(3, 1);
      DenseMatrix64F gamma24Expected = new DenseMatrix64F(3, 1);
      DenseMatrix64F gamma31Expected = new DenseMatrix64F(3, 1);
      DenseMatrix64F gamma32Expected = new DenseMatrix64F(3, 1);
      DenseMatrix64F gamma33Expected = new DenseMatrix64F(3, 1);
      DenseMatrix64F gamma34Expected = new DenseMatrix64F(3, 1);
      DenseMatrix64F coefficients = new DenseMatrix64F(3, 1);

      vrpTrajectory3.getCoefficients(3, coefficients);
      CommonOps.mult(-1, A2InverseB2, coefficients, beta34Expected);

      CommonOps.mult(R1InvDQ, coefficients, gamma34Expected);
      CommonOps.multAdd(-0.5, R1InvBTrans, beta34Expected, gamma34Expected);

      vrpTrajectory3.getCoefficients(2, coefficients);
      CommonOps.mult(3, A2InverseExpected, beta34Expected, beta33Expected);
      CommonOps.multAdd(-1.0, A2InverseB2, coefficients, beta33Expected);

      CommonOps.mult(R1InvDQ, coefficients, gamma33Expected);
      CommonOps.multAdd(-0.5, R1InvBTrans, beta33Expected, gamma33Expected);

      vrpTrajectory3.getCoefficients(1, coefficients);
      CommonOps.mult(2, A2InverseExpected, beta33Expected, beta32Expected);
      CommonOps.multAdd(-1.0, A2InverseB2, coefficients, beta32Expected);

      CommonOps.mult(R1InvDQ, coefficients, gamma32Expected);
      CommonOps.multAdd(-0.5, R1InvBTrans, beta32Expected, gamma32Expected);

      vrpTrajectory3.getCoefficients(0, coefficients);
      CommonOps.subtractEquals(coefficients, finalPosition);
      CommonOps.mult(A2InverseExpected, beta32Expected, beta31Expected);
      CommonOps.multAdd(-1.0, A2InverseB2, coefficients, beta31Expected);

      CommonOps.mult(R1InvDQ, coefficients, gamma31Expected);
      CommonOps.multAdd(-0.5, R1InvBTrans, beta31Expected, gamma31Expected);

      EjmlUnitTests.assertEquals(beta34Expected, controller.betas.get(2).get(3), epsilon);
      EjmlUnitTests.assertEquals(beta33Expected, controller.betas.get(2).get(2), epsilon);
      EjmlUnitTests.assertEquals(beta32Expected, controller.betas.get(2).get(1), epsilon);
      EjmlUnitTests.assertEquals(beta31Expected, controller.betas.get(2).get(0), epsilon);
      EjmlUnitTests.assertEquals(gamma34Expected, controller.gammas.get(2).get(3), epsilon);
      EjmlUnitTests.assertEquals(gamma33Expected, controller.gammas.get(2).get(2), epsilon);
      EjmlUnitTests.assertEquals(gamma32Expected, controller.gammas.get(2).get(1), epsilon);
      EjmlUnitTests.assertEquals(gamma31Expected, controller.gammas.get(2).get(0), epsilon);


      vrpTrajectory2.getCoefficients(3, coefficients);
      CommonOps.mult(-1, A2InverseB2, coefficients, beta24Expected);

      CommonOps.mult(R1InvDQ, coefficients, gamma24Expected);
      CommonOps.multAdd(-0.5, R1InvBTrans, beta24Expected, gamma24Expected);

      vrpTrajectory2.getCoefficients(2, coefficients);
      CommonOps.mult(3, A2InverseExpected, beta24Expected, beta23Expected);
      CommonOps.multAdd(-1.0, A2InverseB2, coefficients, beta23Expected);

      CommonOps.mult(R1InvDQ, coefficients, gamma23Expected);
      CommonOps.multAdd(-0.5, R1InvBTrans, beta23Expected, gamma23Expected);

      vrpTrajectory2.getCoefficients(1, coefficients);
      CommonOps.mult(2, A2InverseExpected, beta23Expected, beta22Expected);
      CommonOps.multAdd(-1.0, A2InverseB2, coefficients, beta22Expected);

      CommonOps.mult(R1InvDQ, coefficients, gamma22Expected);
      CommonOps.multAdd(-0.5, R1InvBTrans, beta22Expected, gamma22Expected);

      vrpTrajectory2.getCoefficients(0, coefficients);
      CommonOps.subtractEquals(coefficients, finalPosition);
      CommonOps.mult(A2InverseExpected, beta22Expected, beta21Expected);
      CommonOps.multAdd(-1.0, A2InverseB2, coefficients, beta21Expected);

      CommonOps.mult(R1InvDQ, coefficients, gamma21Expected);
      CommonOps.multAdd(-0.5, R1InvBTrans, beta21Expected, gamma21Expected);

      EjmlUnitTests.assertEquals(beta24Expected, controller.betas.get(1).get(3), epsilon);
      EjmlUnitTests.assertEquals(beta23Expected, controller.betas.get(1).get(2), epsilon);
      EjmlUnitTests.assertEquals(beta22Expected, controller.betas.get(1).get(1), epsilon);
      EjmlUnitTests.assertEquals(beta21Expected, controller.betas.get(1).get(0), epsilon);
      EjmlUnitTests.assertEquals(gamma24Expected, controller.gammas.get(1).get(3), epsilon);
      EjmlUnitTests.assertEquals(gamma23Expected, controller.gammas.get(1).get(2), epsilon);
      EjmlUnitTests.assertEquals(gamma22Expected, controller.gammas.get(1).get(1), epsilon);
      EjmlUnitTests.assertEquals(gamma21Expected, controller.gammas.get(1).get(0), epsilon);

      vrpTrajectory1.getCoefficients(3, coefficients);
      CommonOps.mult(-1, A2InverseB2, coefficients, beta14Expected);

      CommonOps.mult(R1InvDQ, coefficients, gamma14Expected);
      CommonOps.multAdd(-0.5, R1InvBTrans, beta14Expected, gamma14Expected);

      vrpTrajectory1.getCoefficients(2, coefficients);
      CommonOps.mult(3, A2InverseExpected, beta14Expected, beta13Expected);
      CommonOps.multAdd(-1.0, A2InverseB2, coefficients, beta13Expected);

      CommonOps.mult(R1InvDQ, coefficients, gamma13Expected);
      CommonOps.multAdd(-0.5, R1InvBTrans, beta13Expected, gamma13Expected);

      vrpTrajectory1.getCoefficients(1, coefficients);
      CommonOps.mult(2, A2InverseExpected, beta13Expected, beta12Expected);
      CommonOps.multAdd(-1.0, A2InverseB2, coefficients, beta12Expected);

      CommonOps.mult(R1InvDQ, coefficients, gamma12Expected);
      CommonOps.multAdd(-0.5, R1InvBTrans, beta12Expected, gamma12Expected);

      vrpTrajectory1.getCoefficients(0, coefficients);
      CommonOps.subtractEquals(coefficients, finalPosition);
      CommonOps.mult(A2InverseExpected, beta12Expected, beta11Expected);
      CommonOps.multAdd(-1.0, A2InverseB2, coefficients, beta11Expected);

      CommonOps.mult(R1InvDQ, coefficients, gamma11Expected);
      CommonOps.multAdd(-0.5, R1InvBTrans, beta11Expected, gamma11Expected);

      EjmlUnitTests.assertEquals(beta14Expected, controller.betas.get(0).get(3), epsilon);
      EjmlUnitTests.assertEquals(gamma14Expected, controller.gammas.get(0).get(3), epsilon);
      EjmlUnitTests.assertEquals(beta13Expected, controller.betas.get(0).get(2), epsilon);
      EjmlUnitTests.assertEquals(gamma13Expected, controller.gammas.get(0).get(2), epsilon);
      EjmlUnitTests.assertEquals(beta12Expected, controller.betas.get(0).get(1), epsilon);
      EjmlUnitTests.assertEquals(gamma12Expected, controller.gammas.get(0).get(1), epsilon);
      EjmlUnitTests.assertEquals(beta11Expected, controller.betas.get(0).get(0), epsilon);
      EjmlUnitTests.assertEquals(gamma11Expected, controller.gammas.get(0).get(0), epsilon);

      MatrixExponentialCalculator matrixExponentialCalculator = new MatrixExponentialCalculator(6);

      DenseMatrix64F alpha3Expected = new DenseMatrix64F(6, 1);
      DenseMatrix64F alpha2Expected = new DenseMatrix64F(6, 1);
      DenseMatrix64F alpha1Expected = new DenseMatrix64F(6, 1);

      DenseMatrix64F timeScaledDynamics = new DenseMatrix64F(6, 6);
      DenseMatrix64F matrixExponential = new DenseMatrix64F(6, 6);
      DenseMatrix64F betaSum = new DenseMatrix64F(6, 1);

      CommonOps.scale((finalTime3 - finalTime2), A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      CommonOps.addEquals(betaSum, -1.0 * MathTools.pow(finalTime3 - finalTime2, 0), beta31Expected);
      CommonOps.addEquals(betaSum, -1.0 * MathTools.pow(finalTime3 - finalTime2, 1), beta32Expected);
      CommonOps.addEquals(betaSum, -1.0 * MathTools.pow(finalTime3 - finalTime2, 2), beta33Expected);
      CommonOps.addEquals(betaSum, -1.0 * MathTools.pow(finalTime3 - finalTime2, 3), beta34Expected);

      solver.setA(matrixExponential);
      solver.solve(betaSum, alpha3Expected);

      CommonOps.scale((finalTime2 - finalTime1), A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      betaSum.zero();
      CommonOps.addEquals(betaSum, alpha3Expected);
      CommonOps.addEquals(betaSum, beta31Expected);
      CommonOps.addEquals(betaSum, -1.0 * MathTools.pow(finalTime2 - finalTime1, 0), beta21Expected);
      CommonOps.addEquals(betaSum, -1.0 * MathTools.pow(finalTime2 - finalTime1, 1), beta22Expected);
      CommonOps.addEquals(betaSum, -1.0 * MathTools.pow(finalTime2 - finalTime1, 2), beta23Expected);
      CommonOps.addEquals(betaSum, -1.0 * MathTools.pow(finalTime2 - finalTime1, 3), beta24Expected);

      solver.setA(matrixExponential);
      solver.solve(betaSum, alpha2Expected);

      CommonOps.scale(finalTime1, A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      betaSum.zero();
      CommonOps.addEquals(betaSum, alpha2Expected);
      CommonOps.addEquals(betaSum, beta21Expected);
      CommonOps.addEquals(betaSum, -1.0 * MathTools.pow(finalTime1, 0), beta11Expected);
      CommonOps.addEquals(betaSum, -1.0 * MathTools.pow(finalTime1, 1), beta12Expected);
      CommonOps.addEquals(betaSum, -1.0 * MathTools.pow(finalTime1, 2), beta13Expected);
      CommonOps.addEquals(betaSum, -1.0 * MathTools.pow(finalTime1, 3), beta14Expected);

      solver.setA(matrixExponential);
      solver.solve(betaSum, alpha1Expected);

      EjmlUnitTests.assertEquals(alpha3Expected, controller.alphas.get(2), epsilon);
      EjmlUnitTests.assertEquals(alpha2Expected, controller.alphas.get(1), epsilon);
      EjmlUnitTests.assertEquals(alpha1Expected, controller.alphas.get(0), epsilon);



      // check for continuity

      DenseMatrix64F s2EndOf1 = new DenseMatrix64F(6, 1);
      DenseMatrix64F s2StartOf2 = new DenseMatrix64F(6, 1);
      DenseMatrix64F s2EndOf2 = new DenseMatrix64F(6, 1);
      DenseMatrix64F s2StartOf3 = new DenseMatrix64F(6, 1);

      CommonOps.scale(finalTime1, A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      CommonOps.mult(matrixExponential, controller.alphas.get(0), s2EndOf1);
      CommonOps.addEquals(s2EndOf1, MathTools.pow(finalTime1, 0), controller.betas.get(0).get(0));
      CommonOps.addEquals(s2EndOf1, MathTools.pow(finalTime1, 1), controller.betas.get(0).get(1));
      CommonOps.addEquals(s2EndOf1, MathTools.pow(finalTime1, 2), controller.betas.get(0).get(2));
      CommonOps.addEquals(s2EndOf1, MathTools.pow(finalTime1, 3), controller.betas.get(0).get(3));

      CommonOps.scale(0, A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      CommonOps.mult(matrixExponential, controller.alphas.get(1), s2StartOf2);
      CommonOps.addEquals(s2StartOf2, MathTools.pow(0, 0), controller.betas.get(1).get(0));
      CommonOps.addEquals(s2StartOf2, MathTools.pow(0, 1), controller.betas.get(1).get(1));
      CommonOps.addEquals(s2StartOf2, MathTools.pow(0, 2), controller.betas.get(1).get(2));
      CommonOps.addEquals(s2StartOf2, MathTools.pow(0, 2), controller.betas.get(1).get(3));

      EjmlUnitTests.assertEquals(s2EndOf1, s2StartOf2, epsilon);


      CommonOps.scale(finalTime2 - finalTime1, A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      CommonOps.mult(matrixExponential, controller.alphas.get(1), s2EndOf2);
      CommonOps.addEquals(s2EndOf2, MathTools.pow(finalTime2 - finalTime1, 0), controller.betas.get(1).get(0));
      CommonOps.addEquals(s2EndOf2, MathTools.pow(finalTime2 - finalTime1, 1), controller.betas.get(1).get(1));
      CommonOps.addEquals(s2EndOf2, MathTools.pow(finalTime2 - finalTime1, 2), controller.betas.get(1).get(2));
      CommonOps.addEquals(s2EndOf2, MathTools.pow(finalTime2 - finalTime1, 3), controller.betas.get(1).get(3));

      CommonOps.scale(0, A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      CommonOps.mult(matrixExponential, controller.alphas.get(2), s2StartOf3);
      CommonOps.addEquals(s2StartOf3, MathTools.pow(0, 0), controller.betas.get(2).get(0));
      CommonOps.addEquals(s2StartOf3, MathTools.pow(0, 1), controller.betas.get(2).get(1));
      CommonOps.addEquals(s2StartOf3, MathTools.pow(0, 2), controller.betas.get(2).get(2));
      CommonOps.addEquals(s2StartOf3, MathTools.pow(0, 3), controller.betas.get(2).get(3));

      EjmlUnitTests.assertEquals(s2EndOf2, s2StartOf3, epsilon);

      DenseMatrix64F finalY = new DenseMatrix64F(3, 1);
      vrpEnd.get(finalY);

      // check trajectory

      for (double time = 0; time <= finalTime1; time += 0.01)
      {
         controller.computeS2(time);

         CommonOps.scale(time, A2Expected, timeScaledDynamics);
         matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);

         DenseMatrix64F s2Expected = new DenseMatrix64F(6, 1);
         CommonOps.mult(matrixExponential, alpha1Expected, s2Expected);
         CommonOps.addEquals(s2Expected, MathTools.pow(time, 0), beta11Expected);
         CommonOps.addEquals(s2Expected, MathTools.pow(time, 1), beta12Expected);
         CommonOps.addEquals(s2Expected, MathTools.pow(time, 2), beta13Expected);
         CommonOps.addEquals(s2Expected, MathTools.pow(time, 3), beta14Expected);

         EjmlUnitTests.assertEquals(timeScaledDynamics, controller.timeScaledDynamics, epsilon);
         EjmlUnitTests.assertEquals(matrixExponential, controller.exponential, epsilon);
         EjmlUnitTests.assertEquals(betaSum, controller.summedBetas, epsilon);
         EjmlUnitTests.assertEquals(alpha1Expected, controller.alphas.get(0), epsilon);

         EjmlUnitTests.assertEquals(s2Expected, controller.getCostJacobian(), epsilon);

         vrpTrajectory1.compute(time);
         vrpTrajectory1.getPosition().get(yd);
         CommonOps.subtractEquals(yd, finalY);

         CommonOps.mult(-0.5, R1InvBTrans, s2Expected, k2Method1);
         CommonOps.multAdd(R1InvDQ, yd, k2Method1);

         tempMatrix.reshape(3, 6);
         CommonOps.mult(-0.5, R1InvBTrans, matrixExponential, tempMatrix);
         CommonOps.mult(tempMatrix, controller.alphas.get(0), k2Method2);

         CommonOps.addEquals(k2Method2, MathTools.pow(time, 0), controller.gammas.get(0).get(0));
         CommonOps.addEquals(k2Method2, MathTools.pow(time, 1), controller.gammas.get(0).get(1));
         CommonOps.addEquals(k2Method2, MathTools.pow(time, 2), controller.gammas.get(0).get(2));
         CommonOps.addEquals(k2Method2, MathTools.pow(time, 3), controller.gammas.get(0).get(3));

         DenseMatrix64F K1Expected = new DenseMatrix64F(3, 6);
         CommonOps.mult(-1.0, controller.R1Inverse, NBExpected, K1Expected);

         EjmlUnitTests.assertEquals(k2Method1, k2Method2, epsilon);
         MatrixTestTools.assertMatrixEquals(k2Method1, controller.k2, epsilon);
         MatrixTestTools.assertMatrixEquals(K1Expected, controller.K1, epsilon);
      }

      for (double time = finalTime1; time <= finalTime2; time += 0.01)
      {
         int j = 1;
         double localTime = time - finalTime1;
         controller.computeS2(time);

         CommonOps.scale(localTime, A2Expected, timeScaledDynamics);
         matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);

         DenseMatrix64F s2Expected = new DenseMatrix64F(6, 1);
         CommonOps.mult(matrixExponential, alpha2Expected, s2Expected);
         CommonOps.addEquals(s2Expected, MathTools.pow(localTime, 0), beta21Expected);
         CommonOps.addEquals(s2Expected, MathTools.pow(localTime, 1), beta22Expected);
         CommonOps.addEquals(s2Expected, MathTools.pow(localTime, 2), beta23Expected);
         CommonOps.addEquals(s2Expected, MathTools.pow(localTime, 3), beta24Expected);

         EjmlUnitTests.assertEquals(alpha2Expected, controller.alphas.get(1), epsilon);

         EjmlUnitTests.assertEquals(s2Expected, controller.getCostJacobian(), epsilon);

         vrpTrajectory2.compute(localTime);
         vrpTrajectory2.getPosition().get(yd);
         CommonOps.subtractEquals(yd, finalY);

         CommonOps.mult(-0.5, R1InvBTrans, s2Expected, k2Method1);
         CommonOps.multAdd(R1InvDQ, yd, k2Method1);

         tempMatrix.reshape(3, 6);
         CommonOps.mult(-0.5, R1InvBTrans, matrixExponential, tempMatrix);
         CommonOps.mult(tempMatrix, controller.alphas.get(1), k2Method2);

         CommonOps.addEquals(k2Method2, MathTools.pow(localTime, 0), controller.gammas.get(1).get(0));
         CommonOps.addEquals(k2Method2, MathTools.pow(localTime, 1), controller.gammas.get(1).get(1));
         CommonOps.addEquals(k2Method2, MathTools.pow(localTime, 2), controller.gammas.get(1).get(2));
         CommonOps.addEquals(k2Method2, MathTools.pow(localTime, 3), controller.gammas.get(1).get(3));

         DenseMatrix64F K1Expected = new DenseMatrix64F(3, 6);
         CommonOps.mult(-1.0, controller.R1Inverse, NBExpected, K1Expected);

         MatrixTestTools.assertMatrixEquals(k2Method1, controller.k2, epsilon);
         EjmlUnitTests.assertEquals(k2Method1, k2Method2, epsilon);
         MatrixTestTools.assertMatrixEquals(K1Expected, controller.K1, epsilon);
      }

      for (double time = finalTime2; time <= finalTime3; time += 0.01)
      {
         double localTime = time - finalTime2;
         controller.computeS2(time);

         CommonOps.scale(localTime, A2Expected, timeScaledDynamics);
         matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);

         DenseMatrix64F s2Expected = new DenseMatrix64F(6, 1);
         CommonOps.mult(matrixExponential, alpha3Expected, s2Expected);
         CommonOps.addEquals(s2Expected, MathTools.pow(localTime, 0), beta31Expected);
         CommonOps.addEquals(s2Expected, MathTools.pow(localTime, 1), beta32Expected);
         CommonOps.addEquals(s2Expected, MathTools.pow(localTime, 2), beta33Expected);
         CommonOps.addEquals(s2Expected, MathTools.pow(localTime, 3), beta34Expected);

         EjmlUnitTests.assertEquals(alpha3Expected, controller.alphas.get(2), epsilon);

         EjmlUnitTests.assertEquals(s2Expected, controller.getCostJacobian(), epsilon);

         vrpTrajectory3.compute(localTime);
         vrpTrajectory3.getPosition().get(yd);
         CommonOps.subtractEquals(yd, finalY);

         CommonOps.mult(-0.5, R1InvBTrans, s2Expected, k2Method1);
         CommonOps.multAdd(R1InvDQ, yd, k2Method1);

         tempMatrix.reshape(3, 6);
         CommonOps.mult(-0.5, R1InvBTrans, matrixExponential, tempMatrix);
         CommonOps.mult(tempMatrix, controller.alphas.get(2), k2Method2);

         CommonOps.addEquals(k2Method2, MathTools.pow(localTime, 0), controller.gammas.get(2).get(0));
         CommonOps.addEquals(k2Method2, MathTools.pow(localTime, 1), controller.gammas.get(2).get(1));
         CommonOps.addEquals(k2Method2, MathTools.pow(localTime, 2), controller.gammas.get(2).get(2));
         CommonOps.addEquals(k2Method2, MathTools.pow(localTime, 3), controller.gammas.get(2).get(3));

         DenseMatrix64F K1Expected = new DenseMatrix64F(3, 6);
         CommonOps.mult(-1.0, controller.R1Inverse, NBExpected, K1Expected);

         EjmlUnitTests.assertEquals(k2Method1, k2Method2, epsilon);
         MatrixTestTools.assertMatrixEquals(k2Method1, controller.k2, epsilon);
         MatrixTestTools.assertMatrixEquals(K1Expected, controller.K1, epsilon);
      }

      controller.computeS2(finalTime3);

      // should be zero at tf
      DenseMatrix64F zeroMatrix = new DenseMatrix64F(6, 1);
      EjmlUnitTests.assertEquals(zeroMatrix, controller.getCostJacobian(), epsilon);
   }

   @Test
   public void testBasicTrajectoryTracking()
   {
      SimpleCoMTrajectoryPlanner planner = new SimpleCoMTrajectoryPlanner(() -> omega);
      LQRMomentumController controller = new LQRMomentumController(() -> omega);

      List<SettableContactStateProvider> contactSequence = new ArrayList<>();

      SettableContactStateProvider firstContact = new SettableContactStateProvider();
      SettableContactStateProvider secondContact = new SettableContactStateProvider();
      SettableContactStateProvider thirdContact = new SettableContactStateProvider();

      FramePoint2D vrpStart = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.0, 0.0);
      FramePoint2D vrpMiddle = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.6, 0.75);
      FramePoint2D vrpMiddle2 = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.79, 0.88);
      FramePoint2D vrpEnd = new FramePoint2D(ReferenceFrame.getWorldFrame(), 1.0, 0.5);

      double finalTime1 = 0.05;
      double finalTime2 = 0.1;
      double finalTime3 = 0.15;

      firstContact.getTimeInterval().setInterval(0.0, finalTime1);
      firstContact.setStartCopPosition(vrpStart);
      firstContact.setEndCopPosition(vrpMiddle);

      secondContact.getTimeInterval().setInterval(finalTime1, finalTime2);
      secondContact.setStartCopPosition(vrpMiddle);
      secondContact.setEndCopPosition(vrpMiddle2);

      thirdContact.getTimeInterval().setInterval(finalTime2, finalTime3);
      thirdContact.setStartCopPosition(vrpMiddle2);
      thirdContact.setEndCopPosition(vrpEnd);

      contactSequence.add(firstContact);
//      contactSequence.add(secondContact);
      //      contactSequence.add(thirdContact);

      double nominalHeight = 1.0;
      FramePoint3D initialComPosition = new FramePoint3D();
      initialComPosition.setZ(nominalHeight);
      planner.setNominalCoMHeight(nominalHeight);
      planner.setInitialCenterOfMassState(initialComPosition, new FrameVector3D());
      planner.solveForTrajectory(contactSequence);
      double time = 0.0;
      planner.compute(time);

      controller.setVRPTrajectory(planner.getVRPTrajectories());

      double dt = 5e-7;
      FramePoint3D comPosition = new FramePoint3D(planner.getDesiredCoMPosition());
      FrameVector3D comVelocity = new FrameVector3D(planner.getDesiredCoMVelocity());
      FrameVector3D previousVelocity = new FrameVector3D();
      FrameVector3D previousAcceleration = new FrameVector3D();

      for (; time <= finalTime1; time += dt)
      {
         DenseMatrix64F currentState = new DenseMatrix64F(6, 1);
         comPosition.get(currentState);
         comVelocity.get(3, currentState);

         controller.computeControlInput(currentState, time);

         FrameVector3D acceleration = new FrameVector3D();
         acceleration.set(controller.getU());

         comVelocity.scaleAdd(0.5 * dt, previousAcceleration, comVelocity);
         comVelocity.scaleAdd(0.5 * dt, acceleration, comVelocity);
         comPosition.scaleAdd(0.5 * dt, previousVelocity, comPosition);
         comPosition.scaleAdd(0.5 * dt, comVelocity, comPosition);

         previousAcceleration.set(acceleration);
         previousVelocity.set(comVelocity);
      }

      FramePoint3D finalPosition = new FramePoint3D(vrpMiddle);
      finalPosition.addZ(nominalHeight);

      FramePoint3D icpPosition = new FramePoint3D();
      icpPosition.scaleAdd(1.0 / omega, comVelocity, comPosition);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(finalPosition, icpPosition, 1e-5);
   }
}
