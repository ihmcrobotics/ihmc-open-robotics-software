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
   public void testComputingS2()
   {
      LQRMomentumController controller = new LQRMomentumController();

      Point3D vrpStart = new Point3D(0.0, 0.0, 1.0);
      Point3D vrpEnd = new Point3D(1.0, 0.5, 1.0);
      Trajectory3D vrpTrajectory = new Trajectory3D(4);
      double finalTime = 1.0;
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
//      CommonOps.addEquals(betaSum, -1.0 * MathTools.pow(finalTime, 1), beta2Expected);

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
}
