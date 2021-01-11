package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactState;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SimpleCoMTrajectoryPlanner;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.matrixlib.NativeCommonOps;
import us.ihmc.robotics.linearAlgebra.MatrixExponentialCalculator;
import us.ihmc.robotics.math.trajectories.Trajectory3D;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.robotics.Assert.assertEquals;

public class LQRJumpMomentumControllerNoFlightTest
{
   private static final double omega = 3.0;
   private static final double epsilon = 1e-9;

   @Test
   public void testComputingS1()
   {
      LQRJumpMomentumController controller = new LQRJumpMomentumController(() -> omega, 1.0);

      Point3D vrpStart = new Point3D(0.0, 0.0, 1.0);
      Point3D vrpEnd = new Point3D(1.0, 0.5, 1.0);
      Trajectory3D vrpTrajectory = new Trajectory3D(4);
      vrpTrajectory.setLinear(0.0, 1.0, vrpStart, vrpEnd);
      List<Trajectory3D> trajectories = new ArrayList<>();
      trajectories.add(vrpTrajectory);
      List<SettableContactStateProvider> contactStateProviders = new ArrayList<>();
      contactStateProviders.add(new SettableContactStateProvider());
      contactStateProviders.get(0).setContactState(ContactState.IN_CONTACT);

      controller.setVRPTrajectory(trajectories, contactStateProviders);

      DMatrixRMaj AExpected = new DMatrixRMaj(6, 6);
      AExpected.set(0, 3, 1.0);
      AExpected.set(1, 4, 1.0);
      AExpected.set(2, 5, 1.0);

      DMatrixRMaj BExpected = new DMatrixRMaj(6, 3);
      BExpected.set(3, 0, 1.0);
      BExpected.set(4, 1, 1.0);
      BExpected.set(5, 2, 1.0);

      DMatrixRMaj CExpected = new DMatrixRMaj(3, 6);

      CExpected.set(0, 0, 1.0);
      CExpected.set(1, 1, 1.0);
      CExpected.set(2, 2, 1.0);

      DMatrixRMaj DExpected = new DMatrixRMaj(3, 3);

      DExpected.set(0, 0, -1.0 / MathTools.square(omega));
      DExpected.set(1, 1, -1.0 / MathTools.square(omega));
      DExpected.set(2, 2, -1.0 / MathTools.square(omega));

      EjmlUnitTests.assertEquals(AExpected, controller.getA(), epsilon);
      EjmlUnitTests.assertEquals(BExpected, controller.getB(), epsilon);
      EjmlUnitTests.assertEquals(CExpected, controller.getC(), epsilon);
      EjmlUnitTests.assertEquals(DExpected, controller.getD(), epsilon);

      DMatrixRMaj QExpected = new DMatrixRMaj(3, 3);
      QExpected.set(0, 0, LQRJumpMomentumController.defaultVrpTrackingWeight);
      QExpected.set(1, 1, LQRJumpMomentumController.defaultVrpTrackingWeight);
      QExpected.set(2, 2, LQRJumpMomentumController.defaultVrpTrackingWeight);

      DMatrixRMaj RExpected = new DMatrixRMaj(3, 3);
      RExpected.set(0, 0, LQRJumpMomentumController.defaultMomentumRateWeight);
      RExpected.set(1, 1, LQRJumpMomentumController.defaultMomentumRateWeight);
      RExpected.set(2, 2, LQRJumpMomentumController.defaultMomentumRateWeight);

      EjmlUnitTests.assertEquals(QExpected, controller.getQ(), epsilon);
      EjmlUnitTests.assertEquals(RExpected, controller.getR(), epsilon);

      DMatrixRMaj Q1Expected = new DMatrixRMaj(3, 3);
      NativeCommonOps.multQuad(CExpected, QExpected, Q1Expected);

      DMatrixRMaj R1Expected = new DMatrixRMaj(3, 3);
      DMatrixRMaj R1InverseExpected = new DMatrixRMaj(3, 3);
      NativeCommonOps.multQuad(DExpected, QExpected, R1Expected);
      CommonOps_DDRM.addEquals(R1Expected, RExpected);
      NativeCommonOps.invert(R1Expected, R1InverseExpected);

//      EjmlUnitTests.assertEquals(Q1Expected, controller.Q1, epsilon);
//      EjmlUnitTests.assertEquals(R1Expected, controller.R1, epsilon);
//      EjmlUnitTests.assertEquals(R1InverseExpected, controller.R1Inverse, epsilon);

      DMatrixRMaj NExpected = new DMatrixRMaj(6, 3);
      DMatrixRMaj NTransposeExpected = new DMatrixRMaj(3, 6);
      DMatrixRMaj tempMatrix = new DMatrixRMaj(3, 3);
      CommonOps_DDRM.mult(QExpected, DExpected, tempMatrix);
      CommonOps_DDRM.multTransA(CExpected, tempMatrix, NExpected);
      CommonOps_DDRM.transpose(NExpected, NTransposeExpected);

//      EjmlUnitTests.assertEquals(NExpected, controller.N, epsilon);
//      EjmlUnitTests.assertEquals(NTransposeExpected, controller.NTranspose, epsilon);


      controller.computeS1Segments();

      EjmlUnitTests.assertEquals(AExpected, controller.getA(), epsilon);
      EjmlUnitTests.assertEquals(BExpected, controller.getB(), epsilon);
      EjmlUnitTests.assertEquals(CExpected, controller.getC(), epsilon);
      EjmlUnitTests.assertEquals(DExpected, controller.getD(), epsilon);

      EjmlUnitTests.assertEquals(QExpected, controller.getQ(), epsilon);
      EjmlUnitTests.assertEquals(RExpected, controller.getR(), epsilon);

//      EjmlUnitTests.assertEquals(Q1Expected, controller.Q1, epsilon);
//      EjmlUnitTests.assertEquals(R1Expected, controller.R1, epsilon);
//      EjmlUnitTests.assertEquals(R1InverseExpected, controller.R1Inverse, epsilon);
//
//      EjmlUnitTests.assertEquals(NExpected, controller.N, epsilon);

      DMatrixRMaj QRiccatiExpected = new DMatrixRMaj(3, 3);
      DMatrixRMaj ARiccatiExpected = new DMatrixRMaj(6, 6);

      NativeCommonOps.multQuad(NTransposeExpected, R1InverseExpected, QRiccatiExpected);
      CommonOps_DDRM.scale(-1.0, QRiccatiExpected);
      CommonOps_DDRM.addEquals(QRiccatiExpected, Q1Expected);

      tempMatrix = new DMatrixRMaj(3, 6);
      CommonOps_DDRM.multTransB(R1InverseExpected, NExpected, tempMatrix);
      CommonOps_DDRM.mult(-1.0, BExpected, tempMatrix, ARiccatiExpected);
      CommonOps_DDRM.addEquals(ARiccatiExpected, AExpected);

//      EjmlUnitTests.assertEquals(QRiccatiExpected, controller.QRiccati, epsilon);
//      EjmlUnitTests.assertEquals(ARiccatiExpected, controller.ARiccati, epsilon);


      DMatrixRMaj S1 = controller.getCostHessian();
//
      DMatrixRMaj NB = new DMatrixRMaj(NExpected);
      CommonOps_DDRM.transpose(NB);
      CommonOps_DDRM.multAddTransA(BExpected, S1, NB);
      DMatrixRMaj S1DotExpected = new DMatrixRMaj(6, 6);
      NativeCommonOps.multQuad(NB, R1InverseExpected, S1DotExpected);
      CommonOps_DDRM.addEquals(S1DotExpected, -1.0, Q1Expected);
      CommonOps_DDRM.multAdd(-1.0, S1, AExpected, S1DotExpected);
      CommonOps_DDRM.multAddTransA(-1.0, AExpected, S1, S1DotExpected);

      DMatrixRMaj H = new DMatrixRMaj( 4, 4);
      H.set(0, 1, 1.0);
      H.set(1, 0, 0.5);
      H.set(1, 3, -1.0);
      H.set(2, 0, -0.5);
      H.set(2, 3, -0.5);
      H.set(3, 2, -1.0);



      tempMatrix.reshape(6, 3);
      DMatrixRMaj S1Dot = new DMatrixRMaj(6, 6);
      DMatrixRMaj BTranspose = new DMatrixRMaj(BExpected);
      CommonOps_DDRM.transpose(BTranspose);
      NativeCommonOps.multQuad(BTranspose, R1InverseExpected, tempMatrix);
      NativeCommonOps.multQuad(S1, tempMatrix, S1Dot);
      CommonOps_DDRM.addEquals(S1Dot, -1.0, QRiccatiExpected);
      CommonOps_DDRM.multAdd(-1.0, S1, ARiccatiExpected, S1Dot);
      CommonOps_DDRM.multAddTransA(-1.0, ARiccatiExpected, S1, S1Dot);

      EjmlUnitTests.assertEquals(S1DotExpected, S1Dot, epsilon);
      EjmlUnitTests.assertEquals(new DMatrixRMaj(6, 6), S1Dot, epsilon);
   }

   @Test
   public void testComputingS2FromSingleLinearTrajectory()
   {
      LQRJumpMomentumController controller = new LQRJumpMomentumController(() -> omega, 1.0);
      controller.computeS1Segments();

      Point3D vrpStart = new Point3D(0.0, 0.0, 1.0);
      Point3D vrpEnd = new Point3D(1.0, 0.5, 1.0);
      Trajectory3D vrpTrajectory = new Trajectory3D(4);
      double finalTime = 1.5;
      vrpTrajectory.setLinear(0.0, finalTime, vrpStart, vrpEnd);
      List<Trajectory3D> trajectories = new ArrayList<>();
      trajectories.add(vrpTrajectory);
      List<SettableContactStateProvider> contactStateProviders = new ArrayList<>();
      contactStateProviders.add(new SettableContactStateProvider());
      contactStateProviders.get(0).setContactState(ContactState.IN_CONTACT);

      DMatrixRMaj finalPosition = new DMatrixRMaj(3, 1);
      vrpEnd.get(finalPosition);

      controller.setVRPTrajectory(trajectories, contactStateProviders);

      controller.computeS1Segments();
      controller.computeS2Segments();

      DMatrixRMaj NExpected = new DMatrixRMaj(6, 3);
      DMatrixRMaj NBExpected = new DMatrixRMaj(3, 6);
      DMatrixRMaj NTransposeExpected = new DMatrixRMaj(3, 6);
      DMatrixRMaj tempMatrix = new DMatrixRMaj(3, 3);
      CommonOps_DDRM.mult(controller.getQ(), controller.getD(), tempMatrix);
      CommonOps_DDRM.multTransA(controller.getC(), tempMatrix, NExpected);
      CommonOps_DDRM.transpose(NExpected, NTransposeExpected);

      CommonOps_DDRM.multTransA(controller.getB(), controller.getCostHessian(), NBExpected);
      CommonOps_DDRM.addEquals(NBExpected, NTransposeExpected);

      LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.general(0, 0);

      DMatrixRMaj A2Expected = new DMatrixRMaj(6, 6);
      DMatrixRMaj A2InverseExpected = new DMatrixRMaj(6, 6);
      DMatrixRMaj B2Expected = new DMatrixRMaj(6, 3);
      DMatrixRMaj R1Expected = new DMatrixRMaj(3, 3);
      DMatrixRMaj R1InverseExpected = new DMatrixRMaj(3, 3);

      NativeCommonOps.multQuad(controller.getD(), controller.getQ(), R1Expected);
      CommonOps_DDRM.addEquals(R1Expected, controller.getR());
      NativeCommonOps.invert(R1Expected, R1InverseExpected);

      CommonOps_DDRM.transpose(controller.getA(), A2Expected);
      CommonOps_DDRM.scale(-1.0, A2Expected);

      tempMatrix.reshape(3, 6);
      CommonOps_DDRM.multTransB(R1InverseExpected, controller.getB(), tempMatrix);
      CommonOps_DDRM.multAddTransA(NBExpected, tempMatrix, A2Expected);

      DMatrixRMaj tempMatrix2 = new DMatrixRMaj(6, 3);
      tempMatrix.reshape(6, 3);
      CommonOps_DDRM.multTransA(NBExpected, R1InverseExpected, tempMatrix);
      CommonOps_DDRM.transpose(controller.getC(), tempMatrix2);
      CommonOps_DDRM.multAdd(-1.0, tempMatrix, controller.getD(), tempMatrix2);
      CommonOps_DDRM.mult(2.0, tempMatrix2, controller.getQ(), B2Expected);

      solver.setA(A2Expected);
      solver.invert(A2InverseExpected);

//      EjmlUnitTests.assertEquals(NBExpected, controller.NB, epsilon);
//      EjmlUnitTests.assertEquals(A2Expected, controller.A2, epsilon);
//      EjmlUnitTests.assertEquals(B2Expected, controller.B2, epsilon);
//      EjmlUnitTests.assertEquals(A2InverseExpected, controller.A2Inverse, epsilon);


      DMatrixRMaj A2InverseB2 = new DMatrixRMaj(6, 3);
      DMatrixRMaj R1InvDQ = new DMatrixRMaj(3, 3);
      DMatrixRMaj DQ = new DMatrixRMaj(3, 3);
      DMatrixRMaj R1InvBTrans = new DMatrixRMaj(3, 6);
      DMatrixRMaj k2Method1 = new DMatrixRMaj(3, 1);
      DMatrixRMaj k2Method2 = new DMatrixRMaj(3, 1);
      DMatrixRMaj yd = new DMatrixRMaj(3, 1);

      CommonOps_DDRM.mult(A2InverseExpected, B2Expected, A2InverseB2);
      CommonOps_DDRM.mult(controller.getD(), controller.getQ(), DQ);
      CommonOps_DDRM.mult(R1InverseExpected, DQ, R1InvDQ);
      CommonOps_DDRM.multTransB(R1InverseExpected, controller.getB(), R1InvBTrans);

//      assertEquals(1, controller.alphas.size());
//      assertEquals(1, controller.betas.size());
//      assertEquals(1, controller.gammas.size());
//      assertEquals(2, controller.betas.get(0).size());
//      assertEquals(2, controller.gammas.get(0).size());

      DMatrixRMaj beta1Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj beta2Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj gamma1Expected = new DMatrixRMaj(3, 1);
      DMatrixRMaj gamma2Expected = new DMatrixRMaj(3, 1);
      DMatrixRMaj coefficients = new DMatrixRMaj(3, 1);

      vrpTrajectory.getCoefficients(1, coefficients);
      CommonOps_DDRM.mult(-1, A2InverseB2, coefficients, beta2Expected);

      CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma2Expected);
      CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta2Expected, gamma2Expected);

      vrpTrajectory.getCoefficients(0, coefficients);
      CommonOps_DDRM.subtractEquals(coefficients, finalPosition);
      CommonOps_DDRM.mult(A2InverseExpected, beta2Expected, beta1Expected);
      CommonOps_DDRM.multAdd(-1.0, A2InverseB2, coefficients, beta1Expected);

      CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma1Expected);
      CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta1Expected, gamma1Expected);

      EjmlUnitTests.assertEquals(beta2Expected, ((AlgebraicS2Segment) controller.getS2Segment(0)).getBeta(1), epsilon);
      EjmlUnitTests.assertEquals(beta1Expected, ((AlgebraicS2Segment) controller.getS2Segment(0)).getBeta(0), epsilon);
      //      EjmlUnitTests.assertEquals(gamma2Expected, controller.gammas.get(0).get(1), epsilon);
//      EjmlUnitTests.assertEquals(gamma1Expected, controller.gammas.get(0).get(0), epsilon);

      MatrixExponentialCalculator matrixExponentialCalculator = new MatrixExponentialCalculator(6);

      DMatrixRMaj alphaExpected = new DMatrixRMaj(6, 1);

      DMatrixRMaj timeScaledDynamics = new DMatrixRMaj(6, 6);
      DMatrixRMaj matrixExponential = new DMatrixRMaj(6, 6);
      DMatrixRMaj betaSum = new DMatrixRMaj(6, 1);

      CommonOps_DDRM.scale(finalTime, A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime, 0), beta1Expected);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime, 1), beta2Expected);

      solver.setA(matrixExponential);
      solver.solve(betaSum, alphaExpected);

//      EjmlUnitTests.assertEquals(timeScaledDynamics, controller.timeScaledA2, epsilon);
//      EjmlUnitTests.assertEquals(matrixExponential, controller.A2Exponential, epsilon);
//      EjmlUnitTests.assertEquals(betaSum, controller.summedBetas, epsilon);
      EjmlUnitTests.assertEquals(beta1Expected, ((AlgebraicS2Segment) controller.getS2Segment(0)).getBeta(0), epsilon);
      EjmlUnitTests.assertEquals(beta2Expected, ((AlgebraicS2Segment) controller.getS2Segment(0)).getBeta(1), epsilon);
      EjmlUnitTests.assertEquals(alphaExpected, ((AlgebraicS2Segment) controller.getS2Segment(0)).getAlpha(), epsilon);

      DMatrixRMaj s2 = new DMatrixRMaj(6, 1);
      CommonOps_DDRM.mult(matrixExponential, alphaExpected, s2);
      CommonOps_DDRM.addEquals(s2, betaSum);


      DMatrixRMaj finalY = new DMatrixRMaj(3, 1);
      vrpEnd.get(finalY);

      vrpTrajectory.compute(0.0);
      vrpTrajectory.getPosition().get(yd);
      CommonOps_DDRM.subtractEquals(yd, finalY);

      CommonOps_DDRM.mult(-0.5, R1InvBTrans, s2, k2Method1);
      CommonOps_DDRM.multAdd(R1InvDQ, yd, k2Method1);

      tempMatrix.reshape(3, 6);
//      CommonOps_DDRM.scale(0.0, A2Expected, timeScaledDynamics);
//      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);

      CommonOps_DDRM.mult(-0.5, R1InvBTrans, matrixExponential, tempMatrix);
      CommonOps_DDRM.mult(tempMatrix, alphaExpected, k2Method2);

      CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(finalTime, 0), gamma1Expected);
      CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(finalTime, 1), gamma2Expected);

      DMatrixRMaj K1Expected = new DMatrixRMaj(3, 6);
      CommonOps_DDRM.mult(-1.0, R1InverseExpected, NBExpected, K1Expected);


      controller.computeS1AndK1(finalTime);
      controller.computeS2AndK2(finalTime);
//      MatrixTestTools.assertMatrixEquals(k2Method1, k2Method2, epsilon);
//      MatrixTestTools.assertMatrixEquals(k2Method1, controller.getK2(), epsilon);
//      MatrixTestTools.assertMatrixEquals(K1Expected, controller.getK1(), epsilon);
//
      for (double time = 0.0; time <= finalTime; time += 0.01)
      {
         controller.computeS1AndK1(time);
         controller.computeS2AndK2(time);

         CommonOps_DDRM.scale(time, A2Expected, timeScaledDynamics);
         matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);

         DMatrixRMaj s2Expected = new DMatrixRMaj(6, 1);
         CommonOps_DDRM.mult(matrixExponential, alphaExpected, s2Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(time, 0), beta1Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(time, 1), beta2Expected);

         DMatrixRMaj gamma1ExpectedAlt = new DMatrixRMaj(3, 1);
         DMatrixRMaj gamma2ExpectedAlt = new DMatrixRMaj(3, 1);

         vrpTrajectory.getCoefficients(0, coefficients);
         CommonOps_DDRM.subtractEquals(coefficients, finalPosition);
         CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma1ExpectedAlt);
         CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta1Expected, gamma1ExpectedAlt);

         vrpTrajectory.getCoefficients(1, coefficients);
         CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma2ExpectedAlt);
         CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta2Expected, gamma2ExpectedAlt);

//         EjmlUnitTests.assertEquals(timeScaledDynamics, controller.timeScaledA2, epsilon);
//         EjmlUnitTests.assertEquals(matrixExponential, controller.A2Exponential, epsilon);
//         EjmlUnitTests.assertEquals(betaSum, controller.summedBetas, epsilon);
         EjmlUnitTests.assertEquals(alphaExpected, ((AlgebraicS2Segment) controller.getS2Segment(0)).getAlpha(), epsilon);
         EjmlUnitTests.assertEquals(beta1Expected, ((AlgebraicS2Segment) controller.getS2Segment(0)).getBeta(0), epsilon);
         EjmlUnitTests.assertEquals(beta2Expected, ((AlgebraicS2Segment) controller.getS2Segment(0)).getBeta(1), epsilon);
//         EjmlUnitTests.assertEquals(gamma1ExpectedAlt, gamma1Expected, epsilon);
//         EjmlUnitTests.assertEquals(gamma2ExpectedAlt, gamma2Expected, epsilon);
//         EjmlUnitTests.assertEquals(gamma1Expected, controller.gammas.get(0).get(0), epsilon);
//         EjmlUnitTests.assertEquals(gamma2Expected, controller.gammas.get(0).get(1), epsilon);

         EjmlUnitTests.assertEquals(s2Expected, controller.getCostJacobian(), epsilon);

         vrpTrajectory.compute(time);
         vrpTrajectory.getPosition().get(yd);
         CommonOps_DDRM.subtractEquals(yd, finalY);

         CommonOps_DDRM.mult(-0.5, R1InvBTrans, s2Expected, k2Method1);
         CommonOps_DDRM.multAdd(R1InvDQ, yd, k2Method1);

         tempMatrix.reshape(3, 6);
         CommonOps_DDRM.mult(-0.5, R1InvBTrans, matrixExponential, tempMatrix);
         CommonOps_DDRM.mult(tempMatrix, alphaExpected, k2Method2);

         CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(time, 0), gamma1Expected);
         CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(time, 1), gamma2Expected);

         CommonOps_DDRM.mult(-1.0, R1InverseExpected, NBExpected, K1Expected);

         EjmlUnitTests.assertEquals(k2Method1, k2Method2, epsilon);
         MatrixTestTools.assertMatrixEquals(k2Method1, controller.getK2(), epsilon);
         MatrixTestTools.assertMatrixEquals(K1Expected, controller.getK1(), epsilon);
      }

      controller.computeS1AndK1(finalTime);
      controller.computeS2AndK2(finalTime);

      // should be zero at tf
      DMatrixRMaj zeroMatrix = new DMatrixRMaj(6, 1);
      EjmlUnitTests.assertEquals(zeroMatrix, controller.getCostJacobian(), epsilon);
   }

   @Test
   public void testComputingS2FromSingleCubicTrajectory()
   {
      LQRJumpMomentumController controller = new LQRJumpMomentumController(() -> omega, 1.0);

      Point3D vrpStart = new Point3D(0.0, 0.0, 1.0);
      Point3D vrpEnd = new Point3D(1.0, 0.5, 1.0);
      Trajectory3D vrpTrajectory = new Trajectory3D(4);
      double finalTime = 1.5;
      vrpTrajectory.setCubic(0.0, finalTime, vrpStart, vrpEnd);
      List<Trajectory3D> trajectories = new ArrayList<>();
      trajectories.add(vrpTrajectory);
      List<SettableContactStateProvider> contactStateProviders = new ArrayList<>();
      contactStateProviders.add(new SettableContactStateProvider());
      contactStateProviders.get(0).setContactState(ContactState.IN_CONTACT);

      DMatrixRMaj finalPosition = new DMatrixRMaj(3, 1);
      vrpEnd.get(finalPosition);

      controller.setVRPTrajectory(trajectories, contactStateProviders);

      controller.computeS1Segments();
      controller.computeS2Segments();

      DMatrixRMaj NExpected = new DMatrixRMaj(6, 3);
      DMatrixRMaj NBExpected = new DMatrixRMaj(3, 6);
      DMatrixRMaj NTransposeExpected = new DMatrixRMaj(3, 6);
      DMatrixRMaj tempMatrix = new DMatrixRMaj(3, 3);
      CommonOps_DDRM.mult(controller.getQ(), controller.getD(), tempMatrix);
      CommonOps_DDRM.multTransA(controller.getC(), tempMatrix, NExpected);
      CommonOps_DDRM.transpose(NExpected, NTransposeExpected);

      CommonOps_DDRM.multTransA(controller.getB(), controller.getCostHessian(), NBExpected);
      CommonOps_DDRM.addEquals(NBExpected, NTransposeExpected);

      LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.general(0, 0);

      DMatrixRMaj A2Expected = new DMatrixRMaj(6, 6);
      DMatrixRMaj A2InverseExpected = new DMatrixRMaj(6, 6);
      DMatrixRMaj B2Expected = new DMatrixRMaj(6, 3);
      DMatrixRMaj R1Expected = new DMatrixRMaj(3, 3);
      DMatrixRMaj R1InverseExpected = new DMatrixRMaj(3, 3);

      NativeCommonOps.multQuad(controller.getD(), controller.getQ(), R1Expected);
      CommonOps_DDRM.addEquals(R1Expected, controller.getR());
      NativeCommonOps.invert(R1Expected, R1InverseExpected);

      CommonOps_DDRM.transpose(controller.getA(), A2Expected);
      CommonOps_DDRM.scale(-1.0, A2Expected);

      tempMatrix.reshape(3, 6);
      CommonOps_DDRM.multTransB(R1InverseExpected, controller.getB(), tempMatrix);
      CommonOps_DDRM.multAddTransA(NBExpected, tempMatrix, A2Expected);

      DMatrixRMaj tempMatrix2 = new DMatrixRMaj(6, 3);
      tempMatrix.reshape(6, 3);
      CommonOps_DDRM.multTransA(NBExpected, R1InverseExpected, tempMatrix);
      CommonOps_DDRM.transpose(controller.getC(), tempMatrix2);
      CommonOps_DDRM.multAdd(-1.0, tempMatrix, controller.getD(), tempMatrix2);
      CommonOps_DDRM.mult(2.0, tempMatrix2, controller.getQ(), B2Expected);

      solver.setA(A2Expected);
      solver.invert(A2InverseExpected);

      controller.computeS1AndK1(0.0);


      DMatrixRMaj K1Expected = new DMatrixRMaj(3, 6);
      CommonOps_DDRM.mult(-1.0, R1InverseExpected, NBExpected, K1Expected);

      MatrixTestTools.assertMatrixEquals(K1Expected, controller.getK1(), epsilon);


      DMatrixRMaj A2InverseB2 = new DMatrixRMaj(6, 3);
      DMatrixRMaj R1InvDQ = new DMatrixRMaj(3, 3);
      DMatrixRMaj DQ = new DMatrixRMaj(3, 3);
      DMatrixRMaj R1InvBTrans = new DMatrixRMaj(3, 6);
      DMatrixRMaj k2Method1 = new DMatrixRMaj(3, 1);
      DMatrixRMaj k2Method2 = new DMatrixRMaj(3, 1);
      DMatrixRMaj yd = new DMatrixRMaj(3, 1);

      CommonOps_DDRM.mult(A2InverseExpected, B2Expected, A2InverseB2);
      CommonOps_DDRM.mult(controller.getD(), controller.getQ(), DQ);
      CommonOps_DDRM.mult(R1InverseExpected, DQ, R1InvDQ);
      CommonOps_DDRM.multTransB(R1InverseExpected, controller.getB(), R1InvBTrans);

//      assertEquals(1, controller.alphas.size());
//      assertEquals(1, controller.betas.size());
//      assertEquals(1, controller.gammas.size());
//      assertEquals(4, controller.betas.get(0).size());
//      assertEquals(4, controller.gammas.get(0).size());

      DMatrixRMaj beta1Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj beta2Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj beta3Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj beta4Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj gamma1Expected = new DMatrixRMaj(3, 1);
      DMatrixRMaj gamma2Expected = new DMatrixRMaj(3, 1);
      DMatrixRMaj gamma3Expected = new DMatrixRMaj(3, 1);
      DMatrixRMaj gamma4Expected = new DMatrixRMaj(3, 1);
      DMatrixRMaj coefficients = new DMatrixRMaj(3, 1);

      vrpTrajectory.getCoefficients(3, coefficients);
      CommonOps_DDRM.mult(-1, A2InverseB2, coefficients, beta4Expected);

      CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma4Expected);
      CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta4Expected, gamma4Expected);

      vrpTrajectory.getCoefficients(2, coefficients);
      CommonOps_DDRM.mult(3, A2InverseExpected, beta4Expected, beta3Expected);
      CommonOps_DDRM.multAdd(-1.0, A2InverseB2, coefficients, beta3Expected);

      CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma3Expected);
      CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta3Expected, gamma3Expected);

      vrpTrajectory.getCoefficients(1, coefficients);
      CommonOps_DDRM.mult(2, A2InverseExpected, beta3Expected, beta2Expected);
      CommonOps_DDRM.multAdd(-1.0, A2InverseB2, coefficients, beta2Expected);

      CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma2Expected);
      CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta2Expected, gamma2Expected);

      vrpTrajectory.getCoefficients(0, coefficients);
      CommonOps_DDRM.subtractEquals(coefficients, finalPosition);
      CommonOps_DDRM.mult(A2InverseExpected, beta2Expected, beta1Expected);
      CommonOps_DDRM.multAdd(-1.0, A2InverseB2, coefficients, beta1Expected);

      CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma1Expected);
      CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta1Expected, gamma1Expected);

      EjmlUnitTests.assertEquals(beta4Expected, ((AlgebraicS2Segment) controller.getS2Segment(0)).getBeta(3), epsilon);
      EjmlUnitTests.assertEquals(beta3Expected, ((AlgebraicS2Segment) controller.getS2Segment(0)).getBeta(2), epsilon);
      EjmlUnitTests.assertEquals(beta2Expected, ((AlgebraicS2Segment) controller.getS2Segment(0)).getBeta(1), epsilon);
      EjmlUnitTests.assertEquals(beta1Expected, ((AlgebraicS2Segment) controller.getS2Segment(0)).getBeta(0), epsilon);
//      EjmlUnitTests.assertEquals(gamma4Expected, controller.gammas.get(0).get(3), epsilon);
//      EjmlUnitTests.assertEquals(gamma3Expected, controller.gammas.get(0).get(2), epsilon);
//      EjmlUnitTests.assertEquals(gamma2Expected, controller.gammas.get(0).get(1), epsilon);
//      EjmlUnitTests.assertEquals(gamma1Expected, controller.gammas.get(0).get(0), epsilon);

      MatrixExponentialCalculator matrixExponentialCalculator = new MatrixExponentialCalculator(6);

      DMatrixRMaj alphaExpected = new DMatrixRMaj(6, 1);

      DMatrixRMaj timeScaledDynamics = new DMatrixRMaj(6, 6);
      DMatrixRMaj matrixExponential = new DMatrixRMaj(6, 6);
      DMatrixRMaj betaSum = new DMatrixRMaj(6, 1);

      CommonOps_DDRM.scale(finalTime, A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime, 0), beta1Expected);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime, 1), beta2Expected);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime, 2), beta3Expected);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime, 3), beta4Expected);

      solver.setA(matrixExponential);
      solver.solve(betaSum, alphaExpected);

//      EjmlUnitTests.assertEquals(timeScaledDynamics, controller.timeScaledA2, epsilon);
//      EjmlUnitTests.assertEquals(matrixExponential, controller.A2Exponential, epsilon);
//      EjmlUnitTests.assertEquals(betaSum, controller.summedBetas, epsilon);
      EjmlUnitTests.assertEquals(alphaExpected, ((AlgebraicS2Segment) controller.getS2Segment(0)).getAlpha(), epsilon);

      DMatrixRMaj finalY = new DMatrixRMaj(3, 1);
      vrpEnd.get(finalY);

      for (double time = 0.0; time <= finalTime; time += 0.01)
      {
         controller.computeS1AndK1(time);
         controller.computeS2AndK2(time);

         CommonOps_DDRM.scale(time, A2Expected, timeScaledDynamics);
         matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);

         DMatrixRMaj s2Expected = new DMatrixRMaj(6, 1);
         CommonOps_DDRM.mult(matrixExponential, alphaExpected, s2Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(time, 0), beta1Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(time, 1), beta2Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(time, 2), beta3Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(time, 3), beta4Expected);

//         EjmlUnitTests.assertEquals(timeScaledDynamics, controller.timeScaledA2, epsilon);
//         EjmlUnitTests.assertEquals(matrixExponential, controller.A2Exponential, epsilon);
//         EjmlUnitTests.assertEquals(betaSum, controller.summedBetas, epsilon);
         EjmlUnitTests.assertEquals(alphaExpected, ((AlgebraicS2Segment) controller.getS2Segment(0)).getAlpha(), epsilon);

         EjmlUnitTests.assertEquals(s2Expected, controller.getCostJacobian(), epsilon);

         vrpTrajectory.compute(time);
         vrpTrajectory.getPosition().get(yd);
         CommonOps_DDRM.subtractEquals(yd, finalY);

         CommonOps_DDRM.mult(-0.5, R1InvBTrans, s2Expected, k2Method1);
         CommonOps_DDRM.multAdd(R1InvDQ, yd, k2Method1);

         tempMatrix.reshape(3, 6);
         CommonOps_DDRM.mult(-0.5, R1InvBTrans, matrixExponential, tempMatrix);
         CommonOps_DDRM.mult(tempMatrix, alphaExpected, k2Method2);

//         CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(time, 0), controller.gammas.get(0).get(0));
//         CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(time, 1), controller.gammas.get(0).get(1));
//         CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(time, 2), controller.gammas.get(0).get(2));
//         CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(time, 3), controller.gammas.get(0).get(3));

         K1Expected = new DMatrixRMaj(3, 6);
         CommonOps_DDRM.mult(-1.0, R1InverseExpected, NBExpected, K1Expected);

//         EjmlUnitTests.assertEquals(k2Method1, k2Method2, epsilon);
//         MatrixTestTools.assertMatrixEquals(k2Method2, controller.getK2(), epsilon);
         MatrixTestTools.assertMatrixEquals(k2Method1, controller.getK2(), epsilon);
         MatrixTestTools.assertMatrixEquals(K1Expected, controller.getK1(), epsilon);
      }

      controller.computeS1AndK1(finalTime);
      controller.computeS2AndK2(finalTime);

      // should be zero at tf
      DMatrixRMaj zeroMatrix = new DMatrixRMaj(6, 1);
      EjmlUnitTests.assertEquals(zeroMatrix, controller.getCostJacobian(), epsilon);
   }

   @Test
   public void testComputingS2FromTwoLinearTrajectories()
   {
      LQRJumpMomentumController controller = new LQRJumpMomentumController(() -> omega, 1.0);

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

      List<Trajectory3D> relativeVRPTrajectories = new ArrayList<>();
      Trajectory3D lastTrajectory = trajectories.get(trajectories.size() - 1);
      lastTrajectory.compute(lastTrajectory.getFinalTime());
      DMatrixRMaj finalVRPState = new DMatrixRMaj(3, 1);
      lastTrajectory.getPosition().get(finalVRPState);

      for (int i = 0; i < trajectories.size(); i++)
      {
         Trajectory3D trajectory = trajectories.get(i);
         Trajectory3D relativeTrajectory = new Trajectory3D(5);
         relativeVRPTrajectories.add(relativeTrajectory);

         relativeTrajectory.set(trajectory);
         relativeTrajectory.offsetTrajectoryPosition(-finalVRPState.get(0, 0), -finalVRPState.get(1, 0), -finalVRPState.get(2, 0));
      }

      List<SettableContactStateProvider> contactStateProviders = new ArrayList<>();
      contactStateProviders.add(new SettableContactStateProvider());
      contactStateProviders.add(new SettableContactStateProvider());
      contactStateProviders.get(0).setContactState(ContactState.IN_CONTACT);
      contactStateProviders.get(1).setContactState(ContactState.IN_CONTACT);

      DMatrixRMaj finalPosition = new DMatrixRMaj(3, 1);
      vrpEnd.get(finalPosition);

      controller.setVRPTrajectory(trajectories, contactStateProviders);

      controller.computeS1Segments();
      controller.computeS2Segments();

      DMatrixRMaj NExpected = new DMatrixRMaj(6, 3);
      DMatrixRMaj NBExpected = new DMatrixRMaj(3, 6);
      DMatrixRMaj NTransposeExpected = new DMatrixRMaj(3, 6);
      DMatrixRMaj tempMatrix = new DMatrixRMaj(3, 3);
      CommonOps_DDRM.mult(controller.getQ(), controller.getD(), tempMatrix);
      CommonOps_DDRM.multTransA(controller.getC(), tempMatrix, NExpected);
      CommonOps_DDRM.transpose(NExpected, NTransposeExpected);

      CommonOps_DDRM.multTransA(controller.getB(), controller.getCostHessian(), NBExpected);
      CommonOps_DDRM.addEquals(NBExpected, NTransposeExpected);

      LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.general(0, 0);

      DMatrixRMaj A2Expected = new DMatrixRMaj(6, 6);
      DMatrixRMaj A2InverseExpected = new DMatrixRMaj(6, 6);
      DMatrixRMaj B2Expected = new DMatrixRMaj(6, 3);
      DMatrixRMaj R1Expected = new DMatrixRMaj(3, 3);
      DMatrixRMaj R1InverseExpected = new DMatrixRMaj(3, 3);

      NativeCommonOps.multQuad(controller.getD(), controller.getQ(), R1Expected);
      CommonOps_DDRM.addEquals(R1Expected, controller.getR());
      NativeCommonOps.invert(R1Expected, R1InverseExpected);

      CommonOps_DDRM.transpose(controller.getA(), A2Expected);
      CommonOps_DDRM.scale(-1.0, A2Expected);

      tempMatrix.reshape(3, 6);
      CommonOps_DDRM.multTransB(R1InverseExpected, controller.getB(), tempMatrix);
      CommonOps_DDRM.multAddTransA(NBExpected, tempMatrix, A2Expected);

      DMatrixRMaj tempMatrix2 = new DMatrixRMaj(6, 3);
      tempMatrix.reshape(6, 3);
      CommonOps_DDRM.multTransA(NBExpected, R1InverseExpected, tempMatrix);
      CommonOps_DDRM.transpose(controller.getC(), tempMatrix2);
      CommonOps_DDRM.multAdd(-1.0, tempMatrix, controller.getD(), tempMatrix2);
      CommonOps_DDRM.mult(2.0, tempMatrix2, controller.getQ(), B2Expected);

      solver.setA(A2Expected);
      solver.invert(A2InverseExpected);

//      EjmlUnitTests.assertEquals(NBExpected, controller.NB, epsilon);
//      EjmlUnitTests.assertEquals(A2Expected, controller.A2, epsilon);
//      EjmlUnitTests.assertEquals(B2Expected, controller.B2, epsilon);
//      EjmlUnitTests.assertEquals(A2InverseExpected, controller.A2Inverse, epsilon);


      DMatrixRMaj A2InverseB2 = new DMatrixRMaj(6, 3);
      DMatrixRMaj R1InvDQ = new DMatrixRMaj(3, 3);
      DMatrixRMaj R1InvDQInverse = new DMatrixRMaj(3, 3);
      DMatrixRMaj DQ = new DMatrixRMaj(3, 3);
      DMatrixRMaj R1InvBTrans = new DMatrixRMaj(3, 6);
      DMatrixRMaj k2Method1 = new DMatrixRMaj(3, 1);
      DMatrixRMaj k2Method2 = new DMatrixRMaj(3, 1);
      DMatrixRMaj yd = new DMatrixRMaj(3, 1);

      CommonOps_DDRM.mult(A2InverseExpected, B2Expected, A2InverseB2);
      CommonOps_DDRM.mult(controller.getD(), controller.getQ(), DQ);
      CommonOps_DDRM.mult(R1InverseExpected, DQ, R1InvDQ);
      CommonOps_DDRM.multTransB(R1InverseExpected, controller.getB(), R1InvBTrans);
      solver.setA(R1InvDQ);
      solver.invert(R1InvDQInverse);

//      assertEquals(2, controller.alphas.size());
//      assertEquals(2, controller.betas.size());
//      assertEquals(2, controller.gammas.size());
//      assertEquals(2, controller.betas.get(0).size());
//      assertEquals(2, controller.betas.get(1).size());
//      assertEquals(2, controller.gammas.get(0).size());
//      assertEquals(2, controller.gammas.get(1).size());

      DMatrixRMaj beta11Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj beta12Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj beta21Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj beta22Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj gamma11Expected = new DMatrixRMaj(3, 1);
      DMatrixRMaj gamma12Expected = new DMatrixRMaj(3, 1);
      DMatrixRMaj gamma21Expected = new DMatrixRMaj(3, 1);
      DMatrixRMaj gamma22Expected = new DMatrixRMaj(3, 1);
      DMatrixRMaj coefficients = new DMatrixRMaj(3, 1);
      DMatrixRMaj constructedCoefficient11 = new DMatrixRMaj(3, 1);
      DMatrixRMaj constructedCoefficient12 = new DMatrixRMaj(3, 1);
      DMatrixRMaj constructedCoefficient21 = new DMatrixRMaj(3, 1);
      DMatrixRMaj constructedCoefficient22 = new DMatrixRMaj(3, 1);


      vrpTrajectory2.getCoefficients(1, coefficients);
      CommonOps_DDRM.mult(-1, A2InverseB2, coefficients, beta22Expected);

      CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma22Expected);
      CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta22Expected, gamma22Expected);

      vrpTrajectory2.getCoefficients(0, coefficients);
      CommonOps_DDRM.subtractEquals(coefficients, finalPosition);
      CommonOps_DDRM.mult(A2InverseExpected, beta22Expected, beta21Expected);
      CommonOps_DDRM.multAdd(-1.0, A2InverseB2, coefficients, beta21Expected);

      CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma21Expected);
      CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta21Expected, gamma21Expected);

      tempMatrix.set(gamma21Expected);
      CommonOps_DDRM.multAdd(0.5, R1InvBTrans, beta21Expected, tempMatrix);
      CommonOps_DDRM.mult(R1InvDQInverse, tempMatrix, constructedCoefficient21);

      tempMatrix.set(gamma22Expected);
      CommonOps_DDRM.multAdd(0.5, R1InvBTrans, beta22Expected, tempMatrix);
      CommonOps_DDRM.mult(R1InvDQInverse, tempMatrix, constructedCoefficient22);

      EjmlUnitTests.assertEquals(beta21Expected, ((AlgebraicS2Segment) controller.getS2Segment(1)).getBeta(0), epsilon);
      EjmlUnitTests.assertEquals(beta22Expected, ((AlgebraicS2Segment) controller.getS2Segment(1)).getBeta(1), epsilon);
//      EjmlUnitTests.assertEquals(gamma21Expected, controller.gammas.get(1).get(0), epsilon);
//      EjmlUnitTests.assertEquals(gamma22Expected, controller.gammas.get(1).get(1), epsilon);

      relativeVRPTrajectories.get(1).getCoefficients(0, coefficients);
      MatrixTestTools.assertMatrixEquals(constructedCoefficient21, coefficients, epsilon);
      relativeVRPTrajectories.get(1).getCoefficients(1, coefficients);
      MatrixTestTools.assertMatrixEquals(constructedCoefficient22, coefficients, epsilon);



      vrpTrajectory1.getCoefficients(1, coefficients);
      CommonOps_DDRM.mult(-1, A2InverseB2, coefficients, beta12Expected);

      CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma12Expected);
      CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta12Expected, gamma12Expected);


      vrpTrajectory1.getCoefficients(0, coefficients);
      CommonOps_DDRM.subtractEquals(coefficients, finalPosition);
      CommonOps_DDRM.mult(A2InverseExpected, beta12Expected, beta11Expected);
      CommonOps_DDRM.multAdd(-1.0, A2InverseB2, coefficients, beta11Expected);

      CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma11Expected);
      CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta11Expected, gamma11Expected);


      tempMatrix.set(gamma11Expected);
      CommonOps_DDRM.multAdd(0.5, R1InvBTrans, beta11Expected, tempMatrix);
      CommonOps_DDRM.mult(R1InvDQInverse, tempMatrix, constructedCoefficient11);

      tempMatrix.set(gamma12Expected);
      CommonOps_DDRM.multAdd(0.5, R1InvBTrans, beta12Expected, tempMatrix);
      CommonOps_DDRM.mult(R1InvDQInverse, tempMatrix, constructedCoefficient12);


      EjmlUnitTests.assertEquals(beta11Expected, ((AlgebraicS2Segment) controller.getS2Segment(0)).getBeta(0), epsilon);
      EjmlUnitTests.assertEquals(beta12Expected, ((AlgebraicS2Segment) controller.getS2Segment(0)).getBeta(1), epsilon);
//      EjmlUnitTests.assertEquals(gamma11Expected, controller.gammas.get(0).get(0), epsilon);
//      EjmlUnitTests.assertEquals(gamma12Expected, controller.gammas.get(0).get(1), epsilon);

      relativeVRPTrajectories.get(0).getCoefficients(0, coefficients);
      MatrixTestTools.assertMatrixEquals(constructedCoefficient11, coefficients, epsilon);
      relativeVRPTrajectories.get(0).getCoefficients(1, coefficients);
      MatrixTestTools.assertMatrixEquals(constructedCoefficient12, coefficients, epsilon);


      MatrixExponentialCalculator matrixExponentialCalculator = new MatrixExponentialCalculator(6);

      DMatrixRMaj alpha2Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj alpha1Expected = new DMatrixRMaj(6, 1);

      DMatrixRMaj timeScaledDynamics = new DMatrixRMaj(6, 6);
      DMatrixRMaj matrixExponential = new DMatrixRMaj(6, 6);
      DMatrixRMaj betaSum = new DMatrixRMaj(6, 1);

      CommonOps_DDRM.scale((finalTime2 - finalTime1), A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime2 - finalTime1, 0), beta21Expected);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime2 - finalTime1, 1), beta22Expected);

      solver.setA(matrixExponential);
      solver.solve(betaSum, alpha2Expected);

      CommonOps_DDRM.scale(finalTime1, A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      betaSum.zero();
      CommonOps_DDRM.addEquals(betaSum, alpha2Expected);
      CommonOps_DDRM.addEquals(betaSum, beta21Expected);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime1, 0), beta11Expected);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime1, 1), beta12Expected);

      solver.setA(matrixExponential);
      solver.solve(betaSum, alpha1Expected);

      EjmlUnitTests.assertEquals(alpha2Expected, ((AlgebraicS2Segment) controller.getS2Segment(1)).getAlpha(), epsilon);
      EjmlUnitTests.assertEquals(alpha1Expected, ((AlgebraicS2Segment) controller.getS2Segment(0)).getAlpha(), epsilon);



      /*
      // check for continuity
       */
      DMatrixRMaj relativeMiddleVRP = new DMatrixRMaj(3, 1);
      vrpMiddle.get(relativeMiddleVRP);
      CommonOps_DDRM.subtractEquals(relativeMiddleVRP, finalPosition);

      DMatrixRMaj constructedPositionEndOf1 = new DMatrixRMaj(3, 1);
      DMatrixRMaj constructedPositionStartOf2 = new DMatrixRMaj(3, 1);
      DMatrixRMaj positionEndOf1 = new DMatrixRMaj(3, 1);
      DMatrixRMaj positionStartOf2 = new DMatrixRMaj(3, 1);

      relativeVRPTrajectories.get(0).getCoefficients(0, coefficients);
      CommonOps_DDRM.addEquals(constructedPositionEndOf1, MathTools.pow(finalTime1, 0), coefficients);
      relativeVRPTrajectories.get(0).getCoefficients(1, coefficients);
      CommonOps_DDRM.addEquals(constructedPositionEndOf1, MathTools.pow(finalTime1, 1), coefficients);

      relativeVRPTrajectories.get(0).compute(finalTime1);
      relativeVRPTrajectories.get(1).compute(0.0);
      relativeVRPTrajectories.get(0).getPosition().get(positionEndOf1);
      relativeVRPTrajectories.get(1).getPosition().get(positionStartOf2);


      relativeVRPTrajectories.get(1).getCoefficients(0, coefficients);
      CommonOps_DDRM.addEquals(constructedPositionStartOf2, MathTools.pow(0, 0), coefficients);
      relativeVRPTrajectories.get(1).getCoefficients(1, coefficients);
      CommonOps_DDRM.addEquals(constructedPositionStartOf2, MathTools.pow(0, 1), coefficients);

      DMatrixRMaj s2EndOf1 = new DMatrixRMaj(6, 1);
      DMatrixRMaj s2StartOf2 = new DMatrixRMaj(6, 1);

      DMatrixRMaj k2EndOf1Method1 = new DMatrixRMaj(3, 1);
      DMatrixRMaj k2EndOf1Method2 = new DMatrixRMaj(3, 1);
      DMatrixRMaj k2StartOf2Method1 = new DMatrixRMaj(3, 1);
      DMatrixRMaj k2StartOf2Method2 = new DMatrixRMaj(3, 1);

      DMatrixRMaj actualS2 = new DMatrixRMaj(6, 1);
      DMatrixRMaj actualk2 = new DMatrixRMaj(6, 1);

      controller.computeS1AndK1(finalTime1);
      controller.computeS2AndK2(finalTime1);
      actualk2.set(controller.getK2());
      actualS2.set(controller.getCostJacobian());

      // s2 at end of 1
      CommonOps_DDRM.scale(finalTime1, A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      CommonOps_DDRM.mult(matrixExponential, alpha1Expected, s2EndOf1);
      CommonOps_DDRM.addEquals(s2EndOf1, MathTools.pow(finalTime1, 0), beta11Expected);
      CommonOps_DDRM.addEquals(s2EndOf1, MathTools.pow(finalTime1, 1), beta12Expected);

      // k2 method 1 end of 1
      CommonOps_DDRM.mult(-0.5, R1InvBTrans, s2EndOf1, k2EndOf1Method1);
      CommonOps_DDRM.multAdd(R1InvDQ, relativeMiddleVRP, k2EndOf1Method1);

      // k2 method 2 end of 1
      CommonOps_DDRM.scale(finalTime1, A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      tempMatrix.reshape(3, 6);
      CommonOps_DDRM.mult(-0.5, R1InvBTrans, matrixExponential, tempMatrix);
      CommonOps_DDRM.mult(tempMatrix, alpha1Expected, k2EndOf1Method2);
      CommonOps_DDRM.addEquals(k2EndOf1Method2, MathTools.pow(finalTime1, 0), gamma11Expected);
      CommonOps_DDRM.addEquals(k2EndOf1Method2, MathTools.pow(finalTime1, 1), gamma12Expected);

      // s2 at start of 1
      CommonOps_DDRM.scale(0, A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      CommonOps_DDRM.mult(matrixExponential, alpha2Expected, s2StartOf2);
      CommonOps_DDRM.addEquals(s2StartOf2, MathTools.pow(0, 0), beta21Expected);
      CommonOps_DDRM.addEquals(s2StartOf2, MathTools.pow(0, 1), beta22Expected);

      // k2 method 1 start of 2
      CommonOps_DDRM.mult(-0.5, R1InvBTrans, s2StartOf2, k2StartOf2Method1);
      CommonOps_DDRM.multAdd(R1InvDQ, relativeMiddleVRP, k2StartOf2Method1);

      // k2 method 2 start of 2
      CommonOps_DDRM.scale(0.0, A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      CommonOps_DDRM.mult(-0.5, R1InvBTrans, matrixExponential, tempMatrix);
      CommonOps_DDRM.mult(tempMatrix, alpha2Expected, k2StartOf2Method2);
      CommonOps_DDRM.addEquals(k2StartOf2Method2, MathTools.pow(0, 0), gamma21Expected);
      CommonOps_DDRM.addEquals(k2StartOf2Method2, MathTools.pow(0, 1), gamma22Expected);


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
         controller.computeS1AndK1(time);
         controller.computeS2AndK2(time);

         CommonOps_DDRM.scale(time, A2Expected, timeScaledDynamics);
         matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);

         DMatrixRMaj s2Expected = new DMatrixRMaj(6, 1);
         CommonOps_DDRM.mult(matrixExponential, alpha1Expected, s2Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(time, 0), beta11Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(time, 1), beta12Expected);

//         EjmlUnitTests.assertEquals(timeScaledDynamics, controller.timeScaledA2, epsilon);
//         EjmlUnitTests.assertEquals(matrixExponential, controller.A2Exponential, epsilon);
//         EjmlUnitTests.assertEquals(betaSum, controller.summedBetas, epsilon);
         EjmlUnitTests.assertEquals(alpha1Expected, ((AlgebraicS2Segment) controller.getS2Segment(0)).getAlpha(), epsilon);

         EjmlUnitTests.assertEquals(s2Expected, controller.getCostJacobian(), epsilon);

         vrpTrajectory1.compute(time);
         vrpTrajectory1.getPosition().get(yd);
         CommonOps_DDRM.subtractEquals(yd, finalPosition);

         CommonOps_DDRM.mult(-0.5, R1InvBTrans, s2Expected, k2Method1);
         CommonOps_DDRM.multAdd(R1InvDQ, yd, k2Method1);

         tempMatrix.reshape(3, 6);
         CommonOps_DDRM.mult(-0.5, R1InvBTrans, matrixExponential, tempMatrix);
         CommonOps_DDRM.mult(tempMatrix, alpha1Expected, k2Method2);

         CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(time, 0), gamma11Expected);
         CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(time, 1), gamma12Expected);


         DMatrixRMaj K1Expected = new DMatrixRMaj(3, 6);
         CommonOps_DDRM.mult(-1.0, R1InverseExpected, NBExpected, K1Expected);

         EjmlUnitTests.assertEquals(k2Method1, k2Method2, epsilon);
         MatrixTestTools.assertMatrixEquals(k2Method1, controller.getK2(), epsilon);
         MatrixTestTools.assertMatrixEquals(k2Method2, controller.getK2(), epsilon);
         MatrixTestTools.assertMatrixEquals(K1Expected, controller.getK1(), epsilon);
      }

      // check the actual trajectory for every time step of the second trajectory
      for (double time = finalTime1; time <= finalTime2; time += 0.01)
      {
         double localTime = time - finalTime1;
         controller.computeS1AndK1(time);
         controller.computeS2AndK2(time);

         CommonOps_DDRM.scale(localTime, A2Expected, timeScaledDynamics);
         matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);

         DMatrixRMaj s2Expected = new DMatrixRMaj(6, 1);
         CommonOps_DDRM.mult(matrixExponential, alpha2Expected, s2Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(localTime, 0), beta21Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(localTime, 1), beta22Expected);

         EjmlUnitTests.assertEquals(alpha2Expected, ((AlgebraicS2Segment) controller.getS2Segment(1)).getAlpha(), epsilon);

         MatrixTestTools.assertMatrixEquals("Beta 1 calculation failed at time " + time, beta21Expected, ((AlgebraicS2Segment) controller.getS2Segment(1)).getBeta(0), epsilon);
         MatrixTestTools.assertMatrixEquals("Beta 2 calculation failed at time " + time, beta22Expected, ((AlgebraicS2Segment) controller.getS2Segment(1)).getBeta(1), epsilon);
         MatrixTestTools.assertMatrixEquals("S2 calculation failed at time " + time, s2Expected, controller.getCostJacobian(), epsilon);

         // validate the gammas
         DMatrixRMaj gamma21ExpectedAlt = new DMatrixRMaj(3, 1);
         DMatrixRMaj gamma22ExpectedAlt = new DMatrixRMaj(3, 1);

         vrpTrajectory2.getCoefficients(0, coefficients);
         CommonOps_DDRM.subtractEquals(coefficients, finalPosition);
         CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma21ExpectedAlt);
         CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta21Expected, gamma21ExpectedAlt);

         vrpTrajectory2.getCoefficients(1, coefficients);
         CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma22ExpectedAlt);
         CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta22Expected, gamma22ExpectedAlt);

//         MatrixTestTools.assertMatrixEquals(gamma21ExpectedAlt, controller.gammas.get(1).get(0), epsilon);
//         MatrixTestTools.assertMatrixEquals(gamma22ExpectedAlt, controller.gammas.get(1).get(1), epsilon);

         MatrixTestTools.assertMatrixEquals(gamma21ExpectedAlt, gamma21Expected, epsilon);
         MatrixTestTools.assertMatrixEquals(gamma22ExpectedAlt, gamma22Expected, epsilon);

         // validate the feedback solution
         vrpTrajectory2.compute(localTime);
         vrpTrajectory2.getPosition().get(yd);
         CommonOps_DDRM.subtractEquals(yd, finalPosition);

         CommonOps_DDRM.mult(-0.5, R1InvBTrans, s2Expected, k2Method1);
         CommonOps_DDRM.multAdd(R1InvDQ, yd, k2Method1);


         tempMatrix.reshape(3, 6);
         CommonOps_DDRM.scale(localTime, A2Expected, timeScaledDynamics);
         matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
         CommonOps_DDRM.mult(-0.5, R1InvBTrans, matrixExponential, tempMatrix);
         CommonOps_DDRM.mult(tempMatrix, alpha2Expected, k2Method2);
//         MatrixTestTools.assertMatrixEquals(matrixExponential, controller.A2Exponential, epsilon);

         CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(localTime, 0), gamma21ExpectedAlt);
         CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(localTime, 1), gamma22ExpectedAlt);

         DMatrixRMaj K1Expected = new DMatrixRMaj(3, 6);
         CommonOps_DDRM.mult(-1.0, R1InverseExpected, NBExpected, K1Expected);

         MatrixTestTools.assertMatrixEquals(k2Method1, k2Method2, epsilon);
         MatrixTestTools.assertMatrixEquals(k2Method1, controller.getK2(), epsilon);
         MatrixTestTools.assertMatrixEquals(K1Expected, controller.getK1(), epsilon);
      }

      controller.computeS1AndK1(finalTime2);
      controller.computeS2AndK2(finalTime2);

      // should be zero at tf
      DMatrixRMaj zeroMatrix = new DMatrixRMaj(6, 1);
      EjmlUnitTests.assertEquals(zeroMatrix, controller.getCostJacobian(), epsilon);
   }

   @Test
   public void testComputingS2FromThreeCubicTrajectories()
   {
      LQRJumpMomentumController controller = new LQRJumpMomentumController(() -> omega, 1.0);

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

      List<SettableContactStateProvider> contactStateProviders = new ArrayList<>();
      contactStateProviders.add(new SettableContactStateProvider());
      contactStateProviders.add(new SettableContactStateProvider());
      contactStateProviders.add(new SettableContactStateProvider());
      contactStateProviders.get(0).setContactState(ContactState.IN_CONTACT);
      contactStateProviders.get(1).setContactState(ContactState.IN_CONTACT);
      contactStateProviders.get(2).setContactState(ContactState.IN_CONTACT);

      DMatrixRMaj finalPosition = new DMatrixRMaj(3, 1);
      vrpEnd.get(finalPosition);

      controller.setVRPTrajectory(trajectories, contactStateProviders);

      // double up on the computation to make sure things get zeroed properly between calls
      controller.computeS1Segments();
      controller.computeS1Segments();
      controller.computeS2Segments();
      controller.computeS2Segments();


      DMatrixRMaj NExpected = new DMatrixRMaj(6, 3);
      DMatrixRMaj NBExpected = new DMatrixRMaj(3, 6);
      DMatrixRMaj NTransposeExpected = new DMatrixRMaj(3, 6);
      DMatrixRMaj tempMatrix = new DMatrixRMaj(3, 3);
      CommonOps_DDRM.mult(controller.getQ(), controller.getD(), tempMatrix);
      CommonOps_DDRM.multTransA(controller.getC(), tempMatrix, NExpected);
      CommonOps_DDRM.transpose(NExpected, NTransposeExpected);

      CommonOps_DDRM.multTransA(controller.getB(), controller.getCostHessian(), NBExpected);
      CommonOps_DDRM.addEquals(NBExpected, NTransposeExpected);

      LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.general(0, 0);

      DMatrixRMaj A2Expected = new DMatrixRMaj(6, 6);
      DMatrixRMaj A2InverseExpected = new DMatrixRMaj(6, 6);
      DMatrixRMaj B2Expected = new DMatrixRMaj(6, 3);
      DMatrixRMaj R1Expected = new DMatrixRMaj(3, 3);
      DMatrixRMaj R1InverseExpected = new DMatrixRMaj(3, 3);

      NativeCommonOps.multQuad(controller.getD(), controller.getQ(), R1Expected);
      CommonOps_DDRM.addEquals(R1Expected, controller.getR());
      NativeCommonOps.invert(R1Expected, R1InverseExpected);

      CommonOps_DDRM.transpose(controller.getA(), A2Expected);
      CommonOps_DDRM.scale(-1.0, A2Expected);

      tempMatrix.reshape(3, 6);
      CommonOps_DDRM.multTransB(R1InverseExpected, controller.getB(), tempMatrix);
      CommonOps_DDRM.multAddTransA(NBExpected, tempMatrix, A2Expected);

      DMatrixRMaj tempMatrix2 = new DMatrixRMaj(6, 3);
      tempMatrix.reshape(6, 3);
      CommonOps_DDRM.multTransA(NBExpected, R1InverseExpected, tempMatrix);
      CommonOps_DDRM.transpose(controller.getC(), tempMatrix2);
      CommonOps_DDRM.multAdd(-1.0, tempMatrix, controller.getD(), tempMatrix2);
      CommonOps_DDRM.mult(2.0, tempMatrix2, controller.getQ(), B2Expected);

      solver.setA(A2Expected);
      solver.invert(A2InverseExpected);

//      EjmlUnitTests.assertEquals(NBExpected, controller.NB, epsilon);
//      EjmlUnitTests.assertEquals(A2Expected, controller.A2, epsilon);
//      EjmlUnitTests.assertEquals(B2Expected, controller.B2, epsilon);
//      EjmlUnitTests.assertEquals(A2InverseExpected, controller.A2Inverse, epsilon);


      DMatrixRMaj A2InverseB2 = new DMatrixRMaj(6, 3);
      DMatrixRMaj R1InvDQ = new DMatrixRMaj(3, 3);
      DMatrixRMaj DQ = new DMatrixRMaj(3, 3);
      DMatrixRMaj R1InvBTrans = new DMatrixRMaj(3, 6);
      DMatrixRMaj k2Method1 = new DMatrixRMaj(3, 1);
      DMatrixRMaj k2Method2 = new DMatrixRMaj(3, 1);
      DMatrixRMaj yd = new DMatrixRMaj(3, 1);

      CommonOps_DDRM.mult(A2InverseExpected, B2Expected, A2InverseB2);
      CommonOps_DDRM.mult(controller.getD(), controller.getQ(), DQ);
      CommonOps_DDRM.mult(R1InverseExpected, DQ, R1InvDQ);
      CommonOps_DDRM.multTransB(R1InverseExpected, controller.getB(), R1InvBTrans);

//      assertEquals(3, controller.alphas.size());
//      assertEquals(3, controller.betas.size());
//      assertEquals(3, controller.gammas.size());
//      assertEquals(4, controller.betas.get(0).size());
//      assertEquals(4, controller.betas.get(1).size());
//      assertEquals(4, controller.betas.get(2).size());
//      assertEquals(4, controller.gammas.get(0).size());
//      assertEquals(4, controller.gammas.get(1).size());
//      assertEquals(4, controller.gammas.get(2).size());

      DMatrixRMaj beta11Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj beta12Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj beta13Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj beta14Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj beta21Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj beta22Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj beta23Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj beta24Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj beta31Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj beta32Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj beta33Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj beta34Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj gamma11Expected = new DMatrixRMaj(3, 1);
      DMatrixRMaj gamma12Expected = new DMatrixRMaj(3, 1);
      DMatrixRMaj gamma13Expected = new DMatrixRMaj(3, 1);
      DMatrixRMaj gamma14Expected = new DMatrixRMaj(3, 1);
      DMatrixRMaj gamma21Expected = new DMatrixRMaj(3, 1);
      DMatrixRMaj gamma22Expected = new DMatrixRMaj(3, 1);
      DMatrixRMaj gamma23Expected = new DMatrixRMaj(3, 1);
      DMatrixRMaj gamma24Expected = new DMatrixRMaj(3, 1);
      DMatrixRMaj gamma31Expected = new DMatrixRMaj(3, 1);
      DMatrixRMaj gamma32Expected = new DMatrixRMaj(3, 1);
      DMatrixRMaj gamma33Expected = new DMatrixRMaj(3, 1);
      DMatrixRMaj gamma34Expected = new DMatrixRMaj(3, 1);
      DMatrixRMaj coefficients = new DMatrixRMaj(3, 1);

      vrpTrajectory3.getCoefficients(3, coefficients);
      CommonOps_DDRM.mult(-1, A2InverseB2, coefficients, beta34Expected);

      CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma34Expected);
      CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta34Expected, gamma34Expected);

      vrpTrajectory3.getCoefficients(2, coefficients);
      CommonOps_DDRM.mult(3, A2InverseExpected, beta34Expected, beta33Expected);
      CommonOps_DDRM.multAdd(-1.0, A2InverseB2, coefficients, beta33Expected);

      CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma33Expected);
      CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta33Expected, gamma33Expected);

      vrpTrajectory3.getCoefficients(1, coefficients);
      CommonOps_DDRM.mult(2, A2InverseExpected, beta33Expected, beta32Expected);
      CommonOps_DDRM.multAdd(-1.0, A2InverseB2, coefficients, beta32Expected);

      CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma32Expected);
      CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta32Expected, gamma32Expected);

      vrpTrajectory3.getCoefficients(0, coefficients);
      CommonOps_DDRM.subtractEquals(coefficients, finalPosition);
      CommonOps_DDRM.mult(A2InverseExpected, beta32Expected, beta31Expected);
      CommonOps_DDRM.multAdd(-1.0, A2InverseB2, coefficients, beta31Expected);

      CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma31Expected);
      CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta31Expected, gamma31Expected);

      EjmlUnitTests.assertEquals(beta34Expected, ((AlgebraicS2Segment) controller.getS2Segment(2)).getBeta(3), epsilon);
      EjmlUnitTests.assertEquals(beta33Expected, ((AlgebraicS2Segment) controller.getS2Segment(2)).getBeta(2), epsilon);
      EjmlUnitTests.assertEquals(beta32Expected, ((AlgebraicS2Segment) controller.getS2Segment(2)).getBeta(1), epsilon);
      EjmlUnitTests.assertEquals(beta31Expected, ((AlgebraicS2Segment) controller.getS2Segment(2)).getBeta(0), epsilon);
//      EjmlUnitTests.assertEquals(gamma34Expected, controller.gammas.get(2).get(3), epsilon);
//      EjmlUnitTests.assertEquals(gamma33Expected, controller.gammas.get(2).get(2), epsilon);
//      EjmlUnitTests.assertEquals(gamma32Expected, controller.gammas.get(2).get(1), epsilon);
//      EjmlUnitTests.assertEquals(gamma31Expected, controller.gammas.get(2).get(0), epsilon);


      vrpTrajectory2.getCoefficients(3, coefficients);
      CommonOps_DDRM.mult(-1, A2InverseB2, coefficients, beta24Expected);

      CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma24Expected);
      CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta24Expected, gamma24Expected);

      vrpTrajectory2.getCoefficients(2, coefficients);
      CommonOps_DDRM.mult(3, A2InverseExpected, beta24Expected, beta23Expected);
      CommonOps_DDRM.multAdd(-1.0, A2InverseB2, coefficients, beta23Expected);

      CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma23Expected);
      CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta23Expected, gamma23Expected);

      vrpTrajectory2.getCoefficients(1, coefficients);
      CommonOps_DDRM.mult(2, A2InverseExpected, beta23Expected, beta22Expected);
      CommonOps_DDRM.multAdd(-1.0, A2InverseB2, coefficients, beta22Expected);

      CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma22Expected);
      CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta22Expected, gamma22Expected);

      vrpTrajectory2.getCoefficients(0, coefficients);
      CommonOps_DDRM.subtractEquals(coefficients, finalPosition);
      CommonOps_DDRM.mult(A2InverseExpected, beta22Expected, beta21Expected);
      CommonOps_DDRM.multAdd(-1.0, A2InverseB2, coefficients, beta21Expected);

      CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma21Expected);
      CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta21Expected, gamma21Expected);

      EjmlUnitTests.assertEquals(beta24Expected, ((AlgebraicS2Segment) controller.getS2Segment(1)).getBeta(3), epsilon);
      EjmlUnitTests.assertEquals(beta23Expected, ((AlgebraicS2Segment) controller.getS2Segment(1)).getBeta(2), epsilon);
      EjmlUnitTests.assertEquals(beta22Expected, ((AlgebraicS2Segment) controller.getS2Segment(1)).getBeta(1), epsilon);
      EjmlUnitTests.assertEquals(beta21Expected, ((AlgebraicS2Segment) controller.getS2Segment(1)).getBeta(0), epsilon);
//      EjmlUnitTests.assertEquals(gamma24Expected, controller.gammas.get(1).get(3), epsilon);
//      EjmlUnitTests.assertEquals(gamma23Expected, controller.gammas.get(1).get(2), epsilon);
//      EjmlUnitTests.assertEquals(gamma22Expected, controller.gammas.get(1).get(1), epsilon);
//      EjmlUnitTests.assertEquals(gamma21Expected, controller.gammas.get(1).get(0), epsilon);

      vrpTrajectory1.getCoefficients(3, coefficients);
      CommonOps_DDRM.mult(-1, A2InverseB2, coefficients, beta14Expected);

      CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma14Expected);
      CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta14Expected, gamma14Expected);

      vrpTrajectory1.getCoefficients(2, coefficients);
      CommonOps_DDRM.mult(3, A2InverseExpected, beta14Expected, beta13Expected);
      CommonOps_DDRM.multAdd(-1.0, A2InverseB2, coefficients, beta13Expected);

      CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma13Expected);
      CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta13Expected, gamma13Expected);

      vrpTrajectory1.getCoefficients(1, coefficients);
      CommonOps_DDRM.mult(2, A2InverseExpected, beta13Expected, beta12Expected);
      CommonOps_DDRM.multAdd(-1.0, A2InverseB2, coefficients, beta12Expected);

      CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma12Expected);
      CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta12Expected, gamma12Expected);

      vrpTrajectory1.getCoefficients(0, coefficients);
      CommonOps_DDRM.subtractEquals(coefficients, finalPosition);
      CommonOps_DDRM.mult(A2InverseExpected, beta12Expected, beta11Expected);
      CommonOps_DDRM.multAdd(-1.0, A2InverseB2, coefficients, beta11Expected);

      CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma11Expected);
      CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta11Expected, gamma11Expected);

      EjmlUnitTests.assertEquals(beta14Expected, ((AlgebraicS2Segment) controller.getS2Segment(0)).getBeta(3), epsilon);
      EjmlUnitTests.assertEquals(beta13Expected, ((AlgebraicS2Segment) controller.getS2Segment(0)).getBeta(2), epsilon);
      EjmlUnitTests.assertEquals(beta12Expected, ((AlgebraicS2Segment) controller.getS2Segment(0)).getBeta(1), epsilon);
      EjmlUnitTests.assertEquals(beta11Expected, ((AlgebraicS2Segment) controller.getS2Segment(0)).getBeta(0), epsilon);
      //      EjmlUnitTests.assertEquals(gamma14Expected, controller.gammas.get(0).get(3), epsilon);
      //      EjmlUnitTests.assertEquals(gamma13Expected, controller.gammas.get(0).get(2), epsilon);
      //      EjmlUnitTests.assertEquals(gamma12Expected, controller.gammas.get(0).get(1), epsilon);
//      EjmlUnitTests.assertEquals(gamma11Expected, controller.gammas.get(0).get(0), epsilon);

      MatrixExponentialCalculator matrixExponentialCalculator = new MatrixExponentialCalculator(6);

      DMatrixRMaj alpha3Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj alpha2Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj alpha1Expected = new DMatrixRMaj(6, 1);

      DMatrixRMaj timeScaledDynamics = new DMatrixRMaj(6, 6);
      DMatrixRMaj matrixExponential = new DMatrixRMaj(6, 6);
      DMatrixRMaj betaSum = new DMatrixRMaj(6, 1);

      CommonOps_DDRM.scale((finalTime3 - finalTime2), A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime3 - finalTime2, 0), beta31Expected);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime3 - finalTime2, 1), beta32Expected);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime3 - finalTime2, 2), beta33Expected);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime3 - finalTime2, 3), beta34Expected);

      solver.setA(matrixExponential);
      solver.solve(betaSum, alpha3Expected);

      CommonOps_DDRM.scale((finalTime2 - finalTime1), A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      betaSum.zero();
      CommonOps_DDRM.addEquals(betaSum, alpha3Expected);
      CommonOps_DDRM.addEquals(betaSum, beta31Expected);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime2 - finalTime1, 0), beta21Expected);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime2 - finalTime1, 1), beta22Expected);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime2 - finalTime1, 2), beta23Expected);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime2 - finalTime1, 3), beta24Expected);

      solver.setA(matrixExponential);
      solver.solve(betaSum, alpha2Expected);

      CommonOps_DDRM.scale(finalTime1, A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      betaSum.zero();
      CommonOps_DDRM.addEquals(betaSum, alpha2Expected);
      CommonOps_DDRM.addEquals(betaSum, beta21Expected);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime1, 0), beta11Expected);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime1, 1), beta12Expected);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime1, 2), beta13Expected);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime1, 3), beta14Expected);

      solver.setA(matrixExponential);
      solver.solve(betaSum, alpha1Expected);

      EjmlUnitTests.assertEquals(alpha3Expected, ((AlgebraicS2Segment) controller.getS2Segment(2)).getAlpha(), epsilon);
      EjmlUnitTests.assertEquals(alpha2Expected, ((AlgebraicS2Segment) controller.getS2Segment(1)).getAlpha(), epsilon);
      EjmlUnitTests.assertEquals(alpha1Expected, ((AlgebraicS2Segment) controller.getS2Segment(0)).getAlpha(), epsilon);



      // check for continuity

      DMatrixRMaj s2EndOf1 = new DMatrixRMaj(6, 1);
      DMatrixRMaj s2StartOf2 = new DMatrixRMaj(6, 1);
      DMatrixRMaj s2EndOf2 = new DMatrixRMaj(6, 1);
      DMatrixRMaj s2StartOf3 = new DMatrixRMaj(6, 1);

      CommonOps_DDRM.scale(finalTime1, A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      CommonOps_DDRM.mult(matrixExponential, alpha1Expected, s2EndOf1);
      CommonOps_DDRM.addEquals(s2EndOf1, MathTools.pow(finalTime1, 0), beta11Expected);
      CommonOps_DDRM.addEquals(s2EndOf1, MathTools.pow(finalTime1, 1), beta12Expected);
      CommonOps_DDRM.addEquals(s2EndOf1, MathTools.pow(finalTime1, 2), beta13Expected);
      CommonOps_DDRM.addEquals(s2EndOf1, MathTools.pow(finalTime1, 3), beta14Expected);

      CommonOps_DDRM.scale(0, A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      CommonOps_DDRM.mult(matrixExponential, alpha2Expected, s2StartOf2);
      CommonOps_DDRM.addEquals(s2StartOf2, MathTools.pow(0, 0), beta21Expected);
      CommonOps_DDRM.addEquals(s2StartOf2, MathTools.pow(0, 1), beta22Expected);
      CommonOps_DDRM.addEquals(s2StartOf2, MathTools.pow(0, 2), beta23Expected);
      CommonOps_DDRM.addEquals(s2StartOf2, MathTools.pow(0, 2), beta24Expected);

      EjmlUnitTests.assertEquals(s2EndOf1, s2StartOf2, epsilon);


      CommonOps_DDRM.scale(finalTime2 - finalTime1, A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      CommonOps_DDRM.mult(matrixExponential, alpha2Expected, s2EndOf2);
      CommonOps_DDRM.addEquals(s2EndOf2, MathTools.pow(finalTime2 - finalTime1, 0), beta21Expected);
      CommonOps_DDRM.addEquals(s2EndOf2, MathTools.pow(finalTime2 - finalTime1, 1), beta22Expected);
      CommonOps_DDRM.addEquals(s2EndOf2, MathTools.pow(finalTime2 - finalTime1, 2), beta23Expected);
      CommonOps_DDRM.addEquals(s2EndOf2, MathTools.pow(finalTime2 - finalTime1, 3), beta24Expected);

      CommonOps_DDRM.scale(0, A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      CommonOps_DDRM.mult(matrixExponential, alpha3Expected, s2StartOf3);
      CommonOps_DDRM.addEquals(s2StartOf3, MathTools.pow(0, 0), beta31Expected);
      CommonOps_DDRM.addEquals(s2StartOf3, MathTools.pow(0, 1), beta32Expected);
      CommonOps_DDRM.addEquals(s2StartOf3, MathTools.pow(0, 2), beta33Expected);
      CommonOps_DDRM.addEquals(s2StartOf3, MathTools.pow(0, 3), beta34Expected);

      EjmlUnitTests.assertEquals(s2EndOf2, s2StartOf3, epsilon);

      DMatrixRMaj finalY = new DMatrixRMaj(3, 1);
      vrpEnd.get(finalY);

      // check trajectory

      for (double time = 0; time <= finalTime1; time += 0.01)
      {
         controller.computeS1AndK1(time);
         controller.computeS2AndK2(time);

         CommonOps_DDRM.scale(time, A2Expected, timeScaledDynamics);
         matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);

         DMatrixRMaj s2Expected = new DMatrixRMaj(6, 1);
         CommonOps_DDRM.mult(matrixExponential, alpha1Expected, s2Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(time, 0), beta11Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(time, 1), beta12Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(time, 2), beta13Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(time, 3), beta14Expected);

//         EjmlUnitTests.assertEquals(timeScaledDynamics, controller.timeScaledA2, epsilon);
//         EjmlUnitTests.assertEquals(matrixExponential, controller.A2Exponential, epsilon);
//         EjmlUnitTests.assertEquals(betaSum, controller.summedBetas, epsilon);
         EjmlUnitTests.assertEquals(alpha1Expected, ((AlgebraicS2Segment) controller.getS2Segment(0)).getAlpha(), epsilon);

         EjmlUnitTests.assertEquals(s2Expected, controller.getCostJacobian(), epsilon);

         vrpTrajectory1.compute(time);
         vrpTrajectory1.getPosition().get(yd);
         CommonOps_DDRM.subtractEquals(yd, finalY);

         CommonOps_DDRM.mult(-0.5, R1InvBTrans, s2Expected, k2Method1);
         CommonOps_DDRM.multAdd(R1InvDQ, yd, k2Method1);

         tempMatrix.reshape(3, 6);
         CommonOps_DDRM.mult(-0.5, R1InvBTrans, matrixExponential, tempMatrix);
         CommonOps_DDRM.mult(tempMatrix, alpha1Expected, k2Method2);

         CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(time, 0), gamma11Expected);
         CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(time, 1), gamma12Expected);
         CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(time, 2), gamma13Expected);
         CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(time, 3), gamma14Expected);

         DMatrixRMaj K1Expected = new DMatrixRMaj(3, 6);
         CommonOps_DDRM.mult(-1.0, R1InverseExpected, NBExpected, K1Expected);

         EjmlUnitTests.assertEquals(k2Method1, k2Method2, epsilon);
         MatrixTestTools.assertMatrixEquals(k2Method1, controller.getK2(), epsilon);
         MatrixTestTools.assertMatrixEquals(K1Expected, controller.getK1(), epsilon);
      }

      for (double time = finalTime1; time <= finalTime2; time += 0.01)
      {
         int j = 1;
         double localTime = time - finalTime1;
         controller.computeS1AndK1(time);
         controller.computeS2AndK2(time);

         CommonOps_DDRM.scale(localTime, A2Expected, timeScaledDynamics);
         matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);

         DMatrixRMaj s2Expected = new DMatrixRMaj(6, 1);
         CommonOps_DDRM.mult(matrixExponential, alpha2Expected, s2Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(localTime, 0), beta21Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(localTime, 1), beta22Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(localTime, 2), beta23Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(localTime, 3), beta24Expected);

         EjmlUnitTests.assertEquals(alpha2Expected, ((AlgebraicS2Segment) controller.getS2Segment(1)).getAlpha(), epsilon);

         EjmlUnitTests.assertEquals(s2Expected, controller.getCostJacobian(), epsilon);

         vrpTrajectory2.compute(localTime);
         vrpTrajectory2.getPosition().get(yd);
         CommonOps_DDRM.subtractEquals(yd, finalY);

         CommonOps_DDRM.mult(-0.5, R1InvBTrans, s2Expected, k2Method1);
         CommonOps_DDRM.multAdd(R1InvDQ, yd, k2Method1);

         tempMatrix.reshape(3, 6);
         CommonOps_DDRM.mult(-0.5, R1InvBTrans, matrixExponential, tempMatrix);
         CommonOps_DDRM.mult(tempMatrix, alpha2Expected, k2Method2);

         CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(localTime, 0), gamma21Expected);
         CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(localTime, 1), gamma22Expected);
         CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(localTime, 2), gamma23Expected);
         CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(localTime, 3), gamma24Expected);

         DMatrixRMaj K1Expected = new DMatrixRMaj(3, 6);
         CommonOps_DDRM.mult(-1.0, R1InverseExpected, NBExpected, K1Expected);

         MatrixTestTools.assertMatrixEquals(k2Method1, controller.getK2(), epsilon);
         EjmlUnitTests.assertEquals(k2Method1, k2Method2, epsilon);
         MatrixTestTools.assertMatrixEquals(K1Expected, controller.getK1(), epsilon);
      }

      for (double time = finalTime2; time <= finalTime3; time += 0.01)
      {
         double localTime = time - finalTime2;
         controller.computeS1AndK1(time);
         controller.computeS2AndK2(time);

         CommonOps_DDRM.scale(localTime, A2Expected, timeScaledDynamics);
         matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);

         DMatrixRMaj s2Expected = new DMatrixRMaj(6, 1);
         CommonOps_DDRM.mult(matrixExponential, alpha3Expected, s2Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(localTime, 0), beta31Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(localTime, 1), beta32Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(localTime, 2), beta33Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(localTime, 3), beta34Expected);

         EjmlUnitTests.assertEquals(alpha3Expected, ((AlgebraicS2Segment) controller.getS2Segment(2)).getAlpha(), epsilon);

         EjmlUnitTests.assertEquals(s2Expected, controller.getCostJacobian(), epsilon);

         vrpTrajectory3.compute(localTime);
         vrpTrajectory3.getPosition().get(yd);
         CommonOps_DDRM.subtractEquals(yd, finalY);

         CommonOps_DDRM.mult(-0.5, R1InvBTrans, s2Expected, k2Method1);
         CommonOps_DDRM.multAdd(R1InvDQ, yd, k2Method1);

         tempMatrix.reshape(3, 6);
         CommonOps_DDRM.mult(-0.5, R1InvBTrans, matrixExponential, tempMatrix);
         CommonOps_DDRM.mult(tempMatrix, alpha3Expected, k2Method2);

         CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(localTime, 0), gamma31Expected);
         CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(localTime, 1), gamma32Expected);
         CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(localTime, 2), gamma33Expected);
         CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(localTime, 3), gamma34Expected);

         DMatrixRMaj K1Expected = new DMatrixRMaj(3, 6);
         CommonOps_DDRM.mult(-1.0, R1InverseExpected, NBExpected, K1Expected);

         EjmlUnitTests.assertEquals(k2Method1, k2Method2, epsilon);
         MatrixTestTools.assertMatrixEquals(k2Method1, controller.getK2(), epsilon);
         MatrixTestTools.assertMatrixEquals(K1Expected, controller.getK1(), epsilon);
      }

      controller.computeS1AndK1(finalTime3);
      controller.computeS2AndK2(finalTime3);

      // should be zero at tf
      DMatrixRMaj zeroMatrix = new DMatrixRMaj(6, 1);
      EjmlUnitTests.assertEquals(zeroMatrix, controller.getCostJacobian(), epsilon);
   }

   /*
   @Test
   public void testBasicTrajectoryTracking()
   {
      SimpleCoMTrajectoryPlanner planner = new SimpleCoMTrajectoryPlanner(() -> omega);
      LQRJumpMomentumController controller = new LQRJumpMomentumController(() -> omega);

      List<SettableContactStateProvider> contactSequence = new ArrayList<>();

      SettableContactStateProvider firstContact = new SettableContactStateProvider();
      SettableContactStateProvider secondContact = new SettableContactStateProvider();
      SettableContactStateProvider thirdContact = new SettableContactStateProvider();

      FramePoint2D vrpStart = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.0, 0.0);
      FramePoint2D vrpMiddle = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.6, 0.75);
      FramePoint2D vrpMiddle2 = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.79, 0.88);
      FramePoint2D vrpEnd = new FramePoint2D(ReferenceFrame.getWorldFrame(), 1.0, 0.5);

      double finalTime1 = 0.31;
      double finalTime2 = 0.65;
      double finalTime3 = 1.0;

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

      DMatrixRMaj currentState = new DMatrixRMaj(6, 1);
      comPosition.get(currentState);
      comVelocity.get(3, currentState);
      controller.computeControlInput(currentState, time);


      for (; time <= finalTime1; time += dt)
      {
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

    */
}
