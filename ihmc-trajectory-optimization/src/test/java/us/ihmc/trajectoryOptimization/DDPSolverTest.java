package us.ihmc.trajectoryOptimization;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.RandomMatrices_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.matrixlib.MatrixTestTools;

public class DDPSolverTest
{
   @Test
   public void testComputeUpdatedControl()
   {
      TestDynamics dynamics = new TestDynamics();
      DDPSolver<DefaultDiscreteState> calculator = new DDPSolver<>(dynamics);

      DMatrixRMaj feedforwardTerm = new DMatrixRMaj(3, 1);

      DMatrixRMaj currentState = new DMatrixRMaj(6, 1);
      DMatrixRMaj currentControl = new DMatrixRMaj(3, 1);

      DMatrixRMaj updatedState = new DMatrixRMaj(6, 1);
      DMatrixRMaj updatedControl = new DMatrixRMaj(3, 1);

      DMatrixRMaj updatedControlExpected = new DMatrixRMaj(3, 1);

      DMatrixRMaj gainMatrix = new DMatrixRMaj(3, 6);

      double xyPositionGain = 2.0;
      double zPositionGain = 5.0;
      double xyDotPositionGain = 5.0;
      double zDotPositionGain = 8.0;

      double currentX = 0.7;
      double currentY = 0.25;
      double currentZ = 1.0;
      double currentXDot = 1.5;
      double currentYDot = 0.5;
      double currentZDot = 0.5;

      double updatedX = 1.0;
      double updatedY = 0.2;
      double updatedZ = 1.1;
      double updatedXDot = 1.2;
      double updatedYDot = 0.3;
      double updatedZDot = 0.1;

      double pX = 0.5;
      double pY = 0.26;
      double fZ = 100.0;

      double feedforwardX = 0.4;
      double feedforwardY = 0.2;
      double feedforwardZ = 30.0;

      currentState.set(0, currentX);
      currentState.set(1, currentY);
      currentState.set(2, currentZ);
      currentState.set(3, currentXDot);
      currentState.set(4, currentYDot);
      currentState.set(5, currentZDot);

      updatedState.set(0, updatedX);
      updatedState.set(1, updatedY);
      updatedState.set(2, updatedZ);
      updatedState.set(3, updatedXDot);
      updatedState.set(4, updatedYDot);
      updatedState.set(5, updatedZDot);

      gainMatrix.set(0, 0, xyPositionGain);
      gainMatrix.set(0, 3, xyDotPositionGain);
      gainMatrix.set(1, 1, xyPositionGain);
      gainMatrix.set(1, 4, xyDotPositionGain);
      gainMatrix.set(2, 2, zPositionGain);
      gainMatrix.set(2, 5, zDotPositionGain);

      currentControl.set(0, pX);
      currentControl.set(1, pY);
      currentControl.set(2, fZ);

      feedforwardTerm.set(0, feedforwardX);
      feedforwardTerm.set(1, feedforwardY);
      feedforwardTerm.set(2, feedforwardZ);

      double updatedPx = pX + feedforwardX + xyPositionGain * (updatedX - currentX) + xyDotPositionGain * (updatedXDot - currentXDot);
      double updatedPy = pY + feedforwardY + xyPositionGain * (updatedY - currentY) + xyDotPositionGain * (updatedYDot - currentYDot);
      double updatedFz = fZ + feedforwardZ + zPositionGain * (updatedZ - currentZ) + zDotPositionGain * (updatedZDot - currentZDot);

      updatedControlExpected.set(0, updatedPx);
      updatedControlExpected.set(1, updatedPy);
      updatedControlExpected.set(2, updatedFz);

      calculator.computeUpdatedControl(currentState, updatedState, gainMatrix, feedforwardTerm, currentControl, updatedControl);

      DMatrixRMaj stateError = new DMatrixRMaj(6, 1);
      DMatrixRMaj updatedControlAlternative = new DMatrixRMaj(3, 1);

      CommonOps_DDRM.subtract(updatedState, currentState, stateError);
      CommonOps_DDRM.add(currentControl, feedforwardTerm, updatedControlAlternative);
      CommonOps_DDRM.multAdd(gainMatrix, stateError, updatedControlAlternative);

      MatrixTestTools.assertMatrixEquals(updatedControlAlternative, updatedControl, 1e-12);
      MatrixTestTools.assertMatrixEquals(updatedControlExpected, updatedControl, 1e-12);
   }

   @Test
   public void testUpdateHamiltonianApproximations()
   {
      Random random = new Random(1738);
      DMatrixRMaj V_X = RandomMatrices_DDRM.rectangle(6, 1, random);
      DMatrixRMaj V_XX = RandomMatrices_DDRM.symmetric(6, -1000, 1000, random);

      DMatrixRMaj L_X = RandomMatrices_DDRM.rectangle(6, 1, random);
      DMatrixRMaj L_U = RandomMatrices_DDRM.rectangle(3, 1, random);
      DMatrixRMaj L_XX = RandomMatrices_DDRM.rectangle(6, 6, random);
      DMatrixRMaj L_XU = RandomMatrices_DDRM.rectangle(6, 3, random);
      DMatrixRMaj L_UX = new DMatrixRMaj(3, 6);
      DMatrixRMaj L_UU = RandomMatrices_DDRM.rectangle(3, 3, random);
      CommonOps_DDRM.transpose(L_XU, L_UX);

      DMatrixRMaj f_X = RandomMatrices_DDRM.rectangle(6, 6, random);
      DMatrixRMaj f_U = RandomMatrices_DDRM.rectangle(6, 3, random);

      DMatrixRMaj Qx = new DMatrixRMaj(6, 1);
      DMatrixRMaj Qu = new DMatrixRMaj(3, 1);
      DMatrixRMaj Qxx = new DMatrixRMaj(6, 6);
      DMatrixRMaj Quu = new DMatrixRMaj(3, 3);
      DMatrixRMaj Qux = new DMatrixRMaj(3, 6);
      DMatrixRMaj Qxu = new DMatrixRMaj(6, 3);
      DMatrixRMaj QuExpected = new DMatrixRMaj(3, 1);
      DMatrixRMaj QxExpected = new DMatrixRMaj(6, 1);
      DMatrixRMaj QxxExpected = new DMatrixRMaj(6, 6);
      DMatrixRMaj QuuExpected = new DMatrixRMaj(3, 3);
      DMatrixRMaj QuxExpected = new DMatrixRMaj(3, 6);
      DMatrixRMaj QxuExpected = new DMatrixRMaj(6, 3);

      TestDynamics dynamics = new TestDynamics();
      DDPSolver<DefaultDiscreteState> calculator = new DDPSolver<>(dynamics);

      calculator.updateHamiltonianApproximations(DefaultDiscreteState.DEFAULT, 0, L_X, L_U, L_XX, L_UU, L_XU, f_X, f_U, V_X, V_XX, Qx, Qu, Qxx, Quu, Qxu, Qux);

      QxExpected.set(L_X);
      CommonOps_DDRM.multAddTransA(f_X, V_X, QxExpected);

      QuExpected.set(L_U);
      CommonOps_DDRM.multAddTransA(f_U, V_X, QuExpected);

      DMatrixRMaj aV = new DMatrixRMaj(6, 6);
      DMatrixRMaj bV = new DMatrixRMaj(3, 6);
      CommonOps_DDRM.multTransA(f_X, V_XX, aV);
      CommonOps_DDRM.multTransA(f_U, V_XX, bV);

      QxxExpected.set(L_XX);
      CommonOps_DDRM.multAdd(aV, f_X, QxxExpected);

      QuuExpected.set(L_UU);
      CommonOps_DDRM.multAdd(bV, f_U, QuuExpected);

      QuxExpected.set(L_UX);
      CommonOps_DDRM.multAdd(bV, f_X, QuxExpected);

      QxuExpected.set(L_XU);
      CommonOps_DDRM.multAdd(aV, f_U, QxuExpected);

      MatrixTestTools.assertMatrixEquals(QxExpected, Qx, 1e-12);
      MatrixTestTools.assertMatrixEquals(QuExpected, Qu, 1e-12);
      MatrixTestTools.assertMatrixEquals(QxxExpected, Qxx, 1e-12);
      MatrixTestTools.assertMatrixEquals(QuuExpected, Quu, 1e-12);
      MatrixTestTools.assertMatrixEquals(QxuExpected, Qxu, 1e-12);
      MatrixTestTools.assertMatrixEquals(QuxExpected, Qux, 1e-12);
   }



   @Test
   public void testComputePreviousValueApproximation()
   {
      TestDynamics dynamics = new TestDynamics();
      DDPSolver<DefaultDiscreteState> calculator = new DDPSolver<>(dynamics);

      DMatrixRMaj Q_UU = new DMatrixRMaj(3, 3);
      DMatrixRMaj Q_UU_inv = new DMatrixRMaj(3, 3);
      DMatrixRMaj Q_XU = new DMatrixRMaj(6, 3);
      DMatrixRMaj Q_XX = new DMatrixRMaj(6, 6);

      DMatrixRMaj Q_U = new DMatrixRMaj(3, 1);
      DMatrixRMaj Q_X = new DMatrixRMaj(6, 1);

      Q_U.set(0, 0, 8.5);
      Q_U.set(1, 0, 9.5);
      Q_U.set(2, 0, 10.5);

      Q_X.set(0, 0, 8.5);
      Q_X.set(1, 0, 9.5);
      Q_X.set(2, 0, 10.5);
      Q_X.set(3, 0, 8.5);
      Q_X.set(4, 0, 9.5);
      Q_X.set(5, 0, 10.5);

      Q_UU.set(0, 0, 1.0);
      Q_UU.set(0, 1, 2.0);
      Q_UU.set(0, 2, 3.0);

      Q_UU.set(1, 0, 2.0);
      Q_UU.set(1, 1, 7.0);
      Q_UU.set(1, 2, 8.0);

      Q_UU.set(2, 0, 3.0);
      Q_UU.set(2, 1, 8.0);
      Q_UU.set(2, 2, 12.0);

      Q_XU.set(0, 0, 1.0);
      Q_XU.set(0, 1, 2.0);
      Q_XU.set(0, 2, 3.0);

      Q_XU.set(1, 0, 4.0);
      Q_XU.set(1, 1, 5.0);
      Q_XU.set(1, 2, 6.0);

      Q_XU.set(2, 0, 2.0);
      Q_XU.set(2, 1, 7.0);
      Q_XU.set(2, 2, 8.0);

      Q_XU.set(3, 0, 9.0);
      Q_XU.set(3, 1, 10.0);
      Q_XU.set(3, 2, 11.0);

      Q_XU.set(4, 0, 3.0);
      Q_XU.set(4, 1, 8.0);
      Q_XU.set(4, 2, 12.0);

      Q_XU.set(5, 0, 13.0);
      Q_XU.set(5, 1, 14.0);
      Q_XU.set(5, 2, 15.0);

      Q_XX.set(0, 0, 1.0);
      Q_XX.set(0, 1, 2.0);
      Q_XX.set(0, 2, 3.0);
      Q_XX.set(0, 3, 4.0);
      Q_XX.set(0, 4, 5.0);
      Q_XX.set(0, 5, 6.0);

      Q_XX.set(1, 0, 2.0);
      Q_XX.set(1, 1, 7.0);
      Q_XX.set(1, 2, 8.0);
      Q_XX.set(1, 3, 9.0);
      Q_XX.set(1, 4, 10.0);
      Q_XX.set(1, 5, 11.0);

      Q_XX.set(2, 0, 3.0);
      Q_XX.set(2, 1, 8.0);
      Q_XX.set(2, 2, 12.0);
      Q_XX.set(2, 3, 13.0);
      Q_XX.set(2, 4, 14.0);
      Q_XX.set(2, 5, 15.0);

      Q_XX.set(3, 0, 4.0);
      Q_XX.set(3, 1, 9.0);
      Q_XX.set(3, 2, 13.0);
      Q_XX.set(3, 3, 16.0);
      Q_XX.set(3, 4, 17.0);
      Q_XX.set(3, 5, 18.0);

      Q_XX.set(4, 0, 5.0);
      Q_XX.set(4, 1, 10.0);
      Q_XX.set(4, 2, 14.0);
      Q_XX.set(4, 3, 17.0);
      Q_XX.set(4, 4, 19.0);
      Q_XX.set(4, 5, 20.0);

      Q_XX.set(5, 0, 6.0);
      Q_XX.set(5, 1, 11.0);
      Q_XX.set(5, 2, 15.0);
      Q_XX.set(5, 3, 18.0);
      Q_XX.set(5, 4, 21.0);
      Q_XX.set(5, 5, 21.0);

      LinearSolverDense<DMatrixRMaj> linearSolver = LinearSolverFactory_DDRM.linear(0);
      linearSolver.setA(Q_UU);
      linearSolver.invert(Q_UU_inv);

      DMatrixRMaj gainMatrix = new DMatrixRMaj(3, 6);

      CommonOps_DDRM.multTransB(Q_UU_inv, Q_XU, gainMatrix);

      DMatrixRMaj V_X = new DMatrixRMaj(6, 1);
      DMatrixRMaj V_XX = new DMatrixRMaj(6, 6);

      calculator.computePreviousValueApproximation(Q_X, Q_U, Q_XX, Q_XU, gainMatrix, V_X, V_XX);

      DMatrixRMaj V_X_expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj V_XX_expected = new DMatrixRMaj(6, 6);

      V_X_expected.set(Q_X);
      V_XX_expected.set(Q_XX);

      CommonOps_DDRM.multAddTransA(gainMatrix, Q_U, V_X_expected);
      CommonOps_DDRM.multAdd(Q_XU, gainMatrix, V_XX_expected);

      MatrixTestTools.assertMatrixEquals(V_X_expected, V_X, 1e-12);
      MatrixTestTools.assertMatrixEquals(V_XX_expected, V_XX, 1e-12);
   }

   @Test
   public void testComputeFeedbackGainAndFeedForwardTerm()
   {
      TestDynamics dynamics = new TestDynamics();
      DDPSolver<DefaultDiscreteState> calculator = new DDPSolver<>(dynamics);

      DMatrixRMaj Q_UU = new DMatrixRMaj(3, 3);
      DMatrixRMaj Q_UU_inv = new DMatrixRMaj(3, 3);
      DMatrixRMaj Q_UX = new DMatrixRMaj(3, 6);
      DMatrixRMaj Q_XX = new DMatrixRMaj(6, 6);

      DMatrixRMaj Q_U = new DMatrixRMaj(3, 1);
      DMatrixRMaj Q_X = new DMatrixRMaj(6, 1);

      Q_U.set(0, 0, 8.5);
      Q_U.set(1, 0, 9.5);
      Q_U.set(2, 0, 10.5);

      Q_X.set(0, 0, 8.5);
      Q_X.set(1, 0, 9.5);
      Q_X.set(2, 0, 10.5);
      Q_X.set(3, 0, 8.5);
      Q_X.set(4, 0, 9.5);
      Q_X.set(5, 0, 10.5);

      Q_UU.set(0, 0, 1.0);
      Q_UU.set(0, 1, 2.0);
      Q_UU.set(0, 2, 3.0);

      Q_UU.set(1, 0, 2.0);
      Q_UU.set(1, 1, 7.0);
      Q_UU.set(1, 2, 8.0);

      Q_UU.set(2, 0, 3.0);
      Q_UU.set(2, 1, 8.0);
      Q_UU.set(2, 2, 12.0);

      Q_UX.set(0, 0, 1.0);
      Q_UX.set(0, 1, 2.0);
      Q_UX.set(0, 2, 3.0);
      Q_UX.set(0, 3, 4.0);
      Q_UX.set(0, 4, 5.0);
      Q_UX.set(0, 5, 6.0);

      Q_UX.set(1, 0, 2.0);
      Q_UX.set(1, 1, 7.0);
      Q_UX.set(1, 2, 8.0);
      Q_UX.set(1, 3, 9.0);
      Q_UX.set(1, 4, 10.0);
      Q_UX.set(1, 5, 11.0);

      Q_UX.set(2, 0, 3.0);
      Q_UX.set(2, 1, 8.0);
      Q_UX.set(2, 2, 12.0);
      Q_UX.set(2, 3, 13.0);
      Q_UX.set(2, 4, 14.0);
      Q_UX.set(2, 5, 15.0);

      Q_XX.set(0, 0, 1.0);
      Q_XX.set(0, 1, 2.0);
      Q_XX.set(0, 2, 3.0);
      Q_XX.set(0, 3, 4.0);
      Q_XX.set(0, 4, 5.0);
      Q_XX.set(0, 5, 6.0);

      Q_XX.set(1, 0, 2.0);
      Q_XX.set(1, 1, 7.0);
      Q_XX.set(1, 2, 8.0);
      Q_XX.set(1, 3, 9.0);
      Q_XX.set(1, 4, 10.0);
      Q_XX.set(1, 5, 11.0);

      Q_XX.set(2, 0, 3.0);
      Q_XX.set(2, 1, 8.0);
      Q_XX.set(2, 2, 12.0);
      Q_XX.set(2, 3, 13.0);
      Q_XX.set(2, 4, 14.0);
      Q_XX.set(2, 5, 15.0);

      Q_XX.set(3, 0, 4.0);
      Q_XX.set(3, 1, 9.0);
      Q_XX.set(3, 2, 13.0);
      Q_XX.set(3, 3, 16.0);
      Q_XX.set(3, 4, 17.0);
      Q_XX.set(3, 5, 18.0);

      Q_XX.set(4, 0, 5.0);
      Q_XX.set(4, 1, 10.0);
      Q_XX.set(4, 2, 14.0);
      Q_XX.set(4, 3, 17.0);
      Q_XX.set(4, 4, 19.0);
      Q_XX.set(4, 5, 20.0);

      Q_XX.set(5, 0, 6.0);
      Q_XX.set(5, 1, 11.0);
      Q_XX.set(5, 2, 15.0);
      Q_XX.set(5, 3, 18.0);
      Q_XX.set(5, 4, 21.0);
      Q_XX.set(5, 5, 21.0);

      LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.linear(0);
      solver.setA(Q_UU);
      solver.invert(Q_UU_inv);

      DMatrixRMaj gainMatrix = new DMatrixRMaj(3, 6);
      DMatrixRMaj feedforwardMatrix = new DMatrixRMaj(3, 1);

      calculator.computeFeedbackGainAndFeedForwardTerms(Q_U, Q_UU, Q_UX, gainMatrix, feedforwardMatrix);

      DMatrixRMaj gainExpected = new DMatrixRMaj(3, 6);
      DMatrixRMaj feedforwardExpected = new DMatrixRMaj(3, 1);

      CommonOps_DDRM.mult(-1.0, Q_UU_inv, Q_UX, gainExpected);
      CommonOps_DDRM.mult(-1.0, Q_UU_inv, Q_U, feedforwardExpected);

      MatrixTestTools.assertMatrixEquals(gainExpected, gainMatrix, 1e-6);
      MatrixTestTools.assertMatrixEquals(feedforwardExpected, feedforwardMatrix, 1e-6);
   }

   @Test
   public void testAddMultQuad()
   {
      Random random = new Random(1738);
      DMatrixRMaj d = RandomMatrices_DDRM.rectangle(6, 3, random);
      DMatrixRMaj a = RandomMatrices_DDRM.rectangle(4, 6, random);
      DMatrixRMaj b = RandomMatrices_DDRM.rectangle(4, 4, random);
      DMatrixRMaj c = RandomMatrices_DDRM.rectangle(4, 3, random);

      DMatrixRMaj a_expected = new DMatrixRMaj(a);
      DMatrixRMaj b_expected = new DMatrixRMaj(b);
      DMatrixRMaj c_expected = new DMatrixRMaj(c);
      DMatrixRMaj d_original = new DMatrixRMaj(d);

      TestDynamics dynamics = new TestDynamics();
      DDPSolver<DefaultDiscreteState> calculator = new DDPSolver<>(dynamics);

      calculator.addMultQuad(a, b, c, d);

      DMatrixRMaj d_expected = new DMatrixRMaj(6, 3);
      DMatrixRMaj abc = new DMatrixRMaj(6, 3);

      DMatrixRMaj aTran = new DMatrixRMaj(6, 4);
      CommonOps_DDRM.transpose(a, aTran);

      DMatrixRMaj aTranB = new DMatrixRMaj(6, 4);
      CommonOps_DDRM.mult(aTran, b, aTranB);

      CommonOps_DDRM.mult(aTranB, c, abc);

      CommonOps_DDRM.add(d_original, abc, d_expected);

      MatrixTestTools.assertMatrixEquals(a_expected, a, 1e-12);
      MatrixTestTools.assertMatrixEquals(b_expected, b, 1e-12);
      MatrixTestTools.assertMatrixEquals(c_expected, c, 1e-12);
      MatrixTestTools.assertMatrixEquals(d_expected, d, 1e-12);

      double alpha = RandomNumbers.nextDouble(random, 1000);

      d.set(d_original);
      CommonOps_DDRM.add(d_original, alpha, abc, d_expected);
      calculator.addMultQuad(alpha, a, b, c, d);

      MatrixTestTools.assertMatrixEquals(a_expected, a, 1e-12);
      MatrixTestTools.assertMatrixEquals(b_expected, b, 1e-12);
      MatrixTestTools.assertMatrixEquals(c_expected, c, 1e-12);
      MatrixTestTools.assertMatrixEquals(d_expected, d, 1e-12);
   }

   private class TestDynamics implements DiscreteHybridDynamics<DefaultDiscreteState>
   {

      @Override
      public void setTimeStepSize(double deltaT)
      {
      }

      @Override
      public int getStateVectorSize()
      {
         return 0;
      }

      @Override
      public int getControlVectorSize()
      {
         return 0;
      }

      @Override
      public int getConstantVectorSize()
      {
         return 0;
      }

      @Override
      public void getNextState(DefaultDiscreteState hybridState, DMatrixRMaj currentState, DMatrixRMaj currentControl, DMatrixRMaj constants,
                               DMatrixRMaj matrixToPack)
      {

      }

      @Override
      public void getDynamicsStateGradient(DefaultDiscreteState hybridState, DMatrixRMaj currentState, DMatrixRMaj currentControl,
                                           DMatrixRMaj constants, DMatrixRMaj matrixToPack)
      {

      }

      @Override
      public void getDynamicsControlGradient(DefaultDiscreteState hybridState, DMatrixRMaj currentState, DMatrixRMaj currentControl,
                                             DMatrixRMaj constants, DMatrixRMaj matrixToPack)
      {

      }

      @Override
      public void getDynamicsStateHessian(DefaultDiscreteState hybridState, int stateVariable, DMatrixRMaj currentState, DMatrixRMaj currentControl,
                                          DMatrixRMaj constants, DMatrixRMaj matrixToPack)
      {

      }

      @Override
      public void getDynamicsControlHessian(DefaultDiscreteState hybridState, int controlVariable, DMatrixRMaj currentState, DMatrixRMaj currentControl,
                                            DMatrixRMaj constants, DMatrixRMaj matrixToPack)
      {

      }

      @Override
      public void getDynamicsStateGradientOfControlGradient(DefaultDiscreteState hybridState, int stateVariable, DMatrixRMaj currentState,
                                                            DMatrixRMaj currentControl, DMatrixRMaj constants, DMatrixRMaj matrixToPack)
      {

      }

      @Override
      public void getDynamicsControlGradientOfStateGradient(DefaultDiscreteState hybridState, int controlVariable, DMatrixRMaj currentState,
                                                            DMatrixRMaj currentControl, DMatrixRMaj constants, DMatrixRMaj matrixToPack)
      {

      }

      @Override
      public void getContinuousAMatrix(DMatrixRMaj A)
      {

      }

      @Override
      public void getContinuousBMatrix(DMatrixRMaj A)
      {

      }
   }

   private class BasicLQCostFunction implements LQTrackingCostFunction<DefaultDiscreteState>
   {
      @Override
      public double getCost(DefaultDiscreteState state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj desiredControlVector,
                            DMatrixRMaj desiredStateVector, DMatrixRMaj constants)
      {
         return 0;
      }

      @Override
      public void getCostStateGradient(DefaultDiscreteState state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj desiredControlVector,
                                       DMatrixRMaj desiredStateVector, DMatrixRMaj constants, DMatrixRMaj matrixToPack)
      {
      }

      @Override
      public void getCostControlGradient(DefaultDiscreteState state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj desiredControlVector,
                                         DMatrixRMaj desiredStateVector, DMatrixRMaj constants, DMatrixRMaj matrixToPack)
      {
      }

      @Override
      public void getCostStateHessian(DefaultDiscreteState state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj constants,
                                      DMatrixRMaj matrixToPack)
      {
      }

      @Override
      public void getCostControlHessian(DefaultDiscreteState state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj constants,
                                        DMatrixRMaj matrixToPack)
      {
      }

      @Override
      public void getCostStateGradientOfControlGradient(DefaultDiscreteState state, DMatrixRMaj controlVector, DMatrixRMaj stateVector,
                                                        DMatrixRMaj constants, DMatrixRMaj matrixToPack)
      {
      }

      @Override
      public void getCostControlGradientOfStateGradient(DefaultDiscreteState state, DMatrixRMaj controlVector, DMatrixRMaj stateVector,
                                                        DMatrixRMaj constants, DMatrixRMaj matrixToPack)
      {
      }
   }
}
