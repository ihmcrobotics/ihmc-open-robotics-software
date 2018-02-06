package us.ihmc.trajectoryOptimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import org.ejml.ops.RandomMatrices;
import org.junit.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.testing.JUnitTools;

import java.util.Random;

public class DDPSolverTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testComputeUpdatedControl()
   {
      TestDynamics dynamics = new TestDynamics();
      DDPSolver<DefaultDiscreteState> calculator = new DDPSolver<>(dynamics);

      DenseMatrix64F feedforwardTerm = new DenseMatrix64F(3, 1);

      DenseMatrix64F currentState = new DenseMatrix64F(6, 1);
      DenseMatrix64F currentControl = new DenseMatrix64F(3, 1);

      DenseMatrix64F updatedState = new DenseMatrix64F(6, 1);
      DenseMatrix64F updatedControl = new DenseMatrix64F(3, 1);

      DenseMatrix64F updatedControlExpected = new DenseMatrix64F(3, 1);

      DenseMatrix64F gainMatrix = new DenseMatrix64F(3, 6);

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

      DenseMatrix64F stateError = new DenseMatrix64F(6, 1);
      DenseMatrix64F updatedControlAlternative = new DenseMatrix64F(3, 1);

      CommonOps.subtract(updatedState, currentState, stateError);
      CommonOps.add(currentControl, feedforwardTerm, updatedControlAlternative);
      CommonOps.multAdd(gainMatrix, stateError, updatedControlAlternative);

      JUnitTools.assertMatrixEquals(updatedControlAlternative, updatedControl, 1e-12);
      JUnitTools.assertMatrixEquals(updatedControlExpected, updatedControl, 1e-12);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testUpdateHamiltonianApproximations()
   {
      Random random = new Random(1738);
      DenseMatrix64F V_X = RandomMatrices.createRandom(6, 1, random);
      DenseMatrix64F V_XX = RandomMatrices.createSymmetric(6, -1000, 1000, random);

      DenseMatrix64F L_X = RandomMatrices.createRandom(6, 1, random);
      DenseMatrix64F L_U = RandomMatrices.createRandom(3, 1, random);
      DenseMatrix64F L_XX = RandomMatrices.createRandom(6, 6, random);
      DenseMatrix64F L_XU = RandomMatrices.createRandom(6, 3, random);
      DenseMatrix64F L_UX = new DenseMatrix64F(3, 6);
      DenseMatrix64F L_UU = RandomMatrices.createRandom(3, 3, random);
      CommonOps.transpose(L_XU, L_UX);

      DenseMatrix64F f_X = RandomMatrices.createRandom(6, 6, random);
      DenseMatrix64F f_U = RandomMatrices.createRandom(6, 3, random);

      DenseMatrix64F Qx = new DenseMatrix64F(6, 1);
      DenseMatrix64F Qu = new DenseMatrix64F(3, 1);
      DenseMatrix64F Qxx = new DenseMatrix64F(6, 6);
      DenseMatrix64F Quu = new DenseMatrix64F(3, 3);
      DenseMatrix64F Qux = new DenseMatrix64F(3, 6);
      DenseMatrix64F Qxu = new DenseMatrix64F(6, 3);
      DenseMatrix64F QuExpected = new DenseMatrix64F(3, 1);
      DenseMatrix64F QxExpected = new DenseMatrix64F(6, 1);
      DenseMatrix64F QxxExpected = new DenseMatrix64F(6, 6);
      DenseMatrix64F QuuExpected = new DenseMatrix64F(3, 3);
      DenseMatrix64F QuxExpected = new DenseMatrix64F(3, 6);
      DenseMatrix64F QxuExpected = new DenseMatrix64F(6, 3);

      TestDynamics dynamics = new TestDynamics();
      DDPSolver<DefaultDiscreteState> calculator = new DDPSolver<>(dynamics);

      calculator.updateHamiltonianApproximations(DefaultDiscreteState.DEFAULT, 0, L_X, L_U, L_XX, L_UU, L_XU, f_X, f_U, V_X, V_XX, Qx, Qu, Qxx, Quu, Qxu, Qux);

      QxExpected.set(L_X);
      CommonOps.multAddTransA(f_X, V_X, QxExpected);

      QuExpected.set(L_U);
      CommonOps.multAddTransA(f_U, V_X, QuExpected);

      DenseMatrix64F aV = new DenseMatrix64F(6, 6);
      DenseMatrix64F bV = new DenseMatrix64F(3, 6);
      CommonOps.multTransA(f_X, V_XX, aV);
      CommonOps.multTransA(f_U, V_XX, bV);

      QxxExpected.set(L_XX);
      CommonOps.multAdd(aV, f_X, QxxExpected);

      QuuExpected.set(L_UU);
      CommonOps.multAdd(bV, f_U, QuuExpected);

      QuxExpected.set(L_UX);
      CommonOps.multAdd(bV, f_X, QuxExpected);

      QxuExpected.set(L_XU);
      CommonOps.multAdd(aV, f_U, QxuExpected);

      JUnitTools.assertMatrixEquals(QxExpected, Qx, 1e-12);
      JUnitTools.assertMatrixEquals(QuExpected, Qu, 1e-12);
      JUnitTools.assertMatrixEquals(QxxExpected, Qxx, 1e-12);
      JUnitTools.assertMatrixEquals(QuuExpected, Quu, 1e-12);
      JUnitTools.assertMatrixEquals(QxuExpected, Qxu, 1e-12);
      JUnitTools.assertMatrixEquals(QuxExpected, Qux, 1e-12);
   }



   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testComputePreviousValueApproximation()
   {
      TestDynamics dynamics = new TestDynamics();
      DDPSolver<DefaultDiscreteState> calculator = new DDPSolver<>(dynamics);

      DenseMatrix64F Q_UU = new DenseMatrix64F(3, 3);
      DenseMatrix64F Q_UU_inv = new DenseMatrix64F(3, 3);
      DenseMatrix64F Q_XU = new DenseMatrix64F(6, 3);
      DenseMatrix64F Q_XX = new DenseMatrix64F(6, 6);

      DenseMatrix64F Q_U = new DenseMatrix64F(3, 1);
      DenseMatrix64F Q_X = new DenseMatrix64F(6, 1);

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

      LinearSolver<DenseMatrix64F> linearSolver = LinearSolverFactory.linear(0);
      linearSolver.setA(Q_UU);
      linearSolver.invert(Q_UU_inv);

      DenseMatrix64F gainMatrix = new DenseMatrix64F(3, 6);

      CommonOps.multTransB(Q_UU_inv, Q_XU, gainMatrix);

      DenseMatrix64F V_X = new DenseMatrix64F(6, 1);
      DenseMatrix64F V_XX = new DenseMatrix64F(6, 6);

      calculator.computePreviousValueApproximation(Q_X, Q_U, Q_XX, Q_XU, gainMatrix, V_X, V_XX);

      DenseMatrix64F V_X_expected = new DenseMatrix64F(6, 1);
      DenseMatrix64F V_XX_expected = new DenseMatrix64F(6, 6);

      V_X_expected.set(Q_X);
      V_XX_expected.set(Q_XX);

      CommonOps.multAddTransA(gainMatrix, Q_U, V_X_expected);
      CommonOps.multAdd(Q_XU, gainMatrix, V_XX_expected);

      JUnitTools.assertMatrixEquals(V_X_expected, V_X, 1e-12);
      JUnitTools.assertMatrixEquals(V_XX_expected, V_XX, 1e-12);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testComputeFeedbackGainAndFeedForwardTerm()
   {
      TestDynamics dynamics = new TestDynamics();
      DDPSolver<DefaultDiscreteState> calculator = new DDPSolver<>(dynamics);

      DenseMatrix64F Q_UU = new DenseMatrix64F(3, 3);
      DenseMatrix64F Q_UU_inv = new DenseMatrix64F(3, 3);
      DenseMatrix64F Q_UX = new DenseMatrix64F(3, 6);
      DenseMatrix64F Q_XX = new DenseMatrix64F(6, 6);

      DenseMatrix64F Q_U = new DenseMatrix64F(3, 1);
      DenseMatrix64F Q_X = new DenseMatrix64F(6, 1);

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

      LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.linear(0);
      solver.setA(Q_UU);
      solver.invert(Q_UU_inv);

      DenseMatrix64F gainMatrix = new DenseMatrix64F(3, 6);
      DenseMatrix64F feedforwardMatrix = new DenseMatrix64F(3, 1);

      calculator.computeFeedbackGainAndFeedForwardTerms(Q_U, Q_UU, Q_UX, gainMatrix, feedforwardMatrix);

      DenseMatrix64F gainExpected = new DenseMatrix64F(3, 6);
      DenseMatrix64F feedforwardExpected = new DenseMatrix64F(3, 1);

      CommonOps.mult(-1.0, Q_UU_inv, Q_UX, gainExpected);
      CommonOps.mult(-1.0, Q_UU_inv, Q_U, feedforwardExpected);

      JUnitTools.assertMatrixEquals(gainExpected, gainMatrix, 1e-6);
      JUnitTools.assertMatrixEquals(feedforwardExpected, feedforwardMatrix, 1e-6);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testAddMultQuad()
   {
      Random random = new Random(1738);
      DenseMatrix64F d = RandomMatrices.createRandom(6, 3, random);
      DenseMatrix64F a = RandomMatrices.createRandom(4, 6, random);
      DenseMatrix64F b = RandomMatrices.createRandom(4, 4, random);
      DenseMatrix64F c = RandomMatrices.createRandom(4, 3, random);

      DenseMatrix64F a_expected = new DenseMatrix64F(a);
      DenseMatrix64F b_expected = new DenseMatrix64F(b);
      DenseMatrix64F c_expected = new DenseMatrix64F(c);
      DenseMatrix64F d_original = new DenseMatrix64F(d);

      TestDynamics dynamics = new TestDynamics();
      DDPSolver<DefaultDiscreteState> calculator = new DDPSolver<>(dynamics);

      calculator.addMultQuad(a, b, c, d);

      DenseMatrix64F d_expected = new DenseMatrix64F(6, 3);
      DenseMatrix64F abc = new DenseMatrix64F(6, 3);

      DenseMatrix64F aTran = new DenseMatrix64F(6, 4);
      CommonOps.transpose(a, aTran);

      DenseMatrix64F aTranB = new DenseMatrix64F(6, 4);
      CommonOps.mult(aTran, b, aTranB);

      CommonOps.mult(aTranB, c, abc);

      CommonOps.add(d_original, abc, d_expected);

      JUnitTools.assertMatrixEquals(a_expected, a, 1e-12);
      JUnitTools.assertMatrixEquals(b_expected, b, 1e-12);
      JUnitTools.assertMatrixEquals(c_expected, c, 1e-12);
      JUnitTools.assertMatrixEquals(d_expected, d, 1e-12);

      double alpha = RandomNumbers.nextDouble(random, 1000);

      d.set(d_original);
      CommonOps.add(d_original, alpha, abc, d_expected);
      calculator.addMultQuad(alpha, a, b, c, d);

      JUnitTools.assertMatrixEquals(a_expected, a, 1e-12);
      JUnitTools.assertMatrixEquals(b_expected, b, 1e-12);
      JUnitTools.assertMatrixEquals(c_expected, c, 1e-12);
      JUnitTools.assertMatrixEquals(d_expected, d, 1e-12);
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
      public void getNextState(DefaultDiscreteState hybridState, DenseMatrix64F currentState, DenseMatrix64F currentControl, DenseMatrix64F constants,
                               DenseMatrix64F matrixToPack)
      {

      }

      @Override
      public void getDynamicsStateGradient(DefaultDiscreteState hybridState, DenseMatrix64F currentState, DenseMatrix64F currentControl,
                                           DenseMatrix64F constants, DenseMatrix64F matrixToPack)
      {

      }

      @Override
      public void getDynamicsControlGradient(DefaultDiscreteState hybridState, DenseMatrix64F currentState, DenseMatrix64F currentControl,
                                             DenseMatrix64F constants, DenseMatrix64F matrixToPack)
      {

      }

      @Override
      public void getDynamicsStateHessian(DefaultDiscreteState hybridState, int stateVariable, DenseMatrix64F currentState, DenseMatrix64F currentControl,
                                          DenseMatrix64F constants, DenseMatrix64F matrixToPack)
      {

      }

      @Override
      public void getDynamicsControlHessian(DefaultDiscreteState hybridState, int controlVariable, DenseMatrix64F currentState, DenseMatrix64F currentControl,
                                            DenseMatrix64F constants, DenseMatrix64F matrixToPack)
      {

      }

      @Override
      public void getDynamicsStateGradientOfControlGradient(DefaultDiscreteState hybridState, int stateVariable, DenseMatrix64F currentState,
                                                            DenseMatrix64F currentControl, DenseMatrix64F constants, DenseMatrix64F matrixToPack)
      {

      }

      @Override
      public void getDynamicsControlGradientOfStateGradient(DefaultDiscreteState hybridState, int controlVariable, DenseMatrix64F currentState,
                                                            DenseMatrix64F currentControl, DenseMatrix64F constants, DenseMatrix64F matrixToPack)
      {

      }

      @Override
      public void getContinuousAMatrix(DenseMatrix64F A)
      {

      }

      @Override
      public void getContinuousBMatrix(DenseMatrix64F A)
      {

      }
   }

   private class BasicLQCostFunction implements LQTrackingCostFunction<DefaultDiscreteState>
   {
      @Override
      public double getCost(DefaultDiscreteState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F desiredControlVector,
                            DenseMatrix64F desiredStateVector, DenseMatrix64F constants)
      {
         return 0;
      }

      @Override
      public void getCostStateGradient(DefaultDiscreteState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F desiredControlVector,
                                       DenseMatrix64F desiredStateVector, DenseMatrix64F constants, DenseMatrix64F matrixToPack)
      {
      }

      @Override
      public void getCostControlGradient(DefaultDiscreteState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F desiredControlVector,
                                         DenseMatrix64F desiredStateVector, DenseMatrix64F constants, DenseMatrix64F matrixToPack)
      {
      }

      @Override
      public void getCostStateHessian(DefaultDiscreteState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F constants,
                                      DenseMatrix64F matrixToPack)
      {
      }

      @Override
      public void getCostControlHessian(DefaultDiscreteState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F constants,
                                        DenseMatrix64F matrixToPack)
      {
      }

      @Override
      public void getCostStateGradientOfControlGradient(DefaultDiscreteState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector,
                                                        DenseMatrix64F constants, DenseMatrix64F matrixToPack)
      {
      }

      @Override
      public void getCostControlGradientOfStateGradient(DefaultDiscreteState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector,
                                                        DenseMatrix64F constants, DenseMatrix64F matrixToPack)
      {
      }
   }
}
