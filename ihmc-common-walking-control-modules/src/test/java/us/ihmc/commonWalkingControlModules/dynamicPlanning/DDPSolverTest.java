package us.ihmc.commonWalkingControlModules.dynamicPlanning;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.testing.JUnitTools;

public class DDPSolverTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testComputeNewControlAndNextState()
   {
      LIPMDynamics dynamics = new LIPMDynamics(0.01, 10, 9.81);
      LQCostFunction costFunction = new LIPMSimpleCostFunction();
      LQCostFunction terminalCostFunction = new LIPMTerminalCostFunction();
      DDPSolver<LIPMState> calculator = new DDPSolver<>(dynamics, costFunction, terminalCostFunction);

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

      double updatedPx = pX - feedforwardX + xyPositionGain * (currentX - updatedX) + xyDotPositionGain * (currentXDot - updatedXDot);
      double updatedPy = pY - feedforwardY + xyPositionGain * (currentY - updatedY) + xyDotPositionGain * (currentYDot - updatedYDot);
      double updatedFz = fZ - feedforwardZ + zPositionGain * (currentZ - updatedZ) + zDotPositionGain * (currentZDot - updatedZDot);

      updatedControlExpected.set(0, updatedPx);
      updatedControlExpected.set(1, updatedPy);
      updatedControlExpected.set(2, updatedFz);

      calculator.computeUpdatedControl(currentState, updatedState, gainMatrix, feedforwardTerm, currentControl, updatedControl);

      DenseMatrix64F stateError = new DenseMatrix64F(6, 1);
      DenseMatrix64F updatedControlAlternative = new DenseMatrix64F(3, 1);

      CommonOps.subtract(currentState, updatedState, stateError);
      CommonOps.subtract(currentControl, feedforwardTerm, updatedControlAlternative);
      CommonOps.multAdd(gainMatrix, stateError, updatedControlAlternative);

      JUnitTools.assertMatrixEquals(updatedControlAlternative, updatedControl, 1e-12);
      JUnitTools.assertMatrixEquals(updatedControlExpected, updatedControl, 1e-12);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testComputeValueApproximationAtNextStep()
   {
      LIPMDynamics dynamics = new LIPMDynamics(0.01, 10, 9.81);
      LQCostFunction costFunction = new LIPMSimpleCostFunction();
      LQCostFunction terminalCostFunction = new LIPMTerminalCostFunction();
      DDPSolver<LIPMState> calculator = new DDPSolver<>(dynamics, costFunction, terminalCostFunction);

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

      LinearSolver<DenseMatrix64F> linearSolver = LinearSolverFactory.linear(0);
      linearSolver.setA(Q_UU);
      linearSolver.invert(Q_UU_inv);

      DenseMatrix64F gainMatrix = new DenseMatrix64F(3, 6);

      CommonOps.mult(Q_UU_inv, Q_UX, gainMatrix);


      DenseMatrix64F V_X = new DenseMatrix64F(6, 1);
      DenseMatrix64F V_XX = new DenseMatrix64F(6, 6);

      calculator.computeValueApproximationForNextStep(V_X, V_XX, gainMatrix, Q_X, Q_U, Q_XX, Q_UX);

      DenseMatrix64F V_X_expected = new DenseMatrix64F(1, 6);
      DenseMatrix64F V_XX_expected = new DenseMatrix64F(6, 6);

      CommonOps.transpose(Q_X, V_X_expected);
      V_XX_expected.set(Q_XX);

      CommonOps.multAddTransA(-1.0, Q_U, gainMatrix, V_X_expected);
      CommonOps.multAddTransA(-1.0, Q_UX, gainMatrix, V_XX_expected);

      CommonOps.transpose(V_X_expected);

      JUnitTools.assertMatrixEquals(V_X_expected, V_X, 1e-12);
      JUnitTools.assertMatrixEquals(V_XX_expected, V_XX, 1e-12);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testComputeFeedbackGainAndFeedForwardTerm()
   {
      LIPMDynamics dynamics = new LIPMDynamics(0.01, 10, 9.81);
      LQCostFunction costFunction = new LIPMSimpleCostFunction();
      LQCostFunction terminalCostFunction = new LIPMTerminalCostFunction();
      DDPSolver<LIPMState> calculator = new DDPSolver<>(dynamics, costFunction, terminalCostFunction);

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

      calculator.computeFeedbackGainAndFeedForwardTerms(gainMatrix, feedforwardMatrix, Q_U, Q_UU, Q_UX);

      DenseMatrix64F gainExpected = new DenseMatrix64F(3, 6);
      DenseMatrix64F feedforwardExpected = new DenseMatrix64F(3, 1);

      CommonOps.mult(Q_UU_inv, Q_UX, gainExpected);
      CommonOps.mult(Q_UU_inv, Q_U, feedforwardExpected);

      JUnitTools.assertMatrixEquals(gainExpected, gainMatrix, 1e-12);
      JUnitTools.assertMatrixEquals(feedforwardExpected, feedforwardMatrix, 1e-12);
   }
}
