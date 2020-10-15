package us.ihmc.commonWalkingControlModules.dynamicPlanning.lipm;

import static us.ihmc.robotics.Assert.assertEquals;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.TrackingCostFunctionTest;
import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.trajectoryOptimization.DefaultDiscreteState;
import us.ihmc.trajectoryOptimization.LQTrackingCostFunction;

public class LIPMSimpleCostFunctionTest extends TrackingCostFunctionTest<DefaultDiscreteState>
{
   @Override
   public int getNumberOfStates()
   {
      return 1;
   }

   @Override
   public int getStateVectorSize()
   {
      return LIPMDynamics.stateVectorSize;
   }

   @Override
   public int getControlVectorSize()
   {
      return LIPMDynamics.controlVectorSize;
   }

   @Override
   public int getConstantVectorSize()
   {
      return LIPMDynamics.constantVectorSize;
   }

   public DefaultDiscreteState getHybridState(int hybridStateIndex)
   {
      return DefaultDiscreteState.DEFAULT;
   }

   public LQTrackingCostFunction<DefaultDiscreteState> getCostFunction()
   {
      return new LIPMSimpleCostFunction();
   }

   @Override
   @Test
   public void testCost()
   {
      LIPMSimpleCostFunction costFunction = new LIPMSimpleCostFunction();

      DMatrixRMaj currentState = new DMatrixRMaj(6, 1);
      currentState.set(0, 1.0);
      currentState.set(1, 0.5);
      currentState.set(2, 1.2);
      currentState.set(3, 2.0);
      currentState.set(4, 3.0);
      currentState.set(5, 0.5);

      DMatrixRMaj desiredState = new DMatrixRMaj(6, 1);
      desiredState.set(0, 0.5);
      desiredState.set(1, 0.25);
      desiredState.set(2, 1.0);
      desiredState.set(3, 1.0);
      desiredState.set(4, 2.0);
      desiredState.set(5, 0.0);

      DMatrixRMaj currentControl = new DMatrixRMaj(3, 1);
      currentControl.set(0, 1.5);
      currentControl.set(1, 0.5);
      currentControl.set(2, 150);

      DMatrixRMaj desiredControl = new DMatrixRMaj(3, 1);
      desiredControl.set(0, 1.2);
      desiredControl.set(1, 0.75);
      desiredControl.set(2, 100);

      DMatrixRMaj constants = new DMatrixRMaj(0, 1);

      double stateCost = LIPMSimpleCostFunction.qX * 0.5 * 0.5 + LIPMSimpleCostFunction.qY * 0.25 * 0.25 + LIPMSimpleCostFunction.qZ * 0.2 * 0.2 +
            LIPMSimpleCostFunction.qXDot + LIPMSimpleCostFunction.qYDot + LIPMSimpleCostFunction.qZDot * 0.5 * 0.5;
      double controlCost = LIPMSimpleCostFunction.rXf * 0.3 * 0.3 + LIPMSimpleCostFunction.rYf * 0.25 * 0.25 + LIPMSimpleCostFunction.rFz * 50 * 50;

      double cost = costFunction.getCost(DefaultDiscreteState.DEFAULT, currentControl, currentState, desiredControl, desiredState, constants);

      assertEquals(stateCost + controlCost, cost, 1e-5);
   }

   @Test
   public void testComputeCostStateGradient()
   {
      LIPMSimpleCostFunction costFunction = new LIPMSimpleCostFunction();

      DMatrixRMaj gradient = new DMatrixRMaj(6, 1);
      DMatrixRMaj gradientExpected = new DMatrixRMaj(6, 1);

      DMatrixRMaj currentState = new DMatrixRMaj(6, 1);
      currentState.set(0, 1.0);
      currentState.set(1, 0.5);
      currentState.set(2, 1.2);
      currentState.set(3, 2.0);
      currentState.set(4, 3.0);
      currentState.set(5, 0.5);

      DMatrixRMaj desiredState = new DMatrixRMaj(6, 1);
      desiredState.set(0, 0.5);
      desiredState.set(1, 0.25);
      desiredState.set(2, 1.0);
      desiredState.set(3, 1.0);
      desiredState.set(4, 2.0);
      desiredState.set(5, 0.0);

      DMatrixRMaj currentControl = new DMatrixRMaj(3, 1);
      currentControl.set(0, 1.5);
      currentControl.set(1, 0.5);
      currentControl.set(2, 150);

      DMatrixRMaj desiredControl = new DMatrixRMaj(3, 1);
      desiredControl.set(0, 1.2);
      desiredControl.set(1, 0.75);
      desiredControl.set(2, 100);

      DMatrixRMaj constants = new DMatrixRMaj(0, 1);

      gradientExpected.set(0, 0, 2.0 * LIPMSimpleCostFunction.qX * 0.5);
      gradientExpected.set(1, 0, 2.0 * LIPMSimpleCostFunction.qY * 0.25);
      gradientExpected.set(2, 0, 2.0 * LIPMSimpleCostFunction.qZ * 0.2);
      gradientExpected.set(3, 0, 2.0 * LIPMSimpleCostFunction.qXDot * 1.0);
      gradientExpected.set(4, 0, 2.0 * LIPMSimpleCostFunction.qYDot * 1.0);
      gradientExpected.set(5, 0, 2.0 * LIPMSimpleCostFunction.qZDot * 0.5);

      costFunction.getCostStateGradient(DefaultDiscreteState.DEFAULT, currentControl, currentState, desiredControl, desiredState, constants, gradient);

      MatrixTestTools.assertMatrixEquals(gradientExpected, gradient, 1e-10);
   }

   @Test
   public void testComputeCostControlGradient()
   {
      LIPMSimpleCostFunction costFunction = new LIPMSimpleCostFunction();

      DMatrixRMaj gradient = new DMatrixRMaj(3, 1);
      DMatrixRMaj gradientExpected = new DMatrixRMaj(3, 1);

      DMatrixRMaj currentState = new DMatrixRMaj(6, 1);
      currentState.set(0, 1.0);
      currentState.set(1, 0.5);
      currentState.set(2, 1.2);
      currentState.set(3, 2.0);
      currentState.set(4, 3.0);
      currentState.set(5, 0.5);

      DMatrixRMaj desiredState = new DMatrixRMaj(6, 1);
      desiredState.set(0, 0.5);
      desiredState.set(1, 0.25);
      desiredState.set(2, 1.0);
      desiredState.set(3, 1.0);
      desiredState.set(4, 2.0);
      desiredState.set(5, 0.0);

      DMatrixRMaj currentControl = new DMatrixRMaj(3, 1);
      currentControl.set(0, 1.5);
      currentControl.set(1, 0.5);
      currentControl.set(2, 150);

      DMatrixRMaj desiredControl = new DMatrixRMaj(3, 1);
      desiredControl.set(0, 1.2);
      desiredControl.set(1, 0.75);
      desiredControl.set(2, 100);

      DMatrixRMaj constants = new DMatrixRMaj(0, 1);

      gradientExpected.set(0, 0, 2.0 * LIPMSimpleCostFunction.rXf * (1.5 - 1.2));
      gradientExpected.set(1, 0, 2.0 * LIPMSimpleCostFunction.rYf * (0.5 - 0.75));
      gradientExpected.set(2, 0, 2.0 * LIPMSimpleCostFunction.rFz * (150 - 100));

      costFunction.getCostControlGradient(DefaultDiscreteState.DEFAULT, currentControl, currentState, desiredControl, desiredState, constants, gradient);

      MatrixTestTools.assertMatrixEquals(gradientExpected, gradient, 1e-10);
   }

   @Test
   public void testComputeCostControlHessian()
   {
      DMatrixRMaj R = new DMatrixRMaj(3, 3);
      R.set(0, 0, LIPMSimpleCostFunction.rXf);
      R.set(1, 1, LIPMSimpleCostFunction.rYf);
      R.set(2, 2, LIPMSimpleCostFunction.rFz);

      LIPMSimpleCostFunction costFunction = new LIPMSimpleCostFunction();

      DMatrixRMaj hessian = new DMatrixRMaj(3, 3);

      DMatrixRMaj currentState = new DMatrixRMaj(6, 1);
      currentState.set(0, 1.0);
      currentState.set(1, 0.5);
      currentState.set(2, 1.2);
      currentState.set(3, 2.0);
      currentState.set(4, 3.0);
      currentState.set(5, 0.5);

      DMatrixRMaj currentControl = new DMatrixRMaj(3, 1);
      currentControl.set(0, 1.5);
      currentControl.set(1, 0.5);
      currentControl.set(2, 150);

      DMatrixRMaj constants = new DMatrixRMaj(0, 1);

      costFunction.getCostControlHessian(DefaultDiscreteState.DEFAULT, currentControl, currentState, constants, hessian);

      CommonOps_DDRM.scale(2.0, R);
      MatrixTestTools.assertMatrixEquals(R, hessian, 1e-10);
   }

   @Test
   public void testComputeCostStateHessian()
   {
      DMatrixRMaj Q = new DMatrixRMaj(6, 6);
      Q.set(0, 0, LIPMSimpleCostFunction.qX);
      Q.set(1, 1, LIPMSimpleCostFunction.qY);
      Q.set(2, 2, LIPMSimpleCostFunction.qZ);
      Q.set(3, 3, LIPMSimpleCostFunction.qXDot);
      Q.set(4, 4, LIPMSimpleCostFunction.qYDot);
      Q.set(5, 5, LIPMSimpleCostFunction.qZDot);

      LIPMSimpleCostFunction costFunction = new LIPMSimpleCostFunction();

      DMatrixRMaj hessian = new DMatrixRMaj(6, 6);

      DMatrixRMaj currentState = new DMatrixRMaj(6, 1);
      currentState.set(0, 1.0);
      currentState.set(1, 0.5);
      currentState.set(2, 1.2);
      currentState.set(3, 2.0);
      currentState.set(4, 3.0);
      currentState.set(5, 0.5);

      DMatrixRMaj currentControl = new DMatrixRMaj(3, 1);
      currentControl.set(0, 1.5);
      currentControl.set(1, 0.5);
      currentControl.set(2, 150);

      DMatrixRMaj constants = new DMatrixRMaj(0, 1);

      costFunction.getCostStateHessian(DefaultDiscreteState.DEFAULT, currentControl, currentState, constants, hessian);

      CommonOps_DDRM.scale(2.0, Q);
      MatrixTestTools.assertMatrixEquals(Q, hessian, 1e-10);
   }

   @Test
   public void testComputeCostStateGradientOfControlGradient()
   {
      LIPMSimpleCostFunction costFunction = new LIPMSimpleCostFunction();

      DMatrixRMaj hessian = new DMatrixRMaj(3, 6);
      DMatrixRMaj hessianExpected = new DMatrixRMaj(3, 6);

      DMatrixRMaj currentState = new DMatrixRMaj(6, 1);
      currentState.set(0, 1.0);
      currentState.set(1, 0.5);
      currentState.set(2, 1.2);
      currentState.set(3, 2.0);
      currentState.set(4, 3.0);
      currentState.set(5, 0.5);

      DMatrixRMaj currentControl = new DMatrixRMaj(3, 1);
      currentControl.set(0, 1.5);
      currentControl.set(1, 0.5);
      currentControl.set(2, 150);

      DMatrixRMaj constants = new DMatrixRMaj(0, 1);

      costFunction.getCostStateGradientOfControlGradient(DefaultDiscreteState.DEFAULT, currentControl, currentState, constants, hessian);

      MatrixTestTools.assertMatrixEquals(hessianExpected, hessian, 1e-10);
   }




   @Override
   @Test
   public void testCostStateGradientNumerically()
   {
      super.testCostStateGradientNumerically();
   }

   @Override
   @Test
   public void testCostControlGradientNumerically()
   {
      super.testCostControlGradientNumerically();
   }

   @Override
   @Test
   public void testCostStateHessianNumerically()
   {
      super.testCostStateHessianNumerically();
   }

   @Override
   @Test
   public void testCostControlHessianNumerically()
   {
      super.testCostControlHessianNumerically();
   }

   @Override
   @Test
   public void testCostStateControlHessianNumerically()
   {
      super.testCostStateControlHessianNumerically();
   }
}
