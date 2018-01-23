package us.ihmc.commonWalkingControlModules.dynamicPlanning.lipm;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.TrackingCostFunctionTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.testing.JUnitTools;
import us.ihmc.trajectoryOptimization.DefaultDiscreteState;
import us.ihmc.trajectoryOptimization.LQTrackingCostFunction;

import static org.junit.Assert.assertEquals;

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
   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testCost()
   {
      LIPMSimpleCostFunction costFunction = new LIPMSimpleCostFunction();

      DenseMatrix64F currentState = new DenseMatrix64F(6, 1);
      currentState.set(0, 1.0);
      currentState.set(1, 0.5);
      currentState.set(2, 1.2);
      currentState.set(3, 2.0);
      currentState.set(4, 3.0);
      currentState.set(5, 0.5);

      DenseMatrix64F desiredState = new DenseMatrix64F(6, 1);
      desiredState.set(0, 0.5);
      desiredState.set(1, 0.25);
      desiredState.set(2, 1.0);
      desiredState.set(3, 1.0);
      desiredState.set(4, 2.0);
      desiredState.set(5, 0.0);

      DenseMatrix64F currentControl = new DenseMatrix64F(3, 1);
      currentControl.set(0, 1.5);
      currentControl.set(1, 0.5);
      currentControl.set(2, 150);

      DenseMatrix64F desiredControl = new DenseMatrix64F(3, 1);
      desiredControl.set(0, 1.2);
      desiredControl.set(1, 0.75);
      desiredControl.set(2, 100);

      DenseMatrix64F constants = new DenseMatrix64F(0, 1);

      double stateCost = LIPMSimpleCostFunction.qX * 0.5 * 0.5 + LIPMSimpleCostFunction.qY * 0.25 * 0.25 + LIPMSimpleCostFunction.qZ * 0.2 * 0.2 +
            LIPMSimpleCostFunction.qXDot + LIPMSimpleCostFunction.qYDot + LIPMSimpleCostFunction.qZDot * 0.5 * 0.5;
      double controlCost = LIPMSimpleCostFunction.rXf * 0.3 * 0.3 + LIPMSimpleCostFunction.rYf * 0.25 * 0.25 + LIPMSimpleCostFunction.rFz * 50 * 50;

      double cost = costFunction.getCost(DefaultDiscreteState.DEFAULT, currentControl, currentState, desiredControl, desiredState, constants);

      assertEquals(stateCost + controlCost, cost, 1e-5);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testComputeCostStateGradient()
   {
      LIPMSimpleCostFunction costFunction = new LIPMSimpleCostFunction();

      DenseMatrix64F gradient = new DenseMatrix64F(6, 1);
      DenseMatrix64F gradientExpected = new DenseMatrix64F(6, 1);

      DenseMatrix64F currentState = new DenseMatrix64F(6, 1);
      currentState.set(0, 1.0);
      currentState.set(1, 0.5);
      currentState.set(2, 1.2);
      currentState.set(3, 2.0);
      currentState.set(4, 3.0);
      currentState.set(5, 0.5);

      DenseMatrix64F desiredState = new DenseMatrix64F(6, 1);
      desiredState.set(0, 0.5);
      desiredState.set(1, 0.25);
      desiredState.set(2, 1.0);
      desiredState.set(3, 1.0);
      desiredState.set(4, 2.0);
      desiredState.set(5, 0.0);

      DenseMatrix64F currentControl = new DenseMatrix64F(3, 1);
      currentControl.set(0, 1.5);
      currentControl.set(1, 0.5);
      currentControl.set(2, 150);

      DenseMatrix64F desiredControl = new DenseMatrix64F(3, 1);
      desiredControl.set(0, 1.2);
      desiredControl.set(1, 0.75);
      desiredControl.set(2, 100);

      DenseMatrix64F constants = new DenseMatrix64F(0, 1);

      gradientExpected.set(0, 0, 2.0 * LIPMSimpleCostFunction.qX * 0.5);
      gradientExpected.set(1, 0, 2.0 * LIPMSimpleCostFunction.qY * 0.25);
      gradientExpected.set(2, 0, 2.0 * LIPMSimpleCostFunction.qZ * 0.2);
      gradientExpected.set(3, 0, 2.0 * LIPMSimpleCostFunction.qXDot * 1.0);
      gradientExpected.set(4, 0, 2.0 * LIPMSimpleCostFunction.qYDot * 1.0);
      gradientExpected.set(5, 0, 2.0 * LIPMSimpleCostFunction.qZDot * 0.5);

      costFunction.getCostStateGradient(DefaultDiscreteState.DEFAULT, currentControl, currentState, desiredControl, desiredState, constants, gradient);

      JUnitTools.assertMatrixEquals(gradientExpected, gradient, 1e-10);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testComputeCostControlGradient()
   {
      LIPMSimpleCostFunction costFunction = new LIPMSimpleCostFunction();

      DenseMatrix64F gradient = new DenseMatrix64F(3, 1);
      DenseMatrix64F gradientExpected = new DenseMatrix64F(3, 1);

      DenseMatrix64F currentState = new DenseMatrix64F(6, 1);
      currentState.set(0, 1.0);
      currentState.set(1, 0.5);
      currentState.set(2, 1.2);
      currentState.set(3, 2.0);
      currentState.set(4, 3.0);
      currentState.set(5, 0.5);

      DenseMatrix64F desiredState = new DenseMatrix64F(6, 1);
      desiredState.set(0, 0.5);
      desiredState.set(1, 0.25);
      desiredState.set(2, 1.0);
      desiredState.set(3, 1.0);
      desiredState.set(4, 2.0);
      desiredState.set(5, 0.0);

      DenseMatrix64F currentControl = new DenseMatrix64F(3, 1);
      currentControl.set(0, 1.5);
      currentControl.set(1, 0.5);
      currentControl.set(2, 150);

      DenseMatrix64F desiredControl = new DenseMatrix64F(3, 1);
      desiredControl.set(0, 1.2);
      desiredControl.set(1, 0.75);
      desiredControl.set(2, 100);

      DenseMatrix64F constants = new DenseMatrix64F(0, 1);

      gradientExpected.set(0, 0, 2.0 * LIPMSimpleCostFunction.rXf * (1.5 - 1.2));
      gradientExpected.set(1, 0, 2.0 * LIPMSimpleCostFunction.rYf * (0.5 - 0.75));
      gradientExpected.set(2, 0, 2.0 * LIPMSimpleCostFunction.rFz * (150 - 100));

      costFunction.getCostControlGradient(DefaultDiscreteState.DEFAULT, currentControl, currentState, desiredControl, desiredState, constants, gradient);

      JUnitTools.assertMatrixEquals(gradientExpected, gradient, 1e-10);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testComputeCostControlHessian()
   {
      DenseMatrix64F R = new DenseMatrix64F(3, 3);
      R.set(0, 0, LIPMSimpleCostFunction.rXf);
      R.set(1, 1, LIPMSimpleCostFunction.rYf);
      R.set(2, 2, LIPMSimpleCostFunction.rFz);

      LIPMSimpleCostFunction costFunction = new LIPMSimpleCostFunction();

      DenseMatrix64F hessian = new DenseMatrix64F(3, 3);

      DenseMatrix64F currentState = new DenseMatrix64F(6, 1);
      currentState.set(0, 1.0);
      currentState.set(1, 0.5);
      currentState.set(2, 1.2);
      currentState.set(3, 2.0);
      currentState.set(4, 3.0);
      currentState.set(5, 0.5);

      DenseMatrix64F currentControl = new DenseMatrix64F(3, 1);
      currentControl.set(0, 1.5);
      currentControl.set(1, 0.5);
      currentControl.set(2, 150);

      DenseMatrix64F constants = new DenseMatrix64F(0, 1);

      costFunction.getCostControlHessian(DefaultDiscreteState.DEFAULT, currentControl, currentState, constants, hessian);

      CommonOps.scale(2.0, R);
      JUnitTools.assertMatrixEquals(R, hessian, 1e-10);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testComputeCostStateHessian()
   {
      DenseMatrix64F Q = new DenseMatrix64F(6, 6);
      Q.set(0, 0, LIPMSimpleCostFunction.qX);
      Q.set(1, 1, LIPMSimpleCostFunction.qY);
      Q.set(2, 2, LIPMSimpleCostFunction.qZ);
      Q.set(3, 3, LIPMSimpleCostFunction.qXDot);
      Q.set(4, 4, LIPMSimpleCostFunction.qYDot);
      Q.set(5, 5, LIPMSimpleCostFunction.qZDot);

      LIPMSimpleCostFunction costFunction = new LIPMSimpleCostFunction();

      DenseMatrix64F hessian = new DenseMatrix64F(6, 6);

      DenseMatrix64F currentState = new DenseMatrix64F(6, 1);
      currentState.set(0, 1.0);
      currentState.set(1, 0.5);
      currentState.set(2, 1.2);
      currentState.set(3, 2.0);
      currentState.set(4, 3.0);
      currentState.set(5, 0.5);

      DenseMatrix64F currentControl = new DenseMatrix64F(3, 1);
      currentControl.set(0, 1.5);
      currentControl.set(1, 0.5);
      currentControl.set(2, 150);

      DenseMatrix64F constants = new DenseMatrix64F(0, 1);

      costFunction.getCostStateHessian(DefaultDiscreteState.DEFAULT, currentControl, currentState, constants, hessian);

      CommonOps.scale(2.0, Q);
      JUnitTools.assertMatrixEquals(Q, hessian, 1e-10);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testComputeCostStateGradientOfControlGradient()
   {
      LIPMSimpleCostFunction costFunction = new LIPMSimpleCostFunction();

      DenseMatrix64F hessian = new DenseMatrix64F(3, 6);
      DenseMatrix64F hessianExpected = new DenseMatrix64F(3, 6);

      DenseMatrix64F currentState = new DenseMatrix64F(6, 1);
      currentState.set(0, 1.0);
      currentState.set(1, 0.5);
      currentState.set(2, 1.2);
      currentState.set(3, 2.0);
      currentState.set(4, 3.0);
      currentState.set(5, 0.5);

      DenseMatrix64F currentControl = new DenseMatrix64F(3, 1);
      currentControl.set(0, 1.5);
      currentControl.set(1, 0.5);
      currentControl.set(2, 150);

      DenseMatrix64F constants = new DenseMatrix64F(0, 1);

      costFunction.getCostStateGradientOfControlGradient(DefaultDiscreteState.DEFAULT, currentControl, currentState, constants, hessian);

      JUnitTools.assertMatrixEquals(hessianExpected, hessian, 1e-10);
   }




   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testCostStateGradientNumerically()
   {
      super.testCostStateGradientNumerically();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testCostControlGradientNumerically()
   {
      super.testCostControlGradientNumerically();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testCostStateHessianNumerically()
   {
      super.testCostStateHessianNumerically();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testCostControlHessianNumerically()
   {
      super.testCostControlHessianNumerically();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testCostStateControlHessianNumerically()
   {
      super.testCostStateControlHessianNumerically();
   }
}
