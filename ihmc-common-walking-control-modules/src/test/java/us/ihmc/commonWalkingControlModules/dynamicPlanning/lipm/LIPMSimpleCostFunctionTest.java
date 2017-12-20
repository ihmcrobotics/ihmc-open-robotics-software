package us.ihmc.commonWalkingControlModules.dynamicPlanning.lipm;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.lipm.LIPMSimpleCostFunction;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.testing.JUnitTools;

import static org.junit.Assert.assertEquals;

public class LIPMSimpleCostFunctionTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testComputeCost()
   {
      DenseMatrix64F Q = new DenseMatrix64F(6, 6);
      DenseMatrix64F R = new DenseMatrix64F(3, 3);
      Q.set(0, 0, 1e-4);
      Q.set(1, 1, 1e-4);
      Q.set(2, 2, 10);
      Q.set(3, 3, 1e-2);
      Q.set(4, 4, 1e-2);
      Q.set(5, 5, 1e-2);

      R.set(0, 0, 1);
      R.set(1, 1, 1);
      R.set(2, 2, 1e-6);

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

      double stateCost = 0.25e-4 + 0.625e-6 + 0.4 + 1e-2 + 1e-2 + 0.25e-2;
      double controlCost = 0.09 + 0.0625 + 0.0025;

      double cost = costFunction.getCost(currentControl, currentState, desiredControl, desiredState);

      assertEquals(stateCost + controlCost, cost, 1e-5);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testComputeCostStateGradient()
   {
      DenseMatrix64F Q = new DenseMatrix64F(6, 6);
      DenseMatrix64F R = new DenseMatrix64F(3, 3);
      Q.set(0, 0, 1e-6);
      Q.set(1, 1, 1e-6);
      Q.set(2, 2, 1e1);
      Q.set(3, 3, 1e-6);
      Q.set(4, 4, 1e-6);
      Q.set(5, 5, 1e-6);

      R.set(0, 0, 1e2);
      R.set(1, 1, 1e2);
      R.set(2, 2, 1e-6);

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

      gradientExpected.set(0, 0, 0.5e-4);
      gradientExpected.set(1, 0, 0.25e-4);
      gradientExpected.set(2, 0, 2.0);
      gradientExpected.set(3, 0, 1e-2);
      gradientExpected.set(4, 0, 1e-2);
      gradientExpected.set(5, 0, 0.5e-2);

      costFunction.getCostStateGradient(currentControl, currentState, desiredControl, desiredState, gradient);

      JUnitTools.assertMatrixEquals(gradientExpected, gradient, 1e-10);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testComputeCostControlGradient()
   {
      DenseMatrix64F Q = new DenseMatrix64F(6, 6);
      DenseMatrix64F R = new DenseMatrix64F(3, 3);
      Q.set(0, 0, 1e-6);
      Q.set(1, 1, 1e-6);
      Q.set(2, 2, 1e1);
      Q.set(3, 3, 1e-6);
      Q.set(4, 4, 1e-6);
      Q.set(5, 5, 1e-6);

      R.set(0, 0, 1e2);
      R.set(1, 1, 1e2);
      R.set(2, 2, 1e-6);

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

      gradientExpected.set(0, 0, 0.3);
      gradientExpected.set(1, 0, -0.25);
      gradientExpected.set(2, 0, 50e-6);

      costFunction.getCostControlGradient(currentControl, currentState, desiredControl, desiredState, gradient);

      JUnitTools.assertMatrixEquals(gradientExpected, gradient, 1e-10);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testComputeCostControlHessian()
   {
      DenseMatrix64F R = new DenseMatrix64F(3, 3);
      R.set(0, 0, 1);
      R.set(1, 1, 1);
      R.set(2, 2, 1e-6);

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

      costFunction.getCostControlHessian(currentControl, currentState, hessian);

      JUnitTools.assertMatrixEquals(R, hessian, 1e-10);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testComputeCostStateHessian()
   {
      DenseMatrix64F Q = new DenseMatrix64F(6, 6);
      Q.set(0, 0, 1e-6);
      Q.set(1, 1, 1e-6);
      Q.set(2, 2, 1e1);
      Q.set(3, 3, 1e-6);
      Q.set(4, 4, 1e-6);
      Q.set(5, 5, 1e-6);

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

      costFunction.getCostStateHessian(currentControl, currentState, hessian);

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

      costFunction.getCostStateGradientOfControlGradient(currentControl, currentState, hessian);

      JUnitTools.assertMatrixEquals(hessianExpected, hessian, 1e-10);
   }
}
