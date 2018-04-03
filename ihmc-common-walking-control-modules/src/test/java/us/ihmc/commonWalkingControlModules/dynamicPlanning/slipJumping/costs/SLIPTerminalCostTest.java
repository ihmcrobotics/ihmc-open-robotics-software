package us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.costs;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.TrackingCostFunctionTest;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.trajectoryOptimization.LQTrackingCostFunction;

import java.math.BigDecimal;
import java.util.Random;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.*;

public class SLIPTerminalCostTest extends TrackingCostFunctionTest<SLIPState>
{
   public int getNumberOfStates()
   {
      return 2;
   }

   public int getStateVectorSize()
   {
      return SLIPState.stateVectorSize;
   }

   public int getControlVectorSize()
   {
      return SLIPState.controlVectorSize;
   }

   public int getConstantVectorSize()
   {
      return SLIPState.constantVectorSize;
   }

   public SLIPState getHybridState(int hybridStateIndex)
   {
      switch (hybridStateIndex)
      {
      case 0:
         return SLIPState.FLIGHT;
      default:
         return SLIPState.STANCE;
      }
   }

   public LQTrackingCostFunction<SLIPState> getCostFunction()
   {
      return new SLIPTerminalCost();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testCost()
   {
      LQTrackingCostFunction<SLIPState> costFunction = getCostFunction();

      Random random = new Random(1738L);
      DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize, 1);
      DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);
      DenseMatrix64F desiredState = RandomGeometry.nextDenseMatrix64F(random, getStateVectorSize(), 1);
      DenseMatrix64F desiredControl = RandomGeometry.nextDenseMatrix64F(random, getControlVectorSize(), 1);
      DenseMatrix64F constants = RandomGeometry.nextDenseMatrix64F(random, getConstantVectorSize(), 1);

      double cost = costFunction.getCost(SLIPState.STANCE, currentControl, currentState, desiredControl, desiredState, constants);

      double expectedCost = SLIPTerminalCost.qX * (currentState.get(x) - desiredState.get(x)) * (currentState.get(x) - desiredState.get(x));
      expectedCost += SLIPTerminalCost.qY * (currentState.get(y) - desiredState.get(y)) * (currentState.get(y) - desiredState.get(y));
      expectedCost += SLIPTerminalCost.qZ * (currentState.get(z) - desiredState.get(z)) * (currentState.get(z) - desiredState.get(z));

      expectedCost += SLIPTerminalCost.qThetaX * (currentState.get(thetaX) - desiredState.get(thetaX)) * (currentState.get(thetaX) - desiredState.get(thetaX));
      expectedCost += SLIPTerminalCost.qThetaY * (currentState.get(thetaY) - desiredState.get(thetaY)) * (currentState.get(thetaY) - desiredState.get(thetaY));
      expectedCost += SLIPTerminalCost.qThetaZ * (currentState.get(thetaZ) - desiredState.get(thetaZ)) * (currentState.get(thetaZ) - desiredState.get(thetaZ));

      expectedCost += SLIPTerminalCost.qXDot * (currentState.get(xDot) - desiredState.get(xDot)) * (currentState.get(xDot) - desiredState.get(xDot));
      expectedCost += SLIPTerminalCost.qYDot * (currentState.get(yDot) - desiredState.get(yDot)) * (currentState.get(yDot) - desiredState.get(yDot));
      expectedCost += SLIPTerminalCost.qZDot * (currentState.get(zDot) - desiredState.get(zDot)) * (currentState.get(zDot) - desiredState.get(zDot));

      expectedCost += SLIPTerminalCost.qThetaDotX * (currentState.get(thetaXDot) - desiredState.get(thetaXDot)) * (currentState.get(thetaXDot) - desiredState.get(thetaXDot));
      expectedCost += SLIPTerminalCost.qThetaDotY * (currentState.get(thetaYDot) - desiredState.get(thetaYDot)) * (currentState.get(thetaYDot) - desiredState.get(thetaYDot));
      expectedCost += SLIPTerminalCost.qThetaDotZ * (currentState.get(thetaZDot) - desiredState.get(thetaZDot)) * (currentState.get(thetaZDot) - desiredState.get(thetaZDot));

      expectedCost += SLIPTerminalCost.rFx * (currentControl.get(fx) - desiredControl.get(fx)) * (currentControl.get(fx) - desiredControl.get(fx));
      expectedCost += SLIPTerminalCost.rFy * (currentControl.get(fy) - desiredControl.get(fy)) * (currentControl.get(fy) - desiredControl.get(fy));
      expectedCost += SLIPTerminalCost.rFz * (currentControl.get(fz) - desiredControl.get(fz)) * (currentControl.get(fz) - desiredControl.get(fz));

      expectedCost += SLIPTerminalCost.rTauX * (currentControl.get(tauX) - desiredControl.get(tauX)) * (currentControl.get(tauX) - desiredControl.get(tauX));
      expectedCost += SLIPTerminalCost.rTauY * (currentControl.get(tauY) - desiredControl.get(tauY)) * (currentControl.get(tauY) - desiredControl.get(tauY));
      expectedCost += SLIPTerminalCost.rTauZ * (currentControl.get(tauZ) - desiredControl.get(tauZ)) * (currentControl.get(tauZ) - desiredControl.get(tauZ));

      expectedCost += SLIPTerminalCost.rXf * (currentControl.get(xF) - desiredControl.get(xF)) * (currentControl.get(xF) - desiredControl.get(xF));
      expectedCost += SLIPTerminalCost.rYf * (currentControl.get(yF) - desiredControl.get(yF)) * (currentControl.get(yF) - desiredControl.get(yF));

      expectedCost += SLIPTerminalCost.rK * (currentControl.get(k) - desiredControl.get(k)) * (currentControl.get(k) - desiredControl.get(k));

      int length = (int)(Math.log10(expectedCost) + 1);
      double scale = 1.0 * Math.pow(10, length);
      Assert.assertEquals(expectedCost, cost, scale * 1e-7);
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
