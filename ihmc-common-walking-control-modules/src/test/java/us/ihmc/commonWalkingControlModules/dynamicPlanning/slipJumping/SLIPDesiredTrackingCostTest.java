package us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.TrackingCostFunctionTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.trajectoryOptimization.LQTrackingCostFunction;

import java.util.Random;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.*;

public class SLIPDesiredTrackingCostTest extends TrackingCostFunctionTest<SLIPState>
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
      return new SLIPDesiredTrackingCost();
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

      double cost = costFunction.getCost(SLIPState.STANCE, currentControl, currentState, desiredControl, desiredState);

      double expectedCost = SLIPDesiredTrackingCost.qX * (currentState.get(x) - desiredState.get(x)) * (currentState.get(x) - desiredState.get(x));
      expectedCost += SLIPDesiredTrackingCost.qY * (currentState.get(y) - desiredState.get(y)) * (currentState.get(y) - desiredState.get(y));
      expectedCost += SLIPDesiredTrackingCost.qZ * (currentState.get(z) - desiredState.get(z)) * (currentState.get(z) - desiredState.get(z));

      expectedCost += SLIPDesiredTrackingCost.qThetaX * (currentState.get(thetaX) - desiredState.get(thetaX)) * (currentState.get(thetaX) - desiredState.get(thetaX));
      expectedCost += SLIPDesiredTrackingCost.qThetaY * (currentState.get(thetaY) - desiredState.get(thetaY)) * (currentState.get(thetaY) - desiredState.get(thetaY));
      expectedCost += SLIPDesiredTrackingCost.qThetaZ * (currentState.get(thetaZ) - desiredState.get(thetaZ)) * (currentState.get(thetaZ) - desiredState.get(thetaZ));

      expectedCost += SLIPDesiredTrackingCost.qXDot * (currentState.get(xDot) - desiredState.get(xDot)) * (currentState.get(xDot) - desiredState.get(xDot));
      expectedCost += SLIPDesiredTrackingCost.qYDot * (currentState.get(yDot) - desiredState.get(yDot)) * (currentState.get(yDot) - desiredState.get(yDot));
      expectedCost += SLIPDesiredTrackingCost.qZDot * (currentState.get(zDot) - desiredState.get(zDot)) * (currentState.get(zDot) - desiredState.get(zDot));

      expectedCost += SLIPDesiredTrackingCost.qThetaDotX * (currentState.get(thetaXDot) - desiredState.get(thetaXDot)) * (currentState.get(thetaXDot) - desiredState.get(thetaXDot));
      expectedCost += SLIPDesiredTrackingCost.qThetaDotY * (currentState.get(thetaYDot) - desiredState.get(thetaYDot)) * (currentState.get(thetaYDot) - desiredState.get(thetaYDot));
      expectedCost += SLIPDesiredTrackingCost.qThetaDotZ * (currentState.get(thetaZDot) - desiredState.get(thetaZDot)) * (currentState.get(thetaZDot) - desiredState.get(thetaZDot));

      expectedCost += SLIPDesiredTrackingCost.rFx * (currentControl.get(fx) - desiredControl.get(fx)) * (currentControl.get(fx) - desiredControl.get(fx));
      expectedCost += SLIPDesiredTrackingCost.rFy * (currentControl.get(fy) - desiredControl.get(fy)) * (currentControl.get(fy) - desiredControl.get(fy));
      expectedCost += SLIPDesiredTrackingCost.rFz * (currentControl.get(fz) - desiredControl.get(fz)) * (currentControl.get(fz) - desiredControl.get(fz));

      expectedCost += SLIPDesiredTrackingCost.rTauX * (currentControl.get(tauX) - desiredControl.get(tauX)) * (currentControl.get(tauX) - desiredControl.get(tauX));
      expectedCost += SLIPDesiredTrackingCost.rTauY * (currentControl.get(tauY) - desiredControl.get(tauY)) * (currentControl.get(tauY) - desiredControl.get(tauY));
      expectedCost += SLIPDesiredTrackingCost.rTauZ * (currentControl.get(tauZ) - desiredControl.get(tauZ)) * (currentControl.get(tauZ) - desiredControl.get(tauZ));

      expectedCost += SLIPDesiredTrackingCost.rXf * (currentControl.get(xF) - desiredControl.get(xF)) * (currentControl.get(xF) - desiredControl.get(xF));
      expectedCost += SLIPDesiredTrackingCost.rYf * (currentControl.get(yF) - desiredControl.get(yF)) * (currentControl.get(yF) - desiredControl.get(yF));

      expectedCost += SLIPDesiredTrackingCost.rK * (currentControl.get(k) - desiredControl.get(k)) * (currentControl.get(k) - desiredControl.get(k));

      Assert.assertEquals(expectedCost, cost, 1e-7);
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
