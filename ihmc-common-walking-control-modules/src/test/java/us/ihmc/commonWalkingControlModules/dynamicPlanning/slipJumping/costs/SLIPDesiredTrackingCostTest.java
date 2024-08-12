package us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.costs;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.RandomMatrices_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.TrackingCostFunctionTest;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState;
import us.ihmc.robotics.Assert;
import us.ihmc.trajectoryOptimization.LQTrackingCostFunction;

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
      return new SLIPDesiredTrackingCost();
   }

   @Override
   @Test
   public void testCost()
   {
      LQTrackingCostFunction<SLIPState> costFunction = getCostFunction();

      Random random = new Random(1738L);
      DMatrixRMaj currentState = RandomMatrices_DDRM.rectangle(stateVectorSize, 1, random);
      DMatrixRMaj currentControl = RandomMatrices_DDRM.rectangle(controlVectorSize, 1, random);
      DMatrixRMaj desiredState = RandomMatrices_DDRM.rectangle(getStateVectorSize(), 1, random);
      DMatrixRMaj desiredControl = RandomMatrices_DDRM.rectangle(getControlVectorSize(), 1, random);
      DMatrixRMaj constants = RandomMatrices_DDRM.rectangle(getConstantVectorSize(), 1, random);

      double cost = costFunction.getCost(SLIPState.FLIGHT, currentControl, currentState, desiredControl, desiredState, constants);

      double expectedCost = SLIPDesiredTrackingCost.qXFlight * (currentState.get(x) - desiredState.get(x)) * (currentState.get(x) - desiredState.get(x));
      expectedCost += SLIPDesiredTrackingCost.qYFlight * (currentState.get(y) - desiredState.get(y)) * (currentState.get(y) - desiredState.get(y));
      expectedCost += SLIPDesiredTrackingCost.qZFlight * (currentState.get(z) - desiredState.get(z)) * (currentState.get(z) - desiredState.get(z));

      expectedCost += SLIPDesiredTrackingCost.qThetaXFlight * (currentState.get(thetaX) - desiredState.get(thetaX)) * (currentState.get(thetaX) - desiredState.get(thetaX));
      expectedCost += SLIPDesiredTrackingCost.qThetaYFlight * (currentState.get(thetaY) - desiredState.get(thetaY)) * (currentState.get(thetaY) - desiredState.get(thetaY));
      expectedCost += SLIPDesiredTrackingCost.qThetaZFlight * (currentState.get(thetaZ) - desiredState.get(thetaZ)) * (currentState.get(thetaZ) - desiredState.get(thetaZ));

      expectedCost += SLIPDesiredTrackingCost.qXDotFlight * (currentState.get(xDot) - desiredState.get(xDot)) * (currentState.get(xDot) - desiredState.get(xDot));
      expectedCost += SLIPDesiredTrackingCost.qYDotFlight * (currentState.get(yDot) - desiredState.get(yDot)) * (currentState.get(yDot) - desiredState.get(yDot));
      expectedCost += SLIPDesiredTrackingCost.qZDotFlight * (currentState.get(zDot) - desiredState.get(zDot)) * (currentState.get(zDot) - desiredState.get(zDot));

      expectedCost += SLIPDesiredTrackingCost.qThetaDotXFlight * (currentState.get(thetaXDot) - desiredState.get(thetaXDot)) * (currentState.get(thetaXDot) - desiredState.get(thetaXDot));
      expectedCost += SLIPDesiredTrackingCost.qThetaDotYFlight * (currentState.get(thetaYDot) - desiredState.get(thetaYDot)) * (currentState.get(thetaYDot) - desiredState.get(thetaYDot));
      expectedCost += SLIPDesiredTrackingCost.qThetaDotZFlight * (currentState.get(thetaZDot) - desiredState.get(thetaZDot)) * (currentState.get(thetaZDot) - desiredState.get(thetaZDot));

      expectedCost += SLIPDesiredTrackingCost.rFxFlight * (currentControl.get(fx) - desiredControl.get(fx)) * (currentControl.get(fx) - desiredControl.get(fx));
      expectedCost += SLIPDesiredTrackingCost.rFyFlight * (currentControl.get(fy) - desiredControl.get(fy)) * (currentControl.get(fy) - desiredControl.get(fy));
      expectedCost += SLIPDesiredTrackingCost.rFzFlight * (currentControl.get(fz) - desiredControl.get(fz)) * (currentControl.get(fz) - desiredControl.get(fz));

      expectedCost += SLIPDesiredTrackingCost.rTauXFlight * (currentControl.get(tauX) - desiredControl.get(tauX)) * (currentControl.get(tauX) - desiredControl.get(tauX));
      expectedCost += SLIPDesiredTrackingCost.rTauYFlight * (currentControl.get(tauY) - desiredControl.get(tauY)) * (currentControl.get(tauY) - desiredControl.get(tauY));
      expectedCost += SLIPDesiredTrackingCost.rTauZFlight * (currentControl.get(tauZ) - desiredControl.get(tauZ)) * (currentControl.get(tauZ) - desiredControl.get(tauZ));

      expectedCost += SLIPDesiredTrackingCost.rXfFlight * (currentControl.get(xF) - desiredControl.get(xF)) * (currentControl.get(xF) - desiredControl.get(xF));
      expectedCost += SLIPDesiredTrackingCost.rYfFlight * (currentControl.get(yF) - desiredControl.get(yF)) * (currentControl.get(yF) - desiredControl.get(yF));

      expectedCost += SLIPDesiredTrackingCost.rKFlight * (currentControl.get(k) - desiredControl.get(k)) * (currentControl.get(k) - desiredControl.get(k));

      Assert.assertEquals(expectedCost, cost, 1e-7);




      cost = costFunction.getCost(SLIPState.STANCE, currentControl, currentState, desiredControl, desiredState, constants);

      expectedCost = SLIPDesiredTrackingCost.qXStance * (currentState.get(x) - desiredState.get(x)) * (currentState.get(x) - desiredState.get(x));
      expectedCost += SLIPDesiredTrackingCost.qYStance * (currentState.get(y) - desiredState.get(y)) * (currentState.get(y) - desiredState.get(y));
      expectedCost += SLIPDesiredTrackingCost.qZStance * (currentState.get(z) - desiredState.get(z)) * (currentState.get(z) - desiredState.get(z));

      expectedCost += SLIPDesiredTrackingCost.qThetaXStance * (currentState.get(thetaX) - desiredState.get(thetaX)) * (currentState.get(thetaX) - desiredState.get(thetaX));
      expectedCost += SLIPDesiredTrackingCost.qThetaYStance * (currentState.get(thetaY) - desiredState.get(thetaY)) * (currentState.get(thetaY) - desiredState.get(thetaY));
      expectedCost += SLIPDesiredTrackingCost.qThetaZStance * (currentState.get(thetaZ) - desiredState.get(thetaZ)) * (currentState.get(thetaZ) - desiredState.get(thetaZ));

      expectedCost += SLIPDesiredTrackingCost.qXDotStance * (currentState.get(xDot) - desiredState.get(xDot)) * (currentState.get(xDot) - desiredState.get(xDot));
      expectedCost += SLIPDesiredTrackingCost.qYDotStance * (currentState.get(yDot) - desiredState.get(yDot)) * (currentState.get(yDot) - desiredState.get(yDot));
      expectedCost += SLIPDesiredTrackingCost.qZDotStance * (currentState.get(zDot) - desiredState.get(zDot)) * (currentState.get(zDot) - desiredState.get(zDot));

      expectedCost += SLIPDesiredTrackingCost.qThetaDotXStance * (currentState.get(thetaXDot) - desiredState.get(thetaXDot)) * (currentState.get(thetaXDot) - desiredState.get(thetaXDot));
      expectedCost += SLIPDesiredTrackingCost.qThetaDotYStance * (currentState.get(thetaYDot) - desiredState.get(thetaYDot)) * (currentState.get(thetaYDot) - desiredState.get(thetaYDot));
      expectedCost += SLIPDesiredTrackingCost.qThetaDotZStance * (currentState.get(thetaZDot) - desiredState.get(thetaZDot)) * (currentState.get(thetaZDot) - desiredState.get(thetaZDot));

      expectedCost += SLIPDesiredTrackingCost.rFxStance * (currentControl.get(fx) - desiredControl.get(fx)) * (currentControl.get(fx) - desiredControl.get(fx));
      expectedCost += SLIPDesiredTrackingCost.rFyStance * (currentControl.get(fy) - desiredControl.get(fy)) * (currentControl.get(fy) - desiredControl.get(fy));
      expectedCost += SLIPDesiredTrackingCost.rFzStance * (currentControl.get(fz) - desiredControl.get(fz)) * (currentControl.get(fz) - desiredControl.get(fz));

      expectedCost += SLIPDesiredTrackingCost.rTauXStance * (currentControl.get(tauX) - desiredControl.get(tauX)) * (currentControl.get(tauX) - desiredControl.get(tauX));
      expectedCost += SLIPDesiredTrackingCost.rTauYStance * (currentControl.get(tauY) - desiredControl.get(tauY)) * (currentControl.get(tauY) - desiredControl.get(tauY));
      expectedCost += SLIPDesiredTrackingCost.rTauZStance * (currentControl.get(tauZ) - desiredControl.get(tauZ)) * (currentControl.get(tauZ) - desiredControl.get(tauZ));

      expectedCost += SLIPDesiredTrackingCost.rXfStance * (currentControl.get(xF) - desiredControl.get(xF)) * (currentControl.get(xF) - desiredControl.get(xF));
      expectedCost += SLIPDesiredTrackingCost.rYfStance * (currentControl.get(yF) - desiredControl.get(yF)) * (currentControl.get(yF) - desiredControl.get(yF));

      expectedCost += SLIPDesiredTrackingCost.rKStance * (currentControl.get(k) - desiredControl.get(k)) * (currentControl.get(k) - desiredControl.get(k));

      Assert.assertEquals(expectedCost, cost, 1e-7);
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
