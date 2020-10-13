package us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.costs;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.controlVectorSize;
import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.fx;
import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.fy;
import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.fz;
import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.k;
import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.stateVectorSize;
import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.tauX;
import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.tauY;
import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.tauZ;
import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.thetaX;
import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.thetaXDot;
import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.thetaY;
import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.thetaYDot;
import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.thetaZ;
import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.thetaZDot;
import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.x;
import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.xDot;
import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.xF;
import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.y;
import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.yDot;
import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.yF;
import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.z;
import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.zDot;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.TrackingCostFunctionTest;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.trajectoryOptimization.LQTrackingCostFunction;

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
      DMatrixRMaj currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize, 1);
      DMatrixRMaj currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);
      DMatrixRMaj desiredState = RandomGeometry.nextDenseMatrix64F(random, getStateVectorSize(), 1);
      DMatrixRMaj desiredControl = RandomGeometry.nextDenseMatrix64F(random, getControlVectorSize(), 1);
      DMatrixRMaj constants = RandomGeometry.nextDenseMatrix64F(random, getConstantVectorSize(), 1);

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
