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
