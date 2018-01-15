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

public class SLIPDesiredCostTest extends TrackingCostFunctionTest<SLIPState>
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
      double nominalLength = 1.5;
      LQTrackingCostFunction<SLIPState> costFunction = getCostFunction();

      Random random = new Random(1738L);
      DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize, 1);
      DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);
      DenseMatrix64F desiredState = RandomGeometry.nextDenseMatrix64F(random, getStateVectorSize(), 1);
      DenseMatrix64F desiredControl = RandomGeometry.nextDenseMatrix64F(random, getControlVectorSize(), 1);

      double cost = costFunction.getCost(SLIPState.STANCE, currentControl, currentState, desiredControl, desiredState);

      double x_k = currentState.get(x);
      double y_k = currentState.get(y);
      double z_k = currentState.get(z);

      double xf_k = currentControl.get(xF);
      double yf_k = currentControl.get(yF);

      double fx_k = currentControl.get(fx);
      double fy_k = currentControl.get(fy);
      double fz_k = currentControl.get(fz);

      double tauX_k = currentControl.get(tauX);
      double tauY_k = currentControl.get(tauY);
      double tauZ_k = currentControl.get(tauZ);

      double relativeX = x_k - xf_k;
      double relativeY = y_k - yf_k;


      double k_k = currentControl.get(k);

      double length = Math.sqrt(relativeX * relativeX + relativeY * relativeY + z_k * z_k);

      double force = k_k * (nominalLength / length - 1.0);

      double forceX = force * relativeX;
      double forceY = force * relativeY;
      double forceZ = force * z_k;
      double torqueX = -z_k * fy_k + relativeY * fz_k;
      double torqueY = z_k * fx_k - relativeX * fz_k;
      double torqueZ = -relativeY * fx_k + relativeX * fy_k;

      double expectedCost = SLIPModelForceTrackingCost.qFX * (fx_k - forceX) * (fx_k - forceX);
      expectedCost += SLIPModelForceTrackingCost.qFY * (fy_k - forceY) * (fy_k - forceY);
      expectedCost += SLIPModelForceTrackingCost.qFZ * (fz_k - forceZ) * (fz_k - forceZ);
      expectedCost += SLIPModelForceTrackingCost.qTauX * (tauX_k - torqueX) * (tauX_k - torqueX);
      expectedCost += SLIPModelForceTrackingCost.qTauY * (tauY_k - torqueY) * (tauY_k - torqueY);
      expectedCost += SLIPModelForceTrackingCost.qTauZ * (tauZ_k - torqueZ) * (tauZ_k - torqueZ);

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
