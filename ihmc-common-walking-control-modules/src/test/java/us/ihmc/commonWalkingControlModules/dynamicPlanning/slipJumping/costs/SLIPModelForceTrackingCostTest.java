package us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.costs;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.CostFunctionTest;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.testing.JUnitTools;
import us.ihmc.trajectoryOptimization.LQCostFunction;

import java.util.Random;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.*;

public class SLIPModelForceTrackingCostTest extends CostFunctionTest<SLIPState>
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

   public LQCostFunction<SLIPState> getCostFunction()
   {
      double mass = 15.0;
      double nominalLength = 1.5;
      double gravityZ = 9.81;
      return new SLIPModelForceTrackingCost(mass, gravityZ);
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testCost()
   {
      LQCostFunction<SLIPState> costFunction = getCostFunction();

      Random random = new Random(1738L);
      DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize, 1);
      DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);
      DenseMatrix64F constants = RandomGeometry.nextDenseMatrix64F(random, constantVectorSize, 1);
      constants.set(nominalLength, 0, RandomNumbers.nextDouble(random, 0.1, 10.0));

      double cost = costFunction.getCost(SLIPState.STANCE, currentControl, currentState, constants);

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

      double zf_k = constants.get(zF);
      double nominalPendulumLength = constants.get(nominalLength);

      double relativeX = x_k - xf_k;
      double relativeY = y_k - yf_k;
      double relativeZ = z_k - zf_k;


      double k_k = currentControl.get(k);

      double length = Math.sqrt(relativeX * relativeX + relativeY * relativeY + relativeZ * relativeZ);

      double force = k_k * (nominalPendulumLength / length - 1.0);

      double forceX = force * relativeX;
      double forceY = force * relativeY;
      double forceZ = force * relativeZ;
      double torqueX = -relativeZ * fy_k + relativeY * fz_k;
      double torqueY = relativeZ * fx_k - relativeX * fz_k;
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






   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testCostStateGradient()
   {
      double mass = 15.0;
      double gravityZ = 9.81;
      SLIPModelForceTrackingCost cost = new SLIPModelForceTrackingCost(mass, gravityZ);

      Random random = new Random(1738L);
      DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize, 1);
      DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);
      DenseMatrix64F constants = RandomGeometry.nextDenseMatrix64F(random, constantVectorSize, 1);
      constants.set(nominalLength, 0, RandomNumbers.nextDouble(random, 0.1, 10.0));

      DenseMatrix64F expectedGradient = new DenseMatrix64F(stateVectorSize, 1);
      DenseMatrix64F gradient = new DenseMatrix64F(stateVectorSize, 1);

      cost.getCostStateGradient(SLIPState.STANCE, currentControl, currentState, constants, gradient);

      double x_k = currentState.get(x);
      double y_k = currentState.get(y);
      double z_k = currentState.get(z);

      double xf_k = currentControl.get(xF);
      double yf_k = currentControl.get(yF);

      double zf_k = constants.get(zF);

      double fx_k = currentControl.get(fx);
      double fy_k = currentControl.get(fy);
      double fz_k = currentControl.get(fz);

      double tauX_k = currentControl.get(tauX);
      double tauY_k = currentControl.get(tauY);
      double tauZ_k = currentControl.get(tauZ);

      double nominalPendulumLength = constants.get(nominalLength);

      double relativeX = x_k - xf_k;
      double relativeY = y_k - yf_k;
      double relativeZ = z_k - zf_k;


      double k_k = currentControl.get(k);

      double length = Math.sqrt(relativeX * relativeX + relativeY * relativeY + relativeZ * relativeZ);


      double dynamicsXError = fx_k - k_k * (nominalPendulumLength / length - 1.0) * relativeX;
      double dynamicsYError = fy_k - k_k * (nominalPendulumLength / length - 1.0) * relativeY;
      double dynamicsZError = fz_k - k_k * (nominalPendulumLength / length - 1.0) * relativeZ;
      double dynamicsTauXError = tauX_k + relativeZ * fy_k - (y_k - yf_k) * fz_k;
      double dynamicsTauYError = tauY_k - relativeZ * fx_k - (xf_k - x_k) * fz_k;
      double dynamicsTauZError = tauZ_k - (yf_k - y_k) * fx_k - (x_k - xf_k) * fy_k;

      double gradientXXError = k_k * nominalPendulumLength * Math.pow(length, -3.0) * relativeX * relativeX - k_k * nominalPendulumLength / length + k_k;
      double gradientXYError = k_k * nominalPendulumLength * Math.pow(length, -3.0) * relativeX * relativeY;
      double gradientXZError = k_k * nominalPendulumLength * Math.pow(length, -3.0) * relativeX * relativeZ;
      double gradientXTauXError = 0.0;
      double gradientXTauYError = fz_k;
      double gradientXTauZError = -fy_k;

      double expectedXX = 2.0 * SLIPModelForceTrackingCost.qFX * gradientXXError * dynamicsXError;
      double expectedXY = 2.0 * SLIPModelForceTrackingCost.qFY * gradientXYError * dynamicsYError;
      double expectedXZ = 2.0 * SLIPModelForceTrackingCost.qFZ * gradientXZError * dynamicsZError;
      double expectedXTauX = 2.0 * SLIPModelForceTrackingCost.qTauX * gradientXTauXError * dynamicsTauXError;
      double expectedXTauY = 2.0 * SLIPModelForceTrackingCost.qTauY * gradientXTauYError * dynamicsTauYError;
      double expectedXTauZ = 2.0 * SLIPModelForceTrackingCost.qTauZ * gradientXTauZError * dynamicsTauZError;
      double expectedX = expectedXX + expectedXY + expectedXZ + expectedXTauX + expectedXTauY + expectedXTauZ;

      double gradientYXError = k_k * nominalPendulumLength * Math.pow(length, -3.0) * relativeX * relativeY;
      double gradientYYError = k_k * nominalPendulumLength * Math.pow(length, -3.0) * relativeY * relativeY - k_k * nominalPendulumLength / length + k_k;
      double gradientYZError = k_k * nominalPendulumLength * Math.pow(length, -3.0) * relativeY * relativeZ;
      double gradientYTauXError = -fz_k;
      double gradientYTauYError = 0.0;
      double gradientYTauZError = fx_k;

      double expectedYX = 2.0 * SLIPModelForceTrackingCost.qFX * gradientYXError * dynamicsXError;
      double expectedYY = 2.0 * SLIPModelForceTrackingCost.qFY * gradientYYError * dynamicsYError;
      double expectedYZ = 2.0 * SLIPModelForceTrackingCost.qFZ * gradientYZError * dynamicsZError;
      double expectedYTauX = 2.0 * SLIPModelForceTrackingCost.qTauX * gradientYTauXError * dynamicsTauXError;
      double expectedYTauY = 2.0 * SLIPModelForceTrackingCost.qTauY * gradientYTauYError * dynamicsTauYError;
      double expectedYTauZ = 2.0 * SLIPModelForceTrackingCost.qTauZ * gradientYTauZError * dynamicsTauZError;
      double expectedY = expectedYX + expectedYY + expectedYZ + expectedYTauX + expectedYTauY + expectedYTauZ;

      double gradientZXError = k_k * nominalPendulumLength * Math.pow(length, -3.0) * relativeX * relativeZ;
      double gradientZYError = k_k * nominalPendulumLength * Math.pow(length, -3.0) * relativeY * relativeZ;
      double gradientZZError = k_k * nominalPendulumLength * Math.pow(length, -3.0) * relativeZ * relativeZ - k_k * nominalPendulumLength / length + k_k;
      double gradientZTauXError = fy_k;
      double gradientZTauYError = -fx_k;
      double gradientZTauZError = 0.0;

      double expectedZX = 2.0 * SLIPModelForceTrackingCost.qFX * gradientZXError * dynamicsXError;
      double expectedZY = 2.0 * SLIPModelForceTrackingCost.qFY * gradientZYError * dynamicsYError;
      double expectedZZ = 2.0 * SLIPModelForceTrackingCost.qFZ * gradientZZError * dynamicsZError;
      double expectedZTauX = 2.0 * SLIPModelForceTrackingCost.qTauX * gradientZTauXError * dynamicsTauXError;
      double expectedZTauY = 2.0 * SLIPModelForceTrackingCost.qTauY * gradientZTauYError * dynamicsTauYError;
      double expectedZTauZ = 2.0 * SLIPModelForceTrackingCost.qTauZ * gradientZTauZError * dynamicsTauZError;
      double expectedZ = expectedZX + expectedZY + expectedZZ + expectedZTauX + expectedZTauY + expectedZTauZ;

      expectedGradient.set(x, expectedX);
      expectedGradient.set(y, expectedY);
      expectedGradient.set(z, expectedZ);

      JUnitTools.assertMatrixEquals(expectedGradient, gradient, 1e-7);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testCostControlGradient()
   {
      double mass = 15.0;
      double gravityZ = 9.81;
      SLIPModelForceTrackingCost cost = new SLIPModelForceTrackingCost(mass, gravityZ);

      Random random = new Random(1738L);
      DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize, 1);
      DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);
      DenseMatrix64F constants = RandomGeometry.nextDenseMatrix64F(random, constantVectorSize, 1);
      constants.set(nominalLength, 0, RandomNumbers.nextDouble(random, 0.1, 10.0));

      DenseMatrix64F expectedGradient = new DenseMatrix64F(controlVectorSize, 1);
      DenseMatrix64F gradient = new DenseMatrix64F(controlVectorSize, 1);

      cost.getCostControlGradient(SLIPState.STANCE, currentControl, currentState, constants, gradient);

      double x_k = currentState.get(x);
      double y_k = currentState.get(y);
      double z_k = currentState.get(z);

      double xf_k = currentControl.get(xF);
      double yf_k = currentControl.get(yF);
      double zf_k = constants.get(zF);

      double nominalPendulumLength = constants.get(nominalLength);

      double fx_k = currentControl.get(fx);
      double fy_k = currentControl.get(fy);
      double fz_k = currentControl.get(fz);

      double tauX_k = currentControl.get(tauX);
      double tauY_k = currentControl.get(tauY);
      double tauZ_k = currentControl.get(tauZ);

      double relativeX = x_k - xf_k;
      double relativeY = y_k - yf_k;
      double relativeZ = z_k - zf_k;


      double k_k = currentControl.get(k);

      double length = Math.sqrt(relativeX * relativeX + relativeY * relativeY + relativeZ * relativeZ);

      double dynamicsXError = fx_k - k_k * (nominalPendulumLength / length - 1.0) * relativeX;
      double dynamicsYError = fy_k - k_k * (nominalPendulumLength / length - 1.0) * relativeY;
      double dynamicsZError = fz_k - k_k * (nominalPendulumLength / length - 1.0) * relativeZ;
      double dynamicsTauXError = tauX_k - (zf_k - z_k) * fy_k - (y_k - yf_k) * fz_k;
      double dynamicsTauYError = tauY_k - (z_k - zf_k) * fx_k - (xf_k - x_k) * fz_k;
      double dynamicsTauZError = tauZ_k - (yf_k - y_k) * fx_k - (x_k - xf_k) * fy_k;

      double gradientFxXError = 1.0;
      double gradientFxYError = 0.0;
      double gradientFxZError = 0.0;
      double gradientFxTauXError = 0.0;
      double gradientFxTauYError = (zf_k - z_k);
      double gradientFxTauZError = (y_k - yf_k);

      double expectedFxX = 2.0 * SLIPModelForceTrackingCost.qFX * gradientFxXError * dynamicsXError;
      double expectedFxY = 2.0 * SLIPModelForceTrackingCost.qFY * gradientFxYError * dynamicsYError;
      double expectedFxZ = 2.0 * SLIPModelForceTrackingCost.qFZ * gradientFxZError * dynamicsZError;
      double expectedFxTauX = 2.0 * SLIPModelForceTrackingCost.qTauX * gradientFxTauXError * dynamicsTauXError;
      double expectedFxTauY = 2.0 * SLIPModelForceTrackingCost.qTauY * gradientFxTauYError * dynamicsTauYError;
      double expectedFxTauZ = 2.0 * SLIPModelForceTrackingCost.qTauZ * gradientFxTauZError * dynamicsTauZError;
      double expectedFx = expectedFxX + expectedFxY + expectedFxZ + expectedFxTauX + expectedFxTauY + expectedFxTauZ;

      double gradientFyXError = 0.0;
      double gradientFyYError = 1.0;
      double gradientFyZError = 0.0;
      double gradientFyTauXError = z_k - zf_k;
      double gradientFyTauYError = 0.0;
      double gradientFyTauZError = -(x_k - xf_k);

      double expectedFyX = 2.0 * SLIPModelForceTrackingCost.qFX * gradientFyXError * dynamicsXError;
      double expectedFyY = 2.0 * SLIPModelForceTrackingCost.qFY * gradientFyYError * dynamicsYError;
      double expectedFyZ = 2.0 * SLIPModelForceTrackingCost.qFZ * gradientFyZError * dynamicsZError;
      double expectedFyTauX = 2.0 * SLIPModelForceTrackingCost.qTauX * gradientFyTauXError * dynamicsTauXError;
      double expectedFyTauY = 2.0 * SLIPModelForceTrackingCost.qTauY * gradientFyTauYError * dynamicsTauYError;
      double expectedFyTauZ = 2.0 * SLIPModelForceTrackingCost.qTauZ * gradientFyTauZError * dynamicsTauZError;
      double expectedFy = expectedFyX + expectedFyY + expectedFyZ + expectedFyTauX + expectedFyTauY + expectedFyTauZ;

      double gradientFzXError = 0.0;
      double gradientFzYError = 0.0;
      double gradientFzZError = 1.0;
      double gradientFzTauXError = -(y_k - yf_k);
      double gradientFzTauYError = -(xf_k - x_k);
      double gradientFzTauZError = 0.0;

      double expectedFzX = 2.0 * SLIPModelForceTrackingCost.qFX * gradientFzXError * dynamicsXError;
      double expectedFzY = 2.0 * SLIPModelForceTrackingCost.qFY * gradientFzYError * dynamicsYError;
      double expectedFzZ = 2.0 * SLIPModelForceTrackingCost.qFZ * gradientFzZError * dynamicsZError;
      double expectedFzTauX = 2.0 * SLIPModelForceTrackingCost.qTauX * gradientFzTauXError * dynamicsTauXError;
      double expectedFzTauY = 2.0 * SLIPModelForceTrackingCost.qTauY * gradientFzTauYError * dynamicsTauYError;
      double expectedFzTauZ = 2.0 * SLIPModelForceTrackingCost.qTauZ * gradientFzTauZError * dynamicsTauZError;
      double expectedFz = expectedFzX + expectedFzY + expectedFzZ + expectedFzTauX + expectedFzTauY + expectedFzTauZ;

      double expectedTauX = 2.0 * SLIPModelForceTrackingCost.qTauX * dynamicsTauXError;
      double expectedTauY = 2.0 * SLIPModelForceTrackingCost.qTauY * dynamicsTauYError;
      double expectedTauZ = 2.0 * SLIPModelForceTrackingCost.qTauZ * dynamicsTauZError;

      // TODO xf, yf, k
      double gradientXfXError = -k_k * (relativeX * relativeX * nominalPendulumLength * Math.pow(length, -3.0) - nominalPendulumLength / length + 1.0);
      double gradientXfYError = -k_k * nominalPendulumLength * Math.pow(length, -3.0) * relativeX * relativeY;
      double gradientXfZError = -k_k * nominalPendulumLength * Math.pow(length, -3.0) * relativeX * relativeZ;
      double gradientXfTauXError = 0.0;
      double gradientXfTauYError = -fz_k;
      double gradientXfTauZError = fy_k;

      double expectedXfX = 2.0 * SLIPModelForceTrackingCost.qFX * gradientXfXError * dynamicsXError;
      double expectedXfY = 2.0 * SLIPModelForceTrackingCost.qFY * gradientXfYError * dynamicsYError;
      double expectedXfZ = 2.0 * SLIPModelForceTrackingCost.qFZ * gradientXfZError * dynamicsZError;
      double expectedXfTauX = 2.0 * SLIPModelForceTrackingCost.qTauX * gradientXfTauXError * dynamicsTauXError;
      double expectedXfTauY = 2.0 * SLIPModelForceTrackingCost.qTauY * gradientXfTauYError * dynamicsTauYError;
      double expectedXfTauZ = 2.0 * SLIPModelForceTrackingCost.qTauZ * gradientXfTauZError * dynamicsTauZError;
      double expectedXf = expectedXfX + expectedXfY + expectedXfZ + expectedXfTauX + expectedXfTauY + expectedXfTauZ;

      double gradientYfXError = -k_k * nominalPendulumLength * Math.pow(length, -3.0) * relativeX * relativeY;
      double gradientYfYError = -k_k * (nominalPendulumLength * Math.pow(length, -3.0) * relativeY * relativeY - nominalPendulumLength / length + 1.0);
      double gradientYfZError = -k_k * nominalPendulumLength * Math.pow(length, -3.0) * relativeY * relativeZ;
      double gradientYfTauXError = fz_k;
      double gradientYfTauYError = 0.0;
      double gradientYfTauZError = -fx_k;

      double expectedYfX = 2.0 * SLIPModelForceTrackingCost.qFX * gradientYfXError * dynamicsXError;
      double expectedYfY = 2.0 * SLIPModelForceTrackingCost.qFY * gradientYfYError * dynamicsYError;
      double expectedYfZ = 2.0 * SLIPModelForceTrackingCost.qFZ * gradientYfZError * dynamicsZError;
      double expectedYfTauX = 2.0 * SLIPModelForceTrackingCost.qTauX * gradientYfTauXError * dynamicsTauXError;
      double expectedYfTauY = 2.0 * SLIPModelForceTrackingCost.qTauY * gradientYfTauYError * dynamicsTauYError;
      double expectedYfTauZ = 2.0 * SLIPModelForceTrackingCost.qTauZ * gradientYfTauZError * dynamicsTauZError;
      double expectedYf = expectedYfX + expectedYfY + expectedYfZ + expectedYfTauX + expectedYfTauY + expectedYfTauZ;

      double gradientKXError = -(nominalPendulumLength / length - 1.0) * relativeX;
      double gradientKYError = -(nominalPendulumLength / length - 1.0) * relativeY;
      double gradientKZError = -(nominalPendulumLength / length - 1.0) * relativeZ;
      double gradientKTauXError = 0;
      double gradientKTauYError = 0;
      double gradientKTauZError = 0;

      double expectedKX = 2.0 * SLIPModelForceTrackingCost.qFX * gradientKXError * dynamicsXError;
      double expectedKY = 2.0 * SLIPModelForceTrackingCost.qFY * gradientKYError * dynamicsYError;
      double expectedKZ = 2.0 * SLIPModelForceTrackingCost.qFZ * gradientKZError * dynamicsZError;
      double expectedKTauX = 2.0 * SLIPModelForceTrackingCost.qTauX * gradientKTauXError * dynamicsTauXError;
      double expectedKTauY = 2.0 * SLIPModelForceTrackingCost.qTauY * gradientKTauYError * dynamicsTauYError;
      double expectedKTauZ = 2.0 * SLIPModelForceTrackingCost.qTauZ * gradientKTauZError * dynamicsTauZError;
      double expectedK = expectedKX + expectedKY + expectedKZ + expectedKTauX + expectedKTauY + expectedKTauZ;

      expectedGradient.set(fx, expectedFx);
      expectedGradient.set(fy, expectedFy);
      expectedGradient.set(fz, expectedFz);
      expectedGradient.set(tauX, expectedTauX);
      expectedGradient.set(tauY, expectedTauY);
      expectedGradient.set(tauZ, expectedTauZ);
      expectedGradient.set(xF, expectedXf);
      expectedGradient.set(yF, expectedYf);
      expectedGradient.set(k, expectedK);

      JUnitTools.assertMatrixEquals(expectedGradient, gradient, 1e-7);
   }

}
