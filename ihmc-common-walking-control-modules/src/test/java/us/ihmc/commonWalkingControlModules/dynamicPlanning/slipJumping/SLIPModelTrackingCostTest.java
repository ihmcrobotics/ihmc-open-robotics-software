package us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.testing.JUnitTools;

import java.util.Random;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.*;

public class SLIPModelTrackingCostTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testCost()
   {
      double mass = 15.0;
      double nominalLength = 1.5;
      double gravityZ = 9.81;
      SLIPModelTrackingCost costFunction = new SLIPModelTrackingCost(mass, nominalLength, gravityZ);

      Random random = new Random(1738L);
      DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize, 1);
      DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);

      double cost = costFunction.getCost(SLIPState.STANCE, currentControl, currentState);

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

      double expectedCost = SLIPModelTrackingCost.qFX * (fx_k - forceX) * (fx_k - forceX);
      expectedCost += SLIPModelTrackingCost.qFY * (fy_k - forceY) * (fy_k - forceY);
      expectedCost += SLIPModelTrackingCost.qFZ * (fz_k - forceZ) * (fz_k - forceZ);
      expectedCost += SLIPModelTrackingCost.qTauX * (tauX_k - torqueX) * (tauX_k - torqueX);
      expectedCost += SLIPModelTrackingCost.qTauY * (tauY_k - torqueY) * (tauY_k - torqueY);
      expectedCost += SLIPModelTrackingCost.qTauZ * (tauZ_k - torqueZ) * (tauZ_k - torqueZ);

      Assert.assertEquals(expectedCost, cost, 1e-7);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testCostStateGradientNumericalDifferentiationStance()
   {
      double epsilon = 1e-9;
      double mass = 15.0;
      double nominalLength = 1.5;
      double gravityZ = 9.81;
      SLIPModelTrackingCost costFunction = new SLIPModelTrackingCost(mass, nominalLength, gravityZ);

      Random random = new Random(1738L);
      DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize, 1);
      DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);

      DenseMatrix64F costGradient = new DenseMatrix64F(stateVectorSize, 1);
      DenseMatrix64F expectedCostGradient = new DenseMatrix64F(stateVectorSize, 1);


      double cost = costFunction.getCost(SLIPState.STANCE, currentControl, currentState);

      costFunction.getCostStateGradient(SLIPState.STANCE, currentControl, currentState, costGradient);

      DenseMatrix64F currentStateModified = new DenseMatrix64F(currentState);
      currentStateModified.add(x, 0, epsilon);
      double modifiedCost = costFunction.getCost(SLIPState.STANCE, currentControl, currentStateModified);
      expectedCostGradient.set(x, 0, (modifiedCost - cost) / epsilon);


      currentStateModified.set(currentState);
      currentStateModified.add(y, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.STANCE, currentControl, currentStateModified);
      expectedCostGradient.set(y, 0, (modifiedCost - cost) / epsilon);


      currentStateModified.set(currentState);
      currentStateModified.add(z, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.STANCE, currentControl, currentStateModified);
      expectedCostGradient.set(z, 0, (modifiedCost - cost) / epsilon);


      currentStateModified.set(currentState);
      currentStateModified.add(thetaX, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.STANCE, currentControl, currentStateModified);
      expectedCostGradient.set(thetaX, 0, (modifiedCost - cost) / epsilon);


      currentStateModified.set(currentState);
      currentStateModified.add(thetaY, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.STANCE, currentControl, currentStateModified);
      expectedCostGradient.set(thetaY, 0, (modifiedCost - cost) / epsilon);


      currentStateModified.set(currentState);
      currentStateModified.add(thetaZ, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.STANCE, currentControl, currentStateModified);
      expectedCostGradient.set(thetaZ, 0, (modifiedCost - cost) / epsilon);


      currentStateModified.set(currentState);
      currentStateModified.add(xDot, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.STANCE, currentControl, currentStateModified);
      expectedCostGradient.set(xDot, 0, (modifiedCost - cost) / epsilon);


      currentStateModified.set(currentState);
      currentStateModified.add(yDot, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.STANCE, currentControl, currentStateModified);
      expectedCostGradient.set(yDot, 0, (modifiedCost - cost) / epsilon);


      currentStateModified.set(currentState);
      currentStateModified.add(zDot, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.STANCE, currentControl, currentStateModified);
      expectedCostGradient.set(zDot, 0, (modifiedCost - cost) / epsilon);


      currentStateModified.set(currentState);
      currentStateModified.add(thetaXDot, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.STANCE, currentControl, currentStateModified);
      expectedCostGradient.set(thetaXDot, 0, (modifiedCost - cost) / epsilon);


      currentStateModified.set(currentState);
      currentStateModified.add(thetaYDot, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.STANCE, currentControl, currentStateModified);
      expectedCostGradient.set(thetaYDot, 0, (modifiedCost - cost) / epsilon);


      currentStateModified.set(currentState);
      currentStateModified.add(thetaZDot, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.STANCE, currentControl, currentStateModified);
      expectedCostGradient.set(thetaZDot, 0, (modifiedCost - cost) / epsilon);


      JUnitTools.assertMatrixEquals(expectedCostGradient, costGradient, 1e-2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testCostStateGradientNumericalDifferentiationFlight()
   {
      double epsilon = 1e-7;
      double mass = 15.0;
      double nominalLength = 1.5;
      double gravityZ = 9.81;
      SLIPModelTrackingCost costFunction = new SLIPModelTrackingCost(mass, nominalLength, gravityZ);

      Random random = new Random(1738L);
      DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize, 1);
      DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);

      DenseMatrix64F costGradient = new DenseMatrix64F(stateVectorSize, 1);
      DenseMatrix64F expectedCostGradient = new DenseMatrix64F(stateVectorSize, 1);


      double cost = costFunction.getCost(SLIPState.FLIGHT, currentControl, currentState);

      costFunction.getCostStateGradient(SLIPState.FLIGHT, currentControl, currentState, costGradient);

      DenseMatrix64F currentStateModified = new DenseMatrix64F(currentState);
      currentStateModified.add(x, 0, epsilon);
      double modifiedCost = costFunction.getCost(SLIPState.FLIGHT, currentControl, currentStateModified);
      expectedCostGradient.set(x, 0, (modifiedCost - cost) / epsilon);


      currentStateModified.set(currentState);
      currentStateModified.add(y, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.FLIGHT, currentControl, currentStateModified);
      expectedCostGradient.set(y, 0, (modifiedCost - cost) / epsilon);


      currentStateModified.set(currentState);
      currentStateModified.add(z, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.FLIGHT, currentControl, currentStateModified);
      expectedCostGradient.set(z, 0, (modifiedCost - cost) / epsilon);


      currentStateModified.set(currentState);
      currentStateModified.add(thetaX, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.FLIGHT, currentControl, currentStateModified);
      expectedCostGradient.set(thetaX, 0, (modifiedCost - cost) / epsilon);


      currentStateModified.set(currentState);
      currentStateModified.add(thetaY, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.FLIGHT, currentControl, currentStateModified);
      expectedCostGradient.set(thetaY, 0, (modifiedCost - cost) / epsilon);


      currentStateModified.set(currentState);
      currentStateModified.add(thetaZ, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.FLIGHT, currentControl, currentStateModified);
      expectedCostGradient.set(thetaZ, 0, (modifiedCost - cost) / epsilon);


      currentStateModified.set(currentState);
      currentStateModified.add(xDot, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.FLIGHT, currentControl, currentStateModified);
      expectedCostGradient.set(xDot, 0, (modifiedCost - cost) / epsilon);


      currentStateModified.set(currentState);
      currentStateModified.add(yDot, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.FLIGHT, currentControl, currentStateModified);
      expectedCostGradient.set(yDot, 0, (modifiedCost - cost) / epsilon);


      currentStateModified.set(currentState);
      currentStateModified.add(zDot, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.FLIGHT, currentControl, currentStateModified);
      expectedCostGradient.set(zDot, 0, (modifiedCost - cost) / epsilon);


      currentStateModified.set(currentState);
      currentStateModified.add(thetaXDot, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.FLIGHT, currentControl, currentStateModified);
      expectedCostGradient.set(thetaXDot, 0, (modifiedCost - cost) / epsilon);


      currentStateModified.set(currentState);
      currentStateModified.add(thetaYDot, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.FLIGHT, currentControl, currentStateModified);
      expectedCostGradient.set(thetaYDot, 0, (modifiedCost - cost) / epsilon);


      currentStateModified.set(currentState);
      currentStateModified.add(thetaZDot, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.FLIGHT, currentControl, currentStateModified);
      expectedCostGradient.set(thetaZDot, 0, (modifiedCost - cost) / epsilon);


      JUnitTools.assertMatrixEquals(expectedCostGradient, costGradient, 1e-7);
   }


   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testCostControlGradientNumericalDifferentiationStance()
   {
      double epsilon = 1e-9;
      double mass = 15.0;
      double nominalLength = 1.5;
      double gravityZ = 9.81;
      SLIPModelTrackingCost costFunction = new SLIPModelTrackingCost(mass, nominalLength, gravityZ);

      Random random = new Random(1738L);
      DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize, 1);
      DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);

      DenseMatrix64F costGradient = new DenseMatrix64F(controlVectorSize, 1);
      DenseMatrix64F expectedCostGradient = new DenseMatrix64F(controlVectorSize, 1);


      double cost = costFunction.getCost(SLIPState.STANCE, currentControl, currentState);

      costFunction.getCostControlGradient(SLIPState.STANCE, currentControl, currentState, costGradient);

      DenseMatrix64F currentControlModified = new DenseMatrix64F(currentControl);
      currentControlModified.add(fx, 0, epsilon);
      double modifiedCost = costFunction.getCost(SLIPState.STANCE, currentControlModified, currentState);
      expectedCostGradient.set(fx, 0, (modifiedCost - cost) / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(fy, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.STANCE, currentControlModified, currentState);
      expectedCostGradient.set(fy, 0, (modifiedCost - cost) / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(fz, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.STANCE, currentControlModified, currentState);
      expectedCostGradient.set(fz, 0, (modifiedCost - cost) / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(tauX, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.STANCE, currentControlModified, currentState);
      expectedCostGradient.set(tauX, 0, (modifiedCost - cost) / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(tauY, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.STANCE, currentControlModified, currentState);
      expectedCostGradient.set(tauY, 0, (modifiedCost - cost) / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(tauZ, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.STANCE, currentControlModified, currentState);
      expectedCostGradient.set(tauZ, 0, (modifiedCost - cost) / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(xF, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.STANCE, currentControlModified, currentState);
      expectedCostGradient.set(xF, 0, (modifiedCost - cost) / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(yF, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.STANCE, currentControlModified, currentState);
      expectedCostGradient.set(yF, 0, (modifiedCost - cost) / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(k, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.STANCE, currentControlModified, currentState);
      expectedCostGradient.set(k, 0, (modifiedCost - cost) / epsilon);


      JUnitTools.assertMatrixEquals(expectedCostGradient, costGradient, 1e-2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testCostControlGradientNumericalDifferentiationFlight()
   {
      double epsilon = 1e-7;
      double mass = 15.0;
      double nominalLength = 1.5;
      double gravityZ = 9.81;
      SLIPModelTrackingCost costFunction = new SLIPModelTrackingCost(mass, nominalLength, gravityZ);

      Random random = new Random(1738L);
      DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize, 1);
      DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);

      DenseMatrix64F costGradient = new DenseMatrix64F(controlVectorSize, 1);
      DenseMatrix64F expectedCostGradient = new DenseMatrix64F(controlVectorSize, 1);


      double cost = costFunction.getCost(SLIPState.FLIGHT, currentControl, currentState);

      costFunction.getCostControlGradient(SLIPState.FLIGHT, currentControl, currentState, costGradient);

      DenseMatrix64F currentControlModified = new DenseMatrix64F(currentControl);
      currentControlModified.add(fx, 0, epsilon);
      double modifiedCost = costFunction.getCost(SLIPState.FLIGHT, currentControlModified, currentState);
      expectedCostGradient.set(fx, 0, (modifiedCost - cost) / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(fy, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.FLIGHT, currentControlModified, currentState);
      expectedCostGradient.set(fy, 0, (modifiedCost - cost) / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(fz, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.FLIGHT, currentControlModified, currentState);
      expectedCostGradient.set(fz, 0, (modifiedCost - cost) / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(tauX, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.FLIGHT, currentControlModified, currentState);
      expectedCostGradient.set(tauX, 0, (modifiedCost - cost) / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(tauY, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.FLIGHT, currentControlModified, currentState);
      expectedCostGradient.set(tauY, 0, (modifiedCost - cost) / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(tauZ, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.FLIGHT, currentControlModified, currentState);
      expectedCostGradient.set(tauZ, 0, (modifiedCost - cost) / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(xF, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.FLIGHT, currentControlModified, currentState);
      expectedCostGradient.set(xF, 0, (modifiedCost - cost) / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(yF, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.FLIGHT, currentControlModified, currentState);
      expectedCostGradient.set(yF, 0, (modifiedCost - cost) / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(k, 0, epsilon);
      modifiedCost = costFunction.getCost(SLIPState.FLIGHT, currentControlModified, currentState);
      expectedCostGradient.set(k, 0, (modifiedCost - cost) / epsilon);


      JUnitTools.assertMatrixEquals(expectedCostGradient, costGradient, 1e-7);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testCostStateGradient()
   {
      double mass = 15.0;
      double nominalLength = 1.5;
      double gravityZ = 9.81;
      SLIPModelTrackingCost cost = new SLIPModelTrackingCost(mass, nominalLength, gravityZ);

      Random random = new Random(1738L);
      DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize, 1);
      DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);

      DenseMatrix64F expectedGradient = new DenseMatrix64F(stateVectorSize, 1);
      DenseMatrix64F gradient = new DenseMatrix64F(stateVectorSize, 1);

      cost.getCostStateGradient(SLIPState.STANCE, currentControl, currentState, gradient);

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


      double dynamicsXError = fx_k - k_k * (nominalLength / length - 1.0) * relativeX;
      double dynamicsYError = fy_k - k_k * (nominalLength / length - 1.0) * relativeY;
      double dynamicsZError = fz_k - k_k * (nominalLength / length - 1.0) * z_k;
      double dynamicsTauXError = tauX_k + z_k * fy_k - (y_k - yf_k) * fz_k;
      double dynamicsTauYError = tauY_k - z_k * fx_k - (xf_k - x_k) * fz_k;
      double dynamicsTauZError = tauZ_k - (yf_k - y_k) * fx_k - (x_k - xf_k) * fy_k;

      double gradientXXError = k_k * nominalLength * Math.pow(length, -3.0) * relativeX * relativeX - k_k * nominalLength / length + k_k;
      double gradientXYError = k_k * nominalLength * Math.pow(length, -3.0) * relativeX * relativeY;
      double gradientXZError = k_k * nominalLength * Math.pow(length, -3.0) * relativeX * z_k;
      double gradientXTauXError = 0.0;
      double gradientXTauYError = fz_k;
      double gradientXTauZError = -fy_k;

      double expectedXX = 2.0 * SLIPModelTrackingCost.qFX * gradientXXError * dynamicsXError;
      double expectedXY = 2.0 * SLIPModelTrackingCost.qFY * gradientXYError * dynamicsYError;
      double expectedXZ = 2.0 * SLIPModelTrackingCost.qFZ * gradientXZError * dynamicsZError;
      double expectedXTauX = 2.0 * SLIPModelTrackingCost.qTauX * gradientXTauXError * dynamicsTauXError;
      double expectedXTauY = 2.0 * SLIPModelTrackingCost.qTauY * gradientXTauYError * dynamicsTauYError;
      double expectedXTauZ = 2.0 * SLIPModelTrackingCost.qTauZ * gradientXTauZError * dynamicsTauZError;
      double expectedX = expectedXX + expectedXY + expectedXZ + expectedXTauX + expectedXTauY + expectedXTauZ;

      double gradientYXError = k_k * nominalLength * Math.pow(length, -3.0) * relativeX * relativeY;
      double gradientYYError = k_k * nominalLength * Math.pow(length, -3.0) * relativeY * relativeY - k_k * nominalLength / length + k_k;
      double gradientYZError = k_k * nominalLength * Math.pow(length, -3.0) * relativeY * z_k;
      double gradientYTauXError = -fz_k;
      double gradientYTauYError = 0.0;
      double gradientYTauZError = fx_k;

      double expectedYX = 2.0 * SLIPModelTrackingCost.qFX * gradientYXError * dynamicsXError;
      double expectedYY = 2.0 * SLIPModelTrackingCost.qFY * gradientYYError * dynamicsYError;
      double expectedYZ = 2.0 * SLIPModelTrackingCost.qFZ * gradientYZError * dynamicsZError;
      double expectedYTauX = 2.0 * SLIPModelTrackingCost.qTauX * gradientYTauXError * dynamicsTauXError;
      double expectedYTauY = 2.0 * SLIPModelTrackingCost.qTauY * gradientYTauYError * dynamicsTauYError;
      double expectedYTauZ = 2.0 * SLIPModelTrackingCost.qTauZ * gradientYTauZError * dynamicsTauZError;
      double expectedY = expectedYX + expectedYY + expectedYZ + expectedYTauX + expectedYTauY + expectedYTauZ;

      double gradientZXError = k_k * nominalLength * Math.pow(length, -3.0) * relativeX * z_k;
      double gradientZYError = k_k * nominalLength * Math.pow(length, -3.0) * relativeY * z_k;
      double gradientZZError = k_k * nominalLength * Math.pow(length, -3.0) * z_k * z_k - k_k * nominalLength / length + k_k;
      double gradientZTauXError = fy_k;
      double gradientZTauYError = -fx_k;
      double gradientZTauZError = 0.0;

      double expectedZX = 2.0 * SLIPModelTrackingCost.qFX * gradientZXError * dynamicsXError;
      double expectedZY = 2.0 * SLIPModelTrackingCost.qFY * gradientZYError * dynamicsYError;
      double expectedZZ = 2.0 * SLIPModelTrackingCost.qFZ * gradientZZError * dynamicsZError;
      double expectedZTauX = 2.0 * SLIPModelTrackingCost.qTauX * gradientZTauXError * dynamicsTauXError;
      double expectedZTauY = 2.0 * SLIPModelTrackingCost.qTauY * gradientZTauYError * dynamicsTauYError;
      double expectedZTauZ = 2.0 * SLIPModelTrackingCost.qTauZ * gradientZTauZError * dynamicsTauZError;
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
      double nominalLength = 1.5;
      double gravityZ = 9.81;
      SLIPModelTrackingCost cost = new SLIPModelTrackingCost(mass, nominalLength, gravityZ);

      Random random = new Random(1738L);
      DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize, 1);
      DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);

      DenseMatrix64F expectedGradient = new DenseMatrix64F(controlVectorSize, 1);
      DenseMatrix64F gradient = new DenseMatrix64F(controlVectorSize, 1);

      cost.getCostControlGradient(SLIPState.STANCE, currentControl, currentState, gradient);

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

      double dynamicsXError = fx_k - k_k * (nominalLength / length - 1.0) * relativeX;
      double dynamicsYError = fy_k - k_k * (nominalLength / length - 1.0) * relativeY;
      double dynamicsZError = fz_k - k_k * (nominalLength / length - 1.0) * z_k;
      double dynamicsTauXError = tauX_k + z_k * fy_k - (y_k - yf_k) * fz_k;
      double dynamicsTauYError = tauY_k - z_k * fx_k - (xf_k - x_k) * fz_k;
      double dynamicsTauZError = tauZ_k - (yf_k - y_k) * fx_k - (x_k - xf_k) * fy_k;

      double gradientFxXError = 1.0;
      double gradientFxYError = 0.0;
      double gradientFxZError = 0.0;
      double gradientFxTauXError = 0.0;
      double gradientFxTauYError = -z_k;
      double gradientFxTauZError = (y_k - yf_k);

      double expectedFxX = 2.0 * SLIPModelTrackingCost.qFX * gradientFxXError * dynamicsXError;
      double expectedFxY = 2.0 * SLIPModelTrackingCost.qFY * gradientFxYError * dynamicsYError;
      double expectedFxZ = 2.0 * SLIPModelTrackingCost.qFZ * gradientFxZError * dynamicsZError;
      double expectedFxTauX = 2.0 * SLIPModelTrackingCost.qTauX * gradientFxTauXError * dynamicsTauXError;
      double expectedFxTauY = 2.0 * SLIPModelTrackingCost.qTauY * gradientFxTauYError * dynamicsTauYError;
      double expectedFxTauZ = 2.0 * SLIPModelTrackingCost.qTauZ * gradientFxTauZError * dynamicsTauZError;
      double expectedFx = expectedFxX + expectedFxY + expectedFxZ + expectedFxTauX + expectedFxTauY + expectedFxTauZ;

      double gradientFyXError = 0.0;
      double gradientFyYError = 1.0;
      double gradientFyZError = 0.0;
      double gradientFyTauXError = z_k;
      double gradientFyTauYError = 0.0;
      double gradientFyTauZError = -(x_k - xf_k);

      double expectedFyX = 2.0 * SLIPModelTrackingCost.qFX * gradientFyXError * dynamicsXError;
      double expectedFyY = 2.0 * SLIPModelTrackingCost.qFY * gradientFyYError * dynamicsYError;
      double expectedFyZ = 2.0 * SLIPModelTrackingCost.qFZ * gradientFyZError * dynamicsZError;
      double expectedFyTauX = 2.0 * SLIPModelTrackingCost.qTauX * gradientFyTauXError * dynamicsTauXError;
      double expectedFyTauY = 2.0 * SLIPModelTrackingCost.qTauY * gradientFyTauYError * dynamicsTauYError;
      double expectedFyTauZ = 2.0 * SLIPModelTrackingCost.qTauZ * gradientFyTauZError * dynamicsTauZError;
      double expectedFy = expectedFyX + expectedFyY + expectedFyZ + expectedFyTauX + expectedFyTauY + expectedFyTauZ;

      double gradientFzXError = 0.0;
      double gradientFzYError = 0.0;
      double gradientFzZError = 1.0;
      double gradientFzTauXError = -(y_k - yf_k);
      double gradientFzTauYError = -(xf_k - x_k);
      double gradientFzTauZError = 0.0;

      double expectedFzX = 2.0 * SLIPModelTrackingCost.qFX * gradientFzXError * dynamicsXError;
      double expectedFzY = 2.0 * SLIPModelTrackingCost.qFY * gradientFzYError * dynamicsYError;
      double expectedFzZ = 2.0 * SLIPModelTrackingCost.qFZ * gradientFzZError * dynamicsZError;
      double expectedFzTauX = 2.0 * SLIPModelTrackingCost.qTauX * gradientFzTauXError * dynamicsTauXError;
      double expectedFzTauY = 2.0 * SLIPModelTrackingCost.qTauY * gradientFzTauYError * dynamicsTauYError;
      double expectedFzTauZ = 2.0 * SLIPModelTrackingCost.qTauZ * gradientFzTauZError * dynamicsTauZError;
      double expectedFz = expectedFzX + expectedFzY + expectedFzZ + expectedFzTauX + expectedFzTauY + expectedFzTauZ;

      double expectedTauX = 2.0 * SLIPModelTrackingCost.qTauX * dynamicsTauXError;
      double expectedTauY = 2.0 * SLIPModelTrackingCost.qTauY * dynamicsTauYError;
      double expectedTauZ = 2.0 * SLIPModelTrackingCost.qTauZ * dynamicsTauZError;

      // TODO xf, yf, k
      double gradientXfXError = -k_k * (relativeX * relativeX * nominalLength * Math.pow(length, -3.0) - nominalLength / length + 1.0);
      double gradientXfYError = -k_k * nominalLength * Math.pow(length, -3.0) * relativeX * relativeY;
      double gradientXfZError = -k_k * nominalLength * Math.pow(length, -3.0) * relativeX * z_k;
      double gradientXfTauXError = 0.0;
      double gradientXfTauYError = -fz_k;
      double gradientXfTauZError = fy_k;

      double expectedXfX = 2.0 * SLIPModelTrackingCost.qFX * gradientXfXError * dynamicsXError;
      double expectedXfY = 2.0 * SLIPModelTrackingCost.qFY * gradientXfYError * dynamicsYError;
      double expectedXfZ = 2.0 * SLIPModelTrackingCost.qFZ * gradientXfZError * dynamicsZError;
      double expectedXfTauX = 2.0 * SLIPModelTrackingCost.qTauX * gradientXfTauXError * dynamicsTauXError;
      double expectedXfTauY = 2.0 * SLIPModelTrackingCost.qTauY * gradientXfTauYError * dynamicsTauYError;
      double expectedXfTauZ = 2.0 * SLIPModelTrackingCost.qTauZ * gradientXfTauZError * dynamicsTauZError;
      double expectedXf = expectedXfX + expectedXfY + expectedXfZ + expectedXfTauX + expectedXfTauY + expectedXfTauZ;

      double gradientYfXError = -k_k * nominalLength * Math.pow(length, -3.0) * relativeX * relativeY;
      double gradientYfYError = -k_k * (nominalLength * Math.pow(length, -3.0) * relativeY * relativeY - nominalLength / length + 1.0);
      double gradientYfZError = -k_k * nominalLength * Math.pow(length, -3.0) * relativeY * z_k;
      double gradientYfTauXError = fz_k;
      double gradientYfTauYError = 0.0;
      double gradientYfTauZError = -fx_k;

      double expectedYfX = 2.0 * SLIPModelTrackingCost.qFX * gradientYfXError * dynamicsXError;
      double expectedYfY = 2.0 * SLIPModelTrackingCost.qFY * gradientYfYError * dynamicsYError;
      double expectedYfZ = 2.0 * SLIPModelTrackingCost.qFZ * gradientYfZError * dynamicsZError;
      double expectedYfTauX = 2.0 * SLIPModelTrackingCost.qTauX * gradientYfTauXError * dynamicsTauXError;
      double expectedYfTauY = 2.0 * SLIPModelTrackingCost.qTauY * gradientYfTauYError * dynamicsTauYError;
      double expectedYfTauZ = 2.0 * SLIPModelTrackingCost.qTauZ * gradientYfTauZError * dynamicsTauZError;
      double expectedYf = expectedYfX + expectedYfY + expectedYfZ + expectedYfTauX + expectedYfTauY + expectedYfTauZ;

      double gradientKXError = -(nominalLength / length - 1.0) * relativeX;
      double gradientKYError = -(nominalLength / length - 1.0) * relativeY;
      double gradientKZError = -(nominalLength / length - 1.0) * z_k;
      double gradientKTauXError = 0;
      double gradientKTauYError = 0;
      double gradientKTauZError = 0;

      double expectedKX = 2.0 * SLIPModelTrackingCost.qFX * gradientKXError * dynamicsXError;
      double expectedKY = 2.0 * SLIPModelTrackingCost.qFY * gradientKYError * dynamicsYError;
      double expectedKZ = 2.0 * SLIPModelTrackingCost.qFZ * gradientKZError * dynamicsZError;
      double expectedKTauX = 2.0 * SLIPModelTrackingCost.qTauX * gradientKTauXError * dynamicsTauXError;
      double expectedKTauY = 2.0 * SLIPModelTrackingCost.qTauY * gradientKTauYError * dynamicsTauYError;
      double expectedKTauZ = 2.0 * SLIPModelTrackingCost.qTauZ * gradientKTauZError * dynamicsTauZError;
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

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testCostStateHessianNumericalDifferentiationStance()
   {
      double epsilon = 1e-9;
      double mass = 15.0;
      double nominalLength = 1.5;
      double gravityZ = 9.81;
      SLIPModelTrackingCost costFunction = new SLIPModelTrackingCost(mass, nominalLength, gravityZ);

      Random random = new Random(1738L);
      DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize, 1);
      DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);

      DenseMatrix64F costHessian = new DenseMatrix64F(stateVectorSize, stateVectorSize);
      DenseMatrix64F expectedCostHessian = new DenseMatrix64F(stateVectorSize, stateVectorSize);

      DenseMatrix64F currentGradient = new DenseMatrix64F(stateVectorSize, 1);
      DenseMatrix64F modifiedGradient = new DenseMatrix64F(stateVectorSize, 1);

      costFunction.getCostStateHessian(SLIPState.STANCE, currentControl, currentState, costHessian);
      costFunction.getCostStateGradient(SLIPState.STANCE, currentControl, currentState, currentGradient);


      for (int partialState = 0; partialState < stateVectorSize; partialState++)
      {
         DenseMatrix64F currentStateModified = new DenseMatrix64F(currentState);
         currentStateModified.add(partialState, 0, epsilon);
         costFunction.getCostStateGradient(SLIPState.STANCE, currentControl, currentStateModified, modifiedGradient);

         for (int state = 0; state < stateVectorSize; state++)
            expectedCostHessian.set(state, partialState, (modifiedGradient.get(state) - currentGradient.get(state)) / epsilon);
      }


      JUnitTools.assertMatrixEquals(expectedCostHessian, costHessian, 1e-2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testCostControlHessianNumericalDifferentiationStance()
   {
      double epsilon = 1e-9;
      double mass = 15.0;
      double nominalLength = 1.5;
      double gravityZ = 9.81;
      SLIPModelTrackingCost costFunction = new SLIPModelTrackingCost(mass, nominalLength, gravityZ);

      Random random = new Random(1738L);
      DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize, 1);
      DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);

      DenseMatrix64F costHessian = new DenseMatrix64F(controlVectorSize, controlVectorSize);
      DenseMatrix64F expectedCostHessian = new DenseMatrix64F(controlVectorSize, controlVectorSize);

      DenseMatrix64F currentGradient = new DenseMatrix64F(controlVectorSize, 1);
      DenseMatrix64F modifiedGradient = new DenseMatrix64F(controlVectorSize, 1);

      costFunction.getCostControlHessian(SLIPState.STANCE, currentControl, currentState, costHessian);
      costFunction.getCostControlGradient(SLIPState.STANCE, currentControl, currentState, currentGradient);


      for (int partialControl = 0; partialControl < controlVectorSize; partialControl++)
      {
         DenseMatrix64F currentControlModified = new DenseMatrix64F(currentControl);
         currentControlModified.add(partialControl, 0, epsilon);
         costFunction.getCostControlGradient(SLIPState.STANCE, currentControlModified, currentState, modifiedGradient);

         for (int control = 0; control < controlVectorSize; control++)
            expectedCostHessian.set(control, partialControl, (modifiedGradient.get(control) - currentGradient.get(control)) / epsilon);
      }


      JUnitTools.assertMatrixEquals(expectedCostHessian, costHessian, 1e-2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testCostStateControlHessianNumericalDifferentiationStance()
   {
      double epsilon = 1e-9;
      double mass = 15.0;
      double nominalLength = 1.5;
      double gravityZ = 9.81;
      SLIPModelTrackingCost costFunction = new SLIPModelTrackingCost(mass, nominalLength, gravityZ);

      Random random = new Random(1738L);
      DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize, 1);
      DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);

      DenseMatrix64F costHessian = new DenseMatrix64F(stateVectorSize, controlVectorSize);
      DenseMatrix64F expectedCostHessian = new DenseMatrix64F(stateVectorSize, controlVectorSize);

      DenseMatrix64F currentGradient = new DenseMatrix64F(stateVectorSize, 1);
      DenseMatrix64F modifiedGradient = new DenseMatrix64F(stateVectorSize, 1);

      costFunction.getCostControlGradientOfStateGradient(SLIPState.STANCE, currentControl, currentState, costHessian);
      costFunction.getCostStateGradient(SLIPState.STANCE, currentControl, currentState, currentGradient);


      for (int partialControl = 0; partialControl < controlVectorSize; partialControl++)
      {
         DenseMatrix64F currentControlModified = new DenseMatrix64F(currentControl);
         currentControlModified.add(partialControl, 0, epsilon);
         costFunction.getCostStateGradient(SLIPState.STANCE, currentControlModified, currentState, modifiedGradient);

         for (int state = 0; state < stateVectorSize; state++)
            expectedCostHessian.set(state, partialControl, (modifiedGradient.get(state) - currentGradient.get(state)) / epsilon);
      }


      JUnitTools.assertMatrixEquals(expectedCostHessian, costHessian, 1e-2);
   }
}
