package us.ihmc.commonWalkingControlModules.dynamicPlanning;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.testing.JUnitTools;

public class LIPMDDPCalculatorTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testComputeNewControlAndNextState()
   {
      LIPMDDPCalculator calculator = new LIPMDDPCalculator(0.01, 10.0, 9.81);

      DenseMatrix64F feedforwardTerm = new DenseMatrix64F(3, 1);

      DenseMatrix64F currentState = new DenseMatrix64F(6, 1);
      DenseMatrix64F currentControl = new DenseMatrix64F(3, 1);

      DenseMatrix64F updatedState = new DenseMatrix64F(6, 1);
      DenseMatrix64F updatedControl = new DenseMatrix64F(3, 1);

      DenseMatrix64F updatedControlExpected = new DenseMatrix64F(3, 1);


      DenseMatrix64F gainMatrix = new DenseMatrix64F(3, 6);

      double xyPositionGain = 2.0;
      double zPositionGain = 5.0;
      double xyDotPositionGain = 5.0;
      double zDotPositionGain = 8.0;

      double currentX = 0.7;
      double currentY = 0.25;
      double currentZ = 1.0;
      double currentXDot = 1.5;
      double currentYDot = 0.5;
      double currentZDot = 0.5;

      double updatedX = 1.0;
      double updatedY = 0.2;
      double updatedZ = 1.1;
      double updatedXDot = 1.2;
      double updatedYDot = 0.3;
      double updatedZDot = 0.1;

      double pX = 0.5;
      double pY = 0.26;
      double fZ = 100.0;

      double feedforwardX = 0.4;
      double feedforwardY = 0.2;
      double feedforwardZ = 30.0;

      currentState.set(0, currentX);
      currentState.set(1, currentY);
      currentState.set(2, currentZ);
      currentState.set(3, currentXDot);
      currentState.set(4, currentYDot);
      currentState.set(5, currentZDot);

      updatedState.set(0, updatedX);
      updatedState.set(1, updatedY);
      updatedState.set(2, updatedZ);
      updatedState.set(3, updatedXDot);
      updatedState.set(4, updatedYDot);
      updatedState.set(5, updatedZDot);

      gainMatrix.set(0, 0, xyPositionGain);
      gainMatrix.set(0, 3, xyDotPositionGain);
      gainMatrix.set(1, 1, xyPositionGain);
      gainMatrix.set(1, 4, xyDotPositionGain);
      gainMatrix.set(2, 2, zPositionGain);
      gainMatrix.set(2, 5, zDotPositionGain);

      currentControl.set(0, pX);
      currentControl.set(1, pY);
      currentControl.set(2, fZ);

      feedforwardTerm.set(0, feedforwardX);
      feedforwardTerm.set(1, feedforwardY);
      feedforwardTerm.set(2, feedforwardZ);

      double updatedPx = pX - feedforwardX + xyPositionGain * (currentX - updatedX) + xyDotPositionGain * (currentXDot - updatedXDot);
      double updatedPy = pY - feedforwardY + xyPositionGain * (currentY - updatedY) + xyDotPositionGain * (currentYDot - updatedYDot);
      double updatedFz = fZ - feedforwardZ + zPositionGain * (currentZ - updatedZ) + zDotPositionGain * (currentZDot - updatedZDot);

      updatedControlExpected.set(0, updatedPx);
      updatedControlExpected.set(1, updatedPy);
      updatedControlExpected.set(2, updatedFz);

      calculator.computeUpdatedControl(currentState, updatedState, gainMatrix, feedforwardTerm, currentControl, updatedControl);

      DenseMatrix64F stateError = new DenseMatrix64F(6, 1);
      DenseMatrix64F updatedControlAlternative = new DenseMatrix64F(3, 1);

      CommonOps.subtract(currentState, updatedState, stateError);
      CommonOps.subtract(currentControl, feedforwardTerm, updatedControlAlternative);
      CommonOps.multAdd(gainMatrix, stateError, updatedControlAlternative);

      JUnitTools.assertMatrixEquals(updatedControlAlternative, updatedControl, 1e-12);
      JUnitTools.assertMatrixEquals(updatedControlExpected, updatedControl, 1e-12);
   }
}
