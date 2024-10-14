package us.ihmc.robotics.math.filters;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import static org.junit.jupiter.api.Assertions.*;

public class DeadbandedYoVariableTest
{

   @Test
   public void testDeadband()
   {
      YoRegistry registry = new YoRegistry("test");
      YoDouble deadzoneSize = new YoDouble("deadzoneSize", registry);
      YoDouble input = new YoDouble("input", registry);
      double deadzone = 2.0;
      deadzoneSize.set(deadzone);
      DeadbandedYoVariable testDeadzone = new DeadbandedYoVariable("testDeadZone", input , deadzoneSize, registry);

      double verySmallStep = 1e-4;
      
      input.set(deadzone - verySmallStep);
      testDeadzone.update();
      assertTrue(MathTools.epsilonEquals(testDeadzone.getDoubleValue(), 0.0, 1e-14));
      input.set(deadzone + verySmallStep);
      testDeadzone.update();
      assertTrue(MathTools.epsilonEquals(testDeadzone.getDoubleValue(), verySmallStep, 1e-14));

      input.set(-deadzone + verySmallStep);
      testDeadzone.update();
      assertTrue(MathTools.epsilonEquals(testDeadzone.getDoubleValue(), 0.0, 1e-14));
      input.set(-deadzone - verySmallStep);
      testDeadzone.update();
      assertTrue(MathTools.epsilonEquals(testDeadzone.getDoubleValue(), -verySmallStep, 1e-14));

      for (double valueToBeCorrected = -10.0; valueToBeCorrected < -deadzone; valueToBeCorrected += 0.01)
      {
         input.set(valueToBeCorrected);
         testDeadzone.update();
         assertTrue(MathTools.epsilonEquals(testDeadzone.getDoubleValue(), valueToBeCorrected + deadzone, 1e-14));
      }
      for (double valueToBeCorrected = -deadzone; valueToBeCorrected < deadzone; valueToBeCorrected += 0.01)
      {
         input.set(valueToBeCorrected);
         testDeadzone.update();
         assertTrue(MathTools.epsilonEquals(testDeadzone.getDoubleValue(), 0.0, 1e-14));
      }
      for (double valueToBeCorrected = deadzone; valueToBeCorrected < 10.0; valueToBeCorrected += 0.01)
      {
         input.set(valueToBeCorrected);
         testDeadzone.update();
         assertTrue(MathTools.epsilonEquals(testDeadzone.getDoubleValue(), valueToBeCorrected - deadzone, 1e-14));
      }
   }
}
