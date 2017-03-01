package us.ihmc.robotics.math.filters;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

@ContinuousIntegrationPlan(categories = { IntegrationCategory.FAST })
public class DeadzoneYoVariableTest
{

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDeadzone()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      DoubleYoVariable deadzoneSize = new DoubleYoVariable("deadzoneSize", registry);
      DoubleYoVariable input = new DoubleYoVariable("input", registry);
      double deadzone = 2.0;
      deadzoneSize.set(deadzone);
      DeadzoneYoVariable testDeadzone = new DeadzoneYoVariable("testDeadZone", input , deadzoneSize, registry);

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
