package us.ihmc.robotics.math.filters;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class DelayedDoubleYoVariableTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testDelayedYoVariableMultipleTickDelays()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable variableToDelay = new DoubleYoVariable("variableToDelay", registry);

      for (int ticksToDelay = 0; ticksToDelay < 10; ticksToDelay++)
      {
         double firstValue = Math.random();
         variableToDelay.set(firstValue);

         DelayedDoubleYoVariable delayedYoVariable = new DelayedDoubleYoVariable("delayedVariable" + ticksToDelay, "", variableToDelay, ticksToDelay, registry);

         int ticksToTest = 100;
         double[] valuesToSet = new double[ticksToTest];

         for (int i = 0; i < valuesToSet.length; i++)
         {
            valuesToSet[i] = Math.random();
         }

         assertEquals(delayedYoVariable.getDoubleValue(), firstValue, 1e-7);

         for (int i = 0; i < ticksToTest; i++)
         {
            variableToDelay.set(valuesToSet[i]);
            delayedYoVariable.update();

            if (i < ticksToDelay)
            {
               assertEquals(delayedYoVariable.getDoubleValue(), firstValue, 1e-7);
            }
            else
            {
               assertEquals(delayedYoVariable.getDoubleValue(), valuesToSet[i - ticksToDelay], 1e-7);
            }
         }
      }

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testDelayedYoVariableOneTickDelay()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable variableToDelay = new DoubleYoVariable("variableToDelay", registry);

      int ticksToDelay = 1;

      variableToDelay.set(0.0);
      DelayedDoubleYoVariable delayedYoVariable = new DelayedDoubleYoVariable("delayedVariable" + ticksToDelay, "", variableToDelay, ticksToDelay, registry);
      assertEquals(delayedYoVariable.getDoubleValue(), 0.0, 1e-7);

      variableToDelay.set(1.0);
      delayedYoVariable.update();
      assertEquals(delayedYoVariable.getDoubleValue(), 0.0, 1e-7);

      variableToDelay.set(2.0);
      delayedYoVariable.update();
      assertEquals(delayedYoVariable.getDoubleValue(), 1.0, 1e-7);

      variableToDelay.set(3.0);
      delayedYoVariable.update();
      assertEquals(delayedYoVariable.getDoubleValue(), 2.0, 1e-7);

      delayedYoVariable.update();
      assertEquals(delayedYoVariable.getDoubleValue(), 3.0, 1e-7);

      delayedYoVariable.update();
      assertEquals(delayedYoVariable.getDoubleValue(), 3.0, 1e-7);

      delayedYoVariable.update();
      assertEquals(delayedYoVariable.getDoubleValue(), 3.0, 1e-7);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testDelayedYoVariableZeroTickDelay()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable variableToDelay = new DoubleYoVariable("variableToDelay", registry);

      int ticksToDelay = 0;

      variableToDelay.set(0.0);
      DelayedDoubleYoVariable delayedYoVariable = new DelayedDoubleYoVariable("delayedVariable" + ticksToDelay, "", variableToDelay, ticksToDelay, registry);
      assertEquals(delayedYoVariable.getDoubleValue(), 0.0, 1e-7);

      variableToDelay.set(1.0);
      delayedYoVariable.update();
      assertEquals(delayedYoVariable.getDoubleValue(), 1.0, 1e-7);

      variableToDelay.set(2.0);
      delayedYoVariable.update();
      assertEquals(delayedYoVariable.getDoubleValue(), 2.0, 1e-7);

      variableToDelay.set(3.0);
      delayedYoVariable.update();
      assertEquals(delayedYoVariable.getDoubleValue(), 3.0, 1e-7);

      delayedYoVariable.update();
      assertEquals(delayedYoVariable.getDoubleValue(), 3.0, 1e-7);

      delayedYoVariable.update();
      assertEquals(delayedYoVariable.getDoubleValue(), 3.0, 1e-7);

      delayedYoVariable.update();
      assertEquals(delayedYoVariable.getDoubleValue(), 3.0, 1e-7);
   }

}
