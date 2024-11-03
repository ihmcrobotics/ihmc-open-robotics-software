package us.ihmc.robotics.math.filters;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class DelayedYoDoubleTest
{

	@Test
   public void testDelayedYoVariableMultipleTickDelays()
   {
      YoRegistry registry = new YoRegistry("registry");
      YoDouble variableToDelay = new YoDouble("variableToDelay", registry);

      for (int ticksToDelay = 0; ticksToDelay < 10; ticksToDelay++)
      {
         double firstValue = Math.random();
         variableToDelay.set(firstValue);

         DelayedYoDouble delayedYoVariable = new DelayedYoDouble("delayedVariable" + ticksToDelay, "", variableToDelay, ticksToDelay, registry);

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

	@Test
   public void testDelayedYoVariableOneTickDelay()
   {
      YoRegistry registry = new YoRegistry("registry");
      YoDouble variableToDelay = new YoDouble("variableToDelay", registry);

      int ticksToDelay = 1;

      variableToDelay.set(0.0);
      DelayedYoDouble delayedYoVariable = new DelayedYoDouble("delayedVariable" + ticksToDelay, "", variableToDelay, ticksToDelay, registry);
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

	@Test
   public void testDelayedYoVariableZeroTickDelay()
   {
      YoRegistry registry = new YoRegistry("registry");
      YoDouble variableToDelay = new YoDouble("variableToDelay", registry);

      int ticksToDelay = 0;

      variableToDelay.set(0.0);
      DelayedYoDouble delayedYoVariable = new DelayedYoDouble("delayedVariable" + ticksToDelay, "", variableToDelay, ticksToDelay, registry);
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
