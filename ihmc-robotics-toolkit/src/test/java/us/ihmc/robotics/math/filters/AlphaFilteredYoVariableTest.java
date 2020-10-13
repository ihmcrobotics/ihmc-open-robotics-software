package us.ihmc.robotics.math.filters;

import static us.ihmc.robotics.Assert.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class AlphaFilteredYoVariableTest
{
   private final Random rng = new Random();

	@Test
   public void testNoisyFixedPosition()
   {
      // Use a reasonably large alpha for a reasonably large amount of noise
      double alpha = 0.8;

      YoRegistry registry = new YoRegistry("testRegistry");
      YoDouble positionVariable = new YoDouble("positionVariable", registry);
      AlphaFilteredYoVariable alphaFilteredYoVariable = new AlphaFilteredYoVariable("alphaFilteredYoVariable", registry, alpha, positionVariable);

      double pseudoNoise = 0;

      positionVariable.set(10);
      for (int i = 0; i < 10000; i++)
      {
         // Oscillate the position about some uniformly distributed fixed point slightly larger than 10
         if (i % 2 == 0)
         {
            pseudoNoise = rng.nextDouble();
         }
         positionVariable.add(Math.pow(-1, i) * pseudoNoise);
         alphaFilteredYoVariable.update();
      }

      assertEquals(10, alphaFilteredYoVariable.getDoubleValue(), 1);
   }

	@Test
   public void testAlphaAndBreakFrequencyComputations()
   {
      for (int i = 0; i < 1000; i++)
      {
         double dt = rng.nextDouble();

         double expectedAlpha = rng.nextDouble();
         double breakFrequency = AlphaFilteredYoVariable.computeBreakFrequencyGivenAlpha(expectedAlpha, dt);
         double actualAlpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(breakFrequency, dt);

         assertEquals(expectedAlpha, actualAlpha, 1e-10);

         double maxFrequency = 0.5 * 0.5 / dt;
         double expectedBreakFrequency = maxFrequency * rng.nextDouble();
         double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(expectedBreakFrequency, dt);
         double actualBreakFrequency = AlphaFilteredYoVariable.computeBreakFrequencyGivenAlpha(alpha, dt);

         assertEquals(expectedBreakFrequency, actualBreakFrequency, 1e-7);
      }
   }
}
