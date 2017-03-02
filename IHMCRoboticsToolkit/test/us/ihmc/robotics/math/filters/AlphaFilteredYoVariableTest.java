package us.ihmc.robotics.math.filters;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class AlphaFilteredYoVariableTest
{
   private final Random rng = new Random();

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testNoisyFixedPosition()
   {
      // Use a reasonably large alpha for a reasonably large amount of noise
      double alpha = 0.8;

      YoVariableRegistry registry = new YoVariableRegistry("testRegistry");
      DoubleYoVariable positionVariable = new DoubleYoVariable("positionVariable", registry);
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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testAlphaAndBreakFrequencyComputations()
   {
      double DT = 0.1;
      double randomAlpha = rng.nextDouble();
      double computedBreakFrequency = AlphaFilteredYoVariable.computeBreakFrequencyGivenAlpha(randomAlpha, DT);
      double computedAlpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(computedBreakFrequency, DT);

      assertEquals(randomAlpha, computedAlpha, 1e-7);
      assertEquals(computedBreakFrequency, AlphaFilteredYoVariable.computeBreakFrequencyGivenAlpha(computedAlpha, DT), 1e-7);

      System.out.println("Random Alpha: " + randomAlpha);
      System.out.println("Computed Alpha: " + AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(computedBreakFrequency, DT));

   }
}
