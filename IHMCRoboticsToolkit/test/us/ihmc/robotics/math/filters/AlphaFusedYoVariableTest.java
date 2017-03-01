package us.ihmc.robotics.math.filters;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class AlphaFusedYoVariableTest
{

   private final Random rng = new Random();

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testAlphaFusedYoVariable()
   {
      double alpha = 0.3, slowSignalNoise = 0, fastSignalNoise = 0;

      YoVariableRegistry registry = new YoVariableRegistry("testRegistry");
      DoubleYoVariable slowSignal = new DoubleYoVariable("slowFrequency", registry);
      DoubleYoVariable fastSignal = new DoubleYoVariable("fastFrequency", registry);

      slowSignal.set(10);
      fastSignal.set(10);

      AlphaFusedYoVariable alphaFusedVariable = new AlphaFusedYoVariable("alphaFusedVariable", registry, alpha, slowSignal, fastSignal);

      for (int i = 0; i < 10000; i++)
      {
         if (i % 2 == 0)
         {
            fastSignalNoise = rng.nextDouble();
         }
         fastSignal.add(Math.pow(-1, i) * fastSignalNoise);

         if (i % 50 == 0)
         {
            slowSignalNoise = rng.nextDouble();
            slowSignal.add(slowSignalNoise);
         }

         if ((i - 1) % 50 == 0)
         {
            slowSignal.add(-slowSignalNoise);
         }

         alphaFusedVariable.update();
      }

      //      System.out.println("Fused variable: " + alphaFusedVariable.getDoubleValue());
      assertEquals(10, alphaFusedVariable.getDoubleValue(), 1);
   }

}
