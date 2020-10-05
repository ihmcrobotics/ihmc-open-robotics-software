package us.ihmc.robotics.math.filters;

import static us.ihmc.robotics.Assert.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class AlphaFusedYoVariableTest
{

   private final Random rng = new Random();

	@Test
   public void testAlphaFusedYoVariable()
   {
      double alpha = 0.3, slowSignalNoise = 0, fastSignalNoise = 0;

      YoRegistry registry = new YoRegistry("testRegistry");
      YoDouble slowSignal = new YoDouble("slowFrequency", registry);
      YoDouble fastSignal = new YoDouble("fastFrequency", registry);

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
