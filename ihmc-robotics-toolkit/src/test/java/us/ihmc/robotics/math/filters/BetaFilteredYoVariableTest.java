package us.ihmc.robotics.math.filters;

import static us.ihmc.robotics.Assert.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class BetaFilteredYoVariableTest
{
   private final Random rng = new Random();

	@Test
   public void testBetaFilteredYoVariable()
   {
      int beta = 5000;
      double pseudoNoise = 0;

      YoRegistry registry = new YoRegistry("testRegistry");
      YoDouble positionVariable = new YoDouble("positionVariable", registry);
      BetaFilteredYoVariable betaFilteredYoVariable = new BetaFilteredYoVariable("betaFilteredYoVariable", registry, beta, positionVariable);

      positionVariable.set(10);

      for (int i = 0; i < 10000; i++)
      {
         if (i % 2 == 0)
         {
            pseudoNoise = rng.nextDouble();
         }
         positionVariable.add(Math.pow(-1, i) * pseudoNoise);
         betaFilteredYoVariable.update();
      }

      assertEquals(10, betaFilteredYoVariable.getDoubleValue(), 1);
   }

}
