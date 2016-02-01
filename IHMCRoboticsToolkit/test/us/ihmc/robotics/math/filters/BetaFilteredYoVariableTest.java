package us.ihmc.robotics.math.filters;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.junit.Test;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.BetaFilteredYoVariable;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class BetaFilteredYoVariableTest
{
   private final Random rng = new Random();

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testBetaFilteredYoVariable()
   {
      int beta = 5000;
      double pseudoNoise = 0;

      YoVariableRegistry registry = new YoVariableRegistry("testRegistry");
      DoubleYoVariable positionVariable = new DoubleYoVariable("positionVariable", registry);
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
