package us.ihmc.robotics.math;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class YoRMSCalculatorTest
{
   private Random random;

   @Before
   public void setUp()
   {
      random = new Random(1779L);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testAgainstDefinition()
   {
      int nValues = 100;
      double[] values = getValues(nValues, random);

      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoRMSCalculator calculator = new YoRMSCalculator("testCalculator", registry);
      for (int i = 0; i < nValues; i++)
      {
         calculator.update(values[i]);
      }

      double epsilon = 1e-8;
      assertEquals(computeRMS(values), calculator.val(), epsilon);
   }

   private double computeRMS(double[] values)
   {
      double sumOfSquares = 0.0;
      int n = values.length;
      for (int i = 0; i < n; i++)
      {
         sumOfSquares += values[i] * values[i];
      }
      double ret = Math.sqrt(sumOfSquares / n);
      return ret;
   }

   private double[] getValues(int nValues, Random random)
   {
      double[] ret = new double[nValues];
      for (int i = 0; i < nValues; i++)
      {
         ret[i] = random.nextDouble();
      }
      return ret;
   }
}
