package us.ihmc.robotics.math;

import static us.ihmc.robotics.Assert.*;

import java.util.Random;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoRMSCalculatorTest
{
   private Random random;

   @BeforeEach
   public void setUp()
   {
      random = new Random(1779L);
   }

	@Test
   public void testAgainstDefinition()
   {
      int nValues = 100;
      double[] values = getValues(nValues, random);

      YoRegistry registry = new YoRegistry("registry");
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
