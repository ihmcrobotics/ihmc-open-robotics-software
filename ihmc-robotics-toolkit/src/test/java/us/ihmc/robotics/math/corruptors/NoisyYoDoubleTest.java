package us.ihmc.robotics.math.corruptors;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class NoisyYoDoubleTest
{
   YoRegistry yoVariableRegistry = new YoRegistry("testRegistry");

	@Test
   public void testSimpleConstructor()
   {
      NoisyYoDouble noisyDoubleYoVariable = new NoisyYoDouble("testN", yoVariableRegistry);
      noisyDoubleYoVariable.update();
   }

	@Test
   public void testSimpleConstructorDouble()
   {
      YoDouble yoDouble = new YoDouble("testD", yoVariableRegistry);
      NoisyYoDouble noisyDoubleYoVariable = new NoisyYoDouble("testN", yoVariableRegistry, yoDouble);
      noisyDoubleYoVariable.update();
   }

	@Test
   public void testFullConstructor()
   {
      NoisyYoDouble noisyDoubleYoVariable = new NoisyYoDouble("testN", yoVariableRegistry, false, 2.0, true, 1.0, 2.0, 0.0, 0.01,
                                                       NoiseType.UNIFORM, 1.0);
      noisyDoubleYoVariable.update();
   }

	@Test
   public void testFullConstructorDouble()
   {
      YoDouble yoDouble = new YoDouble("testD", yoVariableRegistry);
      NoisyYoDouble noisyDoubleYoVariable = new NoisyYoDouble("testN", yoVariableRegistry, yoDouble, false, 2.0, true, 1.0, 2.0, 0.0,
                                                       0.01, NoiseType.UNIFORM, 1.0);
      noisyDoubleYoVariable.update();
   }

// @Test
// public void testUpdateException()
// {
//  NoisyYoDouble noisyDoubleYoVariable = new NoisyYoDouble("test", yoVariableRegistry);
//
//  try
//  {
//     noisyDoubleYoVariable.update();
//     fail("Should have gotten a NullPointerExeption. Updated without a perfect value.");
//  }
//  catch (NullPointerException expected)
//  {
//     ;    // Expected - intentional
//  }
// }

	@Test
   public void testGetPerfectDoubleValue()
   {
      NoisyYoDouble noisyDoubleYoVariable = new NoisyYoDouble("testN", yoVariableRegistry);

      noisyDoubleYoVariable.update();
      assertTrue(noisyDoubleYoVariable.getPerfectDoubleValue() == 0.0);

      noisyDoubleYoVariable.update(2.0);
      assertTrue(noisyDoubleYoVariable.getPerfectDoubleValue() == 2.0);
   }

	@Test
   public void testGetPerfectDoubleValueDouble()
   {
      YoDouble yoDouble = new YoDouble("testD", yoVariableRegistry);
      NoisyYoDouble noisyDoubleYoVariable = new NoisyYoDouble("testN", yoVariableRegistry, yoDouble);

      yoDouble.set(2.0);
      assertTrue(noisyDoubleYoVariable.getPerfectDoubleValue() == 2.0);

      noisyDoubleYoVariable.update(3.0);
      assertTrue(noisyDoubleYoVariable.getPerfectDoubleValue() == 3.0);
   }

	@Test
   public void testDiscrete()
   {
      NoisyYoDouble noisyDoubleYoVariable = new NoisyYoDouble("testN", yoVariableRegistry);
      noisyDoubleYoVariable.setIsNoisy(true);
      noisyDoubleYoVariable.setGaussianNoise(1.0);
      noisyDoubleYoVariable.update(10.0);
      double noisy = noisyDoubleYoVariable.getDoubleValue();
      double noisy2 = noisyDoubleYoVariable.getDoubleValue();
      noisyDoubleYoVariable.update();
      double noisy3 = noisyDoubleYoVariable.getDoubleValue();

      assertTrue(noisy == noisy2);
      assertFalse(noisy == noisy3);
   }

	@Test
   public void testRandomBound()
   {
      NoisyYoDouble noisyDoubleYoVariable = new NoisyYoDouble("testN", yoVariableRegistry);

      int count = 10000;
      double randomBound = 10.0 * Math.random();
      double value = 1000.0 * Math.random();

      noisyDoubleYoVariable.setIsNoisy(true);
      noisyDoubleYoVariable.setRandomBound(randomBound);
      noisyDoubleYoVariable.setBias(false);
      noisyDoubleYoVariable.update(value);

      double[] values = new double[count];
      for (int j = 0; j < count; j++)
      {
         noisyDoubleYoVariable.update();
         values[j] = noisyDoubleYoVariable.getDoubleValue();
         assertTrue(values[j] >= value - randomBound);
         assertTrue(values[j] <= value + randomBound);
      }
   }

	@Test
   public void testZeroRandomBound()
   {
      NoisyYoDouble noisyDoubleYoVariable = new NoisyYoDouble("testN", yoVariableRegistry);

      int count = 10000;
      double randomBound = 0.0;
      double value = 1000.0 * Math.random();

      noisyDoubleYoVariable.setIsNoisy(true);
      noisyDoubleYoVariable.setRandomBound(randomBound);
      noisyDoubleYoVariable.setBias(false);
      noisyDoubleYoVariable.update(value);

      for (int j = 0; j < count; j++)
      {
         noisyDoubleYoVariable.update();
         assertEquals(noisyDoubleYoVariable.getDoubleValue(), value, 0.0);
      }
   }

	@Test
   public void testZeroRandomBoundWithPerfect()
   {
      NoisyYoDouble noisyDoubleYoVariable = new NoisyYoDouble("testN", yoVariableRegistry);

      int count = 10000;
      double randomBound = 0.0;
      double value = 1000.0 * Math.random();

      noisyDoubleYoVariable.setIsNoisy(true);
      noisyDoubleYoVariable.setRandomBound(randomBound);
      noisyDoubleYoVariable.setBias(false);
      noisyDoubleYoVariable.update(value);

      for (int j = 0; j < count; j++)
      {
         noisyDoubleYoVariable.update();
         assertEquals(noisyDoubleYoVariable.getDoubleValue(), noisyDoubleYoVariable.getPerfectDoubleValue(), 0.0);
      }
   }

	@Test
   public void testStaticBias()
   {
      NoisyYoDouble noisyDoubleYoVariable = new NoisyYoDouble("testN", yoVariableRegistry);

      double bias = 10.0 * Math.random();
      double value = 1000.0 * Math.random();

      noisyDoubleYoVariable.setIsNoisy(true);
      noisyDoubleYoVariable.setRandomBound(0.0);
      noisyDoubleYoVariable.setBias(bias);

      noisyDoubleYoVariable.update(value);

      assertTrue(noisyDoubleYoVariable.getDoubleValue() == value + bias);
   }

	@Test
   public void testWalkingBias()
   {
      NoisyYoDouble noisyDoubleYoVariable = new NoisyYoDouble("testN", yoVariableRegistry);

      int count = 10000;
      double bias = 10.0 * Math.random();
      double biasMax = bias + 3.0;
      double biasMin = bias - 3.0;
      double biasDelta = bias / 10.0;
      double value = 1000.0 * Math.random();

      noisyDoubleYoVariable.setIsNoisy(true);
      noisyDoubleYoVariable.setRandomBound(0.0);
      noisyDoubleYoVariable.setBias(bias, biasMax, biasMin, biasDelta);
      noisyDoubleYoVariable.update(value);

      double[] values = new double[count];

      for (int j = 0; j < count; j++)
      {
         noisyDoubleYoVariable.update();
         values[j] = noisyDoubleYoVariable.getDoubleValue();
         assertTrue(values[j] <= value + biasMax);
         assertTrue(values[j] >= value + biasMin);
         if (j > 0)
            assertTrue(Math.abs(values[j] - values[j - 1]) <= biasDelta);
      }
   }

	@Test
   public void testRandomBoundAndStaticBias()
   {
      NoisyYoDouble noisyDoubleYoVariable = new NoisyYoDouble("testN", yoVariableRegistry);

      int count = 10000;
      double bias = 10.0 * Math.random();
      double randomBound = 5.0 * Math.random();
      double value = 1000.0 * Math.random();

      noisyDoubleYoVariable.setIsNoisy(true);
      noisyDoubleYoVariable.setRandomBound(randomBound);
      noisyDoubleYoVariable.setBias(bias);
      noisyDoubleYoVariable.update(value);

      double[] values = new double[count];
      for (int j = 0; j < count; j++)
      {
         noisyDoubleYoVariable.update();
         values[j] = noisyDoubleYoVariable.getDoubleValue();
         assertTrue(values[j] >= value - randomBound + bias);
         assertTrue(values[j] <= value + randomBound + bias);
      }
   }

	@Test
   public void testRandomBoundAndWalkingBias()
   {
      NoisyYoDouble noisyDoubleYoVariable = new NoisyYoDouble("testN", yoVariableRegistry);

      int count = 10000;
      double bias = 10.0 * Math.random();
      double biasMax = bias + 3.0;
      double biasMin = bias - 3.0;
      double biasDelta = bias / 10.0;
      double randomBound = 5.0 * Math.random();
      double value = 1000.0 * Math.random();

      noisyDoubleYoVariable.setIsNoisy(true);
      noisyDoubleYoVariable.setRandomBound(randomBound);
      noisyDoubleYoVariable.setBias(bias, biasMax, biasMin, biasDelta);
      noisyDoubleYoVariable.update(value);

      double[] values = new double[count];
      for (int j = 0; j < count; j++)
      {
         noisyDoubleYoVariable.update();
         values[j] = noisyDoubleYoVariable.getDoubleValue();
         assertTrue(values[j] >= value - randomBound + biasMin);
         assertTrue(values[j] <= value + randomBound + biasMax);
      }
   }

	@Test
   public void testGaussianNoise()
   {
      NoisyYoDouble noisyDoubleYoVariable = new NoisyYoDouble("testN", yoVariableRegistry);

      int count = 10000;
      double standardDeviation = 10.0 * Math.random();
      double value = 1000.0 * Math.random();

      noisyDoubleYoVariable.setIsNoisy(true);
      noisyDoubleYoVariable.setBias(false);
      noisyDoubleYoVariable.setGaussianNoise(standardDeviation);
      noisyDoubleYoVariable.update(value);

      double[] values = new double[count];
      for (int j = 0; j < count; j++)
      {
         noisyDoubleYoVariable.update();
         values[j] = noisyDoubleYoVariable.getDoubleValue();
      }

      double delta = 0.05 * standardDeviation;
      assertEquals(standardDeviation, standardDeviation(values), delta);
   }

	@Test
   public void testGaussianNoiseAndStaticBias()
   {
      NoisyYoDouble noisyDoubleYoVariable = new NoisyYoDouble("testN", yoVariableRegistry);

      int count = 10000;
      double bias = 10.0 * Math.random();
      double standardDeviation = 5.0 * Math.random();
      double value = 1000.0 * Math.random();

      noisyDoubleYoVariable.setIsNoisy(true);
      noisyDoubleYoVariable.setBias(bias);
      noisyDoubleYoVariable.setGaussianNoise(standardDeviation);
      noisyDoubleYoVariable.update(value);

      double[] values = new double[count];
      for (int j = 0; j < count; j++)
      {
         noisyDoubleYoVariable.update();
         values[j] = noisyDoubleYoVariable.getDoubleValue();
      }

      double delta = 0.05 * standardDeviation;
      assertEquals(standardDeviation, standardDeviation(values), delta);
   }


   /*
    * Private functions.
    */
   private static double standardDeviation(double[] values)
   {
      double mean, standardDeviation;
      double[] squareDiffs = new double[values.length];

      mean = sum(values) / values.length;

      for (int j = 0; j < values.length; j++)
      {
         squareDiffs[j] = Math.pow(values[j] - mean, 2);
      }

      standardDeviation = Math.sqrt(sum(squareDiffs) / (values.length - 1));

      return standardDeviation;
   }

   private static double sum(double[] values)
   {
      double sum = 0.0;

      for (int j = 0; j < values.length; j++)
      {
         sum += values[j];
      }

      return sum;
   }
}
