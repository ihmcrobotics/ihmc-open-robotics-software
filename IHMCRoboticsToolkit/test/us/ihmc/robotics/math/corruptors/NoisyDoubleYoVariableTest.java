package us.ihmc.robotics.math.corruptors;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class NoisyDoubleYoVariableTest
{
   YoVariableRegistry yoVariableRegistry = new YoVariableRegistry("testRegistry");

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSimpleConstructor()
   {
      NoisyDoubleYoVariable noisyDoubleYoVariable = new NoisyDoubleYoVariable("testN", yoVariableRegistry);
      noisyDoubleYoVariable.update();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSimpleConstructorDouble()
   {
      DoubleYoVariable doubleYoVariable = new DoubleYoVariable("testD", yoVariableRegistry);
      NoisyDoubleYoVariable noisyDoubleYoVariable = new NoisyDoubleYoVariable("testN", yoVariableRegistry, doubleYoVariable);
      noisyDoubleYoVariable.update();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testFullConstructor()
   {
      NoisyDoubleYoVariable noisyDoubleYoVariable = new NoisyDoubleYoVariable("testN", yoVariableRegistry, false, 2.0, true, 1.0, 2.0, 0.0, 0.01,
                                                       NoiseType.UNIFORM, 1.0);
      noisyDoubleYoVariable.update();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testFullConstructorDouble()
   {
      DoubleYoVariable doubleYoVariable = new DoubleYoVariable("testD", yoVariableRegistry);
      NoisyDoubleYoVariable noisyDoubleYoVariable = new NoisyDoubleYoVariable("testN", yoVariableRegistry, doubleYoVariable, false, 2.0, true, 1.0, 2.0, 0.0,
                                                       0.01, NoiseType.UNIFORM, 1.0);
      noisyDoubleYoVariable.update();
   }

// @Test(timeout=300000)
// public void testUpdateException()
// {
//  NoisyDoubleYoVariable noisyDoubleYoVariable = new NoisyDoubleYoVariable("test", yoVariableRegistry);
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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetPerfectDoubleValue()
   {
      NoisyDoubleYoVariable noisyDoubleYoVariable = new NoisyDoubleYoVariable("testN", yoVariableRegistry);

      noisyDoubleYoVariable.update();
      assertTrue(noisyDoubleYoVariable.getPerfectDoubleValue() == 0.0);

      noisyDoubleYoVariable.update(2.0);
      assertTrue(noisyDoubleYoVariable.getPerfectDoubleValue() == 2.0);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetPerfectDoubleValueDouble()
   {
      DoubleYoVariable doubleYoVariable = new DoubleYoVariable("testD", yoVariableRegistry);
      NoisyDoubleYoVariable noisyDoubleYoVariable = new NoisyDoubleYoVariable("testN", yoVariableRegistry, doubleYoVariable);

      doubleYoVariable.set(2.0);
      assertTrue(noisyDoubleYoVariable.getPerfectDoubleValue() == 2.0);

      noisyDoubleYoVariable.update(3.0);
      assertTrue(noisyDoubleYoVariable.getPerfectDoubleValue() == 3.0);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testDiscrete()
   {
      NoisyDoubleYoVariable noisyDoubleYoVariable = new NoisyDoubleYoVariable("testN", yoVariableRegistry);
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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testRandomBound()
   {
      NoisyDoubleYoVariable noisyDoubleYoVariable = new NoisyDoubleYoVariable("testN", yoVariableRegistry);

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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testZeroRandomBound()
   {
      NoisyDoubleYoVariable noisyDoubleYoVariable = new NoisyDoubleYoVariable("testN", yoVariableRegistry);

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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testZeroRandomBoundWithPerfect()
   {
      NoisyDoubleYoVariable noisyDoubleYoVariable = new NoisyDoubleYoVariable("testN", yoVariableRegistry);

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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testStaticBias()
   {
      NoisyDoubleYoVariable noisyDoubleYoVariable = new NoisyDoubleYoVariable("testN", yoVariableRegistry);

      double bias = 10.0 * Math.random();
      double value = 1000.0 * Math.random();

      noisyDoubleYoVariable.setIsNoisy(true);
      noisyDoubleYoVariable.setRandomBound(0.0);
      noisyDoubleYoVariable.setBias(bias);

      noisyDoubleYoVariable.update(value);

      assertTrue(noisyDoubleYoVariable.getDoubleValue() == value + bias);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testWalkingBias()
   {
      NoisyDoubleYoVariable noisyDoubleYoVariable = new NoisyDoubleYoVariable("testN", yoVariableRegistry);

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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testRandomBoundAndStaticBias()
   {
      NoisyDoubleYoVariable noisyDoubleYoVariable = new NoisyDoubleYoVariable("testN", yoVariableRegistry);

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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testRandomBoundAndWalkingBias()
   {
      NoisyDoubleYoVariable noisyDoubleYoVariable = new NoisyDoubleYoVariable("testN", yoVariableRegistry);

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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGaussianNoise()
   {
      NoisyDoubleYoVariable noisyDoubleYoVariable = new NoisyDoubleYoVariable("testN", yoVariableRegistry);

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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGaussianNoiseAndStaticBias()
   {
      NoisyDoubleYoVariable noisyDoubleYoVariable = new NoisyDoubleYoVariable("testN", yoVariableRegistry);

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
