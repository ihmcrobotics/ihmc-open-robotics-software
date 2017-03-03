package us.ihmc.commons;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class RandomNumbersTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGenerateRandomFloatArray()
   {
      Random random = new Random();

      float[] randomFloatArray = RandomNumbers.nextFloatArray(random, 1000, 0, 10);

      for (float randomFloat : randomFloatArray)
      {
         assertTrue((randomFloat >= 0) && (randomFloat <= 10));
      }

      randomFloatArray = RandomNumbers.nextFloatArray(random, 1000, 5);

      for (float randomFloat : randomFloatArray)
      {
         assertTrue((randomFloat >= -5) && (randomFloat <= 5));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGenerateRandomIntArray()
   {
      Random random = new Random();

      int[] randomIntArray = RandomNumbers.nextIntArray(random, 1000, 0, 10);

      for (int randomInt : randomIntArray)
      {
         assertTrue((randomInt >= 0) && (randomInt <= 10));
      }

      randomIntArray = RandomNumbers.nextIntArray(random, 1000, 5);

      for (float randomFloat : randomIntArray)
      {
         assertTrue((randomFloat >= -5) && (randomFloat <= 5));
      }
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGenerateRandomDoubleInRange()
   {
      Random random = new Random(1876L);
      double range1 = 100.0;
      double range2 = 30.0;

      for (int i = 0; i < 25; i++)
      {
         double actualReturn = RandomNumbers.nextDouble(random, range1, range2);
         assertTrue(((range1 < actualReturn) && (actualReturn < range2)) || ((range2 < actualReturn) && (actualReturn < range1)));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGenerateRandomFloatInRange()
   {
      Random random = new Random(1876L);
      float range1 = 100.0f;
      float range2 = 30.0f;

      for (int i = 0; i < 25; i++)
      {
         double actualReturn = RandomNumbers.nextFloat(random, range1, range2);
         assertTrue(((range1 < actualReturn) && (actualReturn < range2)) || ((range2 < actualReturn) && (actualReturn < range1)));
      }
   }
}
