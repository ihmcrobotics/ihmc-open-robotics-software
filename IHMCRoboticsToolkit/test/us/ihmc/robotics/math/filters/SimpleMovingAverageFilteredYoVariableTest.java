package us.ihmc.robotics.math.filters;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.random.RandomTools;

public class SimpleMovingAverageFilteredYoVariableTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testWithFixedSizeDoubleArrays() throws Exception
   {
      for (int i = 0; i < 100; i++)
      {
         YoVariableRegistry registry = new YoVariableRegistry("Blop");
         Random random = new Random(6541654L);
         int windowSize = RandomTools.generateRandomInt(random, 1, 1000);
         SimpleMovingAverageFilteredYoVariable sma = new SimpleMovingAverageFilteredYoVariable("tested", windowSize, registry);
         double amplitude = 100.0;
         double[] randomArray = RandomTools.generateRandomDoubleArray(random, windowSize, amplitude);
         double expected = 0.0;
         for (double val : randomArray)
            expected += val / windowSize;

         for (int j = 0; j < randomArray.length; j++)
         {
            assertFalse(sma.getHasBufferWindowFilled());
            sma.update(randomArray[j]);
         }

         assertTrue(sma.getHasBufferWindowFilled());
         assertEquals(expected, sma.getDoubleValue(), 1.0e-10);
      }
   }
}
