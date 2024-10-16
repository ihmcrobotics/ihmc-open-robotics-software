package us.ihmc.yoVariables.filters;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.yoVariables.registry.YoRegistry;

import static org.junit.jupiter.api.Assertions.*;

public class SimpleMovingAverageFilteredYoVariableTest
{
   @Test
   public void testWithFixedSizeDoubleArrays()
   {
      for (int i = 0; i < 100; i++)
      {
         YoRegistry registry = new YoRegistry("Blop");
         Random random = new Random(6541654L);
         int windowSize = RandomNumbers.nextInt(random, 1, 1000);
         SimpleMovingAverageFilteredYoVariable sma = new SimpleMovingAverageFilteredYoVariable("tested", windowSize, registry);
         double amplitude = 100.0;
         double[] randomArray = RandomNumbers.nextDoubleArray(random, windowSize, amplitude);
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
