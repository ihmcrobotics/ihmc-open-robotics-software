package us.ihmc.perception.detections;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;

import java.util.Random;

public class InstantDetectionTest
{
   @Test
   public void testImmutabilityAfterConstruction()
   {
      Random random = new Random(1738L);
      for (int i = 0; i < 1000; i++)
      {
         double confidence = RandomNumbers.nextDouble(random, 0.0, 1.0);
      }
   }
}
