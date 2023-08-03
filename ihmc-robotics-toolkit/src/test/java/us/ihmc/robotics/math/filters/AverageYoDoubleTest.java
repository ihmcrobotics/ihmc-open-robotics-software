package us.ihmc.robotics.math.filters;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Random;

import org.apache.commons.math3.stat.descriptive.moment.Mean;
import org.junit.jupiter.api.Test;

public class AverageYoDoubleTest
{
   @Test
   public void testAgainstApacheMean()
   {
      Random random = new Random(1);
      AverageYoDouble yoAverage = new AverageYoDouble("", null);
      Mean mean = new Mean();
      double next = -10.0;

      for (int i = 0; i < 1000; i++)
      {
         next += random.nextDouble();
         yoAverage.update(next);
         mean.increment(next);

         assertEquals(mean.getResult(), yoAverage.getValue());
         assertEquals(mean.getN(), yoAverage.getSampleSize());
      }
   }
}
