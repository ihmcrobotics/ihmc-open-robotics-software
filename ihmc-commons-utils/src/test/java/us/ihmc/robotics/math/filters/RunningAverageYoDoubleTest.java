package us.ihmc.robotics.math.filters;

import java.util.Random;

import org.apache.commons.math3.stat.descriptive.moment.Mean;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class RunningAverageYoDoubleTest
{
   @Test
   public void testAgainstApacheMean()
   {
      Random random = new Random(1);
      RunningAverageYoDouble yoAverage = new RunningAverageYoDouble("", null);
      Mean mean = new Mean();
      double next = -10.0;

      for (int i = 0; i < 1000; i++)
      {
         next += random.nextDouble();
         yoAverage.update(next);
         mean.increment(next);

         Assertions.assertEquals(mean.getResult(), yoAverage.getValue());
         Assertions.assertEquals(mean.getN(), yoAverage.getSampleSize());
      }
   }
}
