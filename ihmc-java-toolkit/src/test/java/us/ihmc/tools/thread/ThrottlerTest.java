package us.ihmc.tools.thread;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.robotics.TestTools;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.time.FrequencyCalculator;
import us.ihmc.tools.time.FrequencyStatisticPrinter;

public class ThrottlerTest
{
   @Test
   public void testThrottler10Hz()
   {
   	Throttler throttler = new Throttler();

      Stopwatch stopwatch = new Stopwatch();
      stopwatch.start();

      double hertz = 10.0;
      FrequencyStatisticPrinter frequencyStatisticPrinter = new FrequencyStatisticPrinter();
      FrequencyCalculator frequencyCalculator = new FrequencyCalculator();

      while (stopwatch.totalElapsed() < 5.0)
      {
         if (throttler.run(UnitConversions.hertzToSeconds(hertz)))
         {
            frequencyStatisticPrinter.ping();
            frequencyCalculator.ping();
         }
      }

      frequencyStatisticPrinter.destroy();

      TestTools.assertEpsilonEquals(hertz, frequencyCalculator.getFrequency(), 0.5, "Frequency not correct");
   }

   @Test
   public void testThrottler5Hz()
   {
      double hertz = 5.0;
      Throttler throttler = new Throttler().setFrequency(hertz);

      Stopwatch stopwatch = new Stopwatch();
      stopwatch.start();

      FrequencyStatisticPrinter frequencyStatisticPrinter = new FrequencyStatisticPrinter();
      FrequencyCalculator frequencyCalculator = new FrequencyCalculator();

      while (stopwatch.totalElapsed() < 5.0)
      {
         if (throttler.run())
         {
            frequencyStatisticPrinter.ping();
            frequencyCalculator.ping();
         }
      }

      frequencyStatisticPrinter.destroy();

      TestTools.assertEpsilonEquals(hertz, frequencyCalculator.getFrequency(), 0.5, "Frequency not correct");
   }

   @Test
   public void testThrottler5HzSleep()
   {
      double hertz = 5.0;
      Throttler throttler = new Throttler().setPeriod(UnitConversions.hertzToSeconds(hertz));

      Stopwatch stopwatch = new Stopwatch();
      stopwatch.start();

      FrequencyStatisticPrinter frequencyStatisticPrinter = new FrequencyStatisticPrinter();
      FrequencyCalculator frequencyCalculator = new FrequencyCalculator();

      while (stopwatch.totalElapsed() < 5.0)
      {
         throttler.waitAndRun();
         frequencyStatisticPrinter.ping();
         frequencyCalculator.ping();
      }

      frequencyStatisticPrinter.destroy();

      TestTools.assertEpsilonEquals(hertz, frequencyCalculator.getFrequency(), 0.5, "Frequency not correct");
   }
}
