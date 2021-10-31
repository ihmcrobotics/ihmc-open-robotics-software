package us.ihmc.tools.thread;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.time.FrequencyStatisticPrinter;

public class ThrottlerTest
{
   @Test
   public void testThrottler10Hz()
   {
   	Throttler throttler = new Throttler();

      Stopwatch stopwatch = new Stopwatch();
      stopwatch.start();

      FrequencyStatisticPrinter frequencyStatisticPrinter = new FrequencyStatisticPrinter();

      while (stopwatch.totalElapsed() < 5.0)
      {
         if (throttler.run(UnitConversions.hertzToSeconds(10.0)))
         {
            frequencyStatisticPrinter.ping();
         }
      }

      frequencyStatisticPrinter.destroy();
   }
}
