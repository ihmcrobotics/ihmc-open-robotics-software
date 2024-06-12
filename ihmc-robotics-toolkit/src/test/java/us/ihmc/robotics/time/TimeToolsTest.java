package us.ihmc.robotics.time;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.Conversions;

import java.time.Duration;

import static org.junit.jupiter.api.Assertions.*;

public class TimeToolsTest
{
   private static final double EPSILON = 1E-9;

   @Test
   public void testUnitConversions()
   {
      // test duration
      for (long millis = 0; millis < 10000; ++millis)
      {
         assertEquals(millis / 1000.0, TimeTools.toDoubleSeconds(Duration.ofMillis(millis)), EPSILON);
      }

      for (int seconds = 0; seconds < 10; ++seconds)
      {
         for (int nanos = 0; nanos < 1000000000; nanos += 1000)
         {
            Duration duration = Duration.ofSeconds(seconds, nanos);
            double doubleSeconds = seconds + Conversions.nanosecondsToSeconds(nanos);
            assertEquals(doubleSeconds, TimeTools.toDoubleSeconds(duration), EPSILON);
         }
      }

      // test seconds & millis
      for (long seconds = 0; seconds < 100; ++seconds)
      {
         for (long millis = 0; millis < 10000; ++millis)
         {
            double doubleSeconds = seconds + Conversions.millisecondsToSeconds(millis);
            assertEquals(doubleSeconds, TimeTools.secondsMillisToDoubleSeconds(seconds, millis), EPSILON);
         }
      }

      // test seconds to duration
      Duration twoPointFiveSeconds = TimeTools.durationOfSeconds(2.5);
      assertEquals(2L, twoPointFiveSeconds.getSeconds());
      assertEquals(5E8, twoPointFiveSeconds.getNano());
      for (double seconds = 0.0; seconds < 50.0; seconds += 0.05)
      {
         Duration duration = TimeTools.durationOfSeconds(seconds);

         long durationSeconds = duration.getSeconds();
         long durationNanos = duration.getNano();

         long trueSeconds = (long) seconds;
         long trueNanos = (long) ((seconds - trueSeconds) * 1E9);

         assertEquals(trueSeconds, durationSeconds);
         assertEquals(trueNanos, durationNanos);
      }
   }
}
