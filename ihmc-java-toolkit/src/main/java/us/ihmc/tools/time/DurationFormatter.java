package us.ihmc.tools.time;

import java.util.concurrent.TimeUnit;

public class DurationFormatter
{
   public static String formatHoursMinutesSecondsMillis(long nanoTime)
   {
      long hours = TimeUnit.NANOSECONDS.toHours(nanoTime);
      long minutes = TimeUnit.NANOSECONDS.toMinutes(nanoTime);
      long seconds = TimeUnit.NANOSECONDS.toSeconds(nanoTime);
      long millis = TimeUnit.NANOSECONDS.toMillis(nanoTime);
      millis -= 1000 * seconds;
      seconds -= 60 * minutes;
      minutes -= 60 * hours;

      String time = String.format("%02ds%03d", seconds, millis);

      if (minutes > 0 || hours > 0)
         time = String.format("%02d", minutes) + "m " + time;
      if (hours > 0)
         time = hours + "h " + time;

      return time;
   }
}