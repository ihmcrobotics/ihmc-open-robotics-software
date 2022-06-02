package us.ihmc.robotics.time;

import us.ihmc.commons.Conversions;

import java.time.Instant;

public class TimeTools
{
   /**
    * Nanoseconds since the epoch goes beyond what a long can hold. (?)
    * Or anyways, we do what Instant tells us to.
    */
   public static double calculateDelay(long eventSecondsSinceEpoch, long eventAdditionalNanos)
   {
      Instant now = Instant.now();
      long seconds = now.getEpochSecond() - eventSecondsSinceEpoch;
      long nano = now.getNano() - eventAdditionalNanos;
      return seconds + Conversions.nanosecondsToSeconds(nano);
   }
}
