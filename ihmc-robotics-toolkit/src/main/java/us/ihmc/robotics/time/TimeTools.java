package us.ihmc.robotics.time;

import us.ihmc.commons.Conversions;

import java.time.Instant;

/**
 * This class is to assist with calulating delays and timings
 * when multiple processes are involved.
 *
 * If multiple machines are involved, it is best to use this in combination
 * with Chrony (https://chrony.tuxfamily.org/) such that system clocks on the computers
 * involved are as closely synchronized as possible.
 *
 * In general, you want to use Instant.now and send two long values over the network.
 */
public class TimeTools
{
   public static Instant now()
   {
      return Instant.now();
   }

   /**
    * @return Seconds since then.
    */
   public static double calculateDelay(Instant then)
   {
      return calculateDelay(then.getEpochSecond(), then.getNano());
   }

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
