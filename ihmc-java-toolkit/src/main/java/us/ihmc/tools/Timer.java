package us.ihmc.tools;

import us.ihmc.commons.Conversions;
import us.ihmc.tools.thread.MissingThreadTools;

public class Timer
{
   private double resetTime = Double.NaN;

   /**
    * Reset or "crank" the timer back to zero time elapsed.
    */
   public void reset()
   {
      resetTime = Conversions.nanosecondsToSeconds(System.nanoTime());
   }

   /**
    * @return Total elapsed time.
    */
   public double getElapsedTime()
   {
      return Conversions.nanosecondsToSeconds(System.nanoTime()) - resetTime;
   }

   /**
    * If the timer is running, sleep until it expires.
    *
    * @param expirationDuration
    */
   public void sleepUntilExpiration(double expirationDuration)
   {
      if (!Double.isNaN(resetTime))
      {
         double expirationTime = resetTime + expirationDuration;
         double remainingDuration = expirationTime - Conversions.nanosecondsToSeconds(System.nanoTime());

         MissingThreadTools.sleepAtLeast(remainingDuration);
      }
   }

   /**
    * @return If the timer has ever been set. (i.e. reset() has ever been called)
    */
   public boolean hasBeenSet()
   {
      return hasBeenSet(getElapsedTime());
   }

   /**
    * @param expirationDuration
    * @return If the timer has been set and it has expired.
    */
   public boolean isExpired(double expirationDuration)
   {
      return isExpired(getElapsedTime(), expirationDuration);
   }

   /**
    * @param expirationDuration
    * @return If the timer has been set and it's still running.
    */
   public boolean isRunning(double expirationDuration)
   {
      return isRunning(getElapsedTime(), expirationDuration);
   }

   static boolean isRunning(double timePassedSinceReset, double expirationDuration)
   {
      return hasBeenSet(timePassedSinceReset) && !isExpired(timePassedSinceReset, expirationDuration);
   }

   static boolean hasBeenSet(double timePassedSinceReset)
   {
      return !Double.isNaN(timePassedSinceReset);
   }

   /**
    * @param timePassedSinceReset
    * @param expirationDuration
    * @return false if the timer has never been set. else returns if it's expired.
    */
   static boolean isExpired(double timePassedSinceReset, double expirationDuration)
   {
      return hasBeenSet(timePassedSinceReset) && timePassedSinceReset > expirationDuration;
   }

   public TimerSnapshotWithExpiration createSnapshot(double expirationDuration)
   {
      return new TimerSnapshotWithExpiration(getElapsedTime(), expirationDuration);
   }

   public TimerSnapshot createSnapshot()
   {
      return new TimerSnapshot(getElapsedTime());
   }
}
