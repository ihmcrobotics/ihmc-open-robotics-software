package us.ihmc.tools;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;

public class Timer
{
   private final Stopwatch stopwatch = new Stopwatch();

   /**
    * Reset or "crank" the timer back to zero time elapsed.
    *
    * TODO: Rename to "set" as in "set a timer"?
    */
   public void reset()
   {
      stopwatch.start();
   }

   /**
    * @return Total elapsed time.
    */
   public double getElapsedTime()
   {
      return stopwatch.totalElapsed();
   }

   /**
    * If the timer is running, sleep until it expires.
    *
    * @param expirationTime
    */
   public void sleepUntilExpiration(double expirationTime)
   {
      TimerSnapshotWithExpiration snapshot = createSnapshot(expirationTime);
      if (snapshot.isRunning())
      {
         ThreadTools.sleepSeconds(expirationTime - snapshot.getTimePassedSinceReset());
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
    * @param expirationTime
    * @return If the timer has been set and it has expired.
    */
   public boolean isExpired(double expirationTime)
   {
      return isExpired(getElapsedTime(), expirationTime);
   }

   /**
    * @param expirationTime
    * @return If the timer has been set and it's still running.
    */
   public boolean isRunning(double expirationTime)
   {
      return isRunning(getElapsedTime(), expirationTime);
   }

   static boolean isRunning(double timePassedSinceReset, double expirationTime)
   {
      return hasBeenSet(timePassedSinceReset) && !isExpired(timePassedSinceReset, expirationTime);
   }

   static boolean hasBeenSet(double timePassedSinceReset)
   {
      return !Double.isNaN(timePassedSinceReset);
   }

   /**
    * @param timePassedSinceReset
    * @param expirationTime
    * @return false if the timer has never been set. else returns if it's expired.
    */
   static boolean isExpired(double timePassedSinceReset, double expirationTime)
   {
      return hasBeenSet(timePassedSinceReset) && timePassedSinceReset > expirationTime;
   }

   public TimerSnapshotWithExpiration createSnapshot(double expirationTime)
   {
      return new TimerSnapshotWithExpiration(getElapsedTime(), expirationTime);
   }

   public TimerSnapshot createSnapshot()
   {
      return new TimerSnapshot(getElapsedTime());
   }
}
