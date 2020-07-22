package us.ihmc.communication.util;

import us.ihmc.commons.time.Stopwatch;

public class Timer
{
   private final Stopwatch stopwatch = new Stopwatch();

   public void reset()
   {
      stopwatch.start();
   }

   public double getElapsedTime()
   {
      return stopwatch.totalElapsed();
   }

   public boolean hasBeenSet()
   {
      return  hasBeenSet(getElapsedTime());
   }

   public boolean isExpired(double expirationTime)
   {
      return isExpired(getElapsedTime(), expirationTime);
   }

   static boolean hasBeenSet(double timePassedSinceReset)
   {
      return !Double.isNaN(timePassedSinceReset);
   }

   static boolean isExpired(double timePassedSinceReset, double expirationTime)
   {
      return timePassedSinceReset > expirationTime;
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
