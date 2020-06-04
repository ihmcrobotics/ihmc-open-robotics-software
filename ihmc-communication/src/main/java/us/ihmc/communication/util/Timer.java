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

   public boolean isExpired(double time)
   {
      return isExpired(getElapsedTime(), time);
   }

   static boolean hasBeenSet(double timePassedSinceReset)
   {
      return !Double.isNaN(timePassedSinceReset);
   }

   static boolean isExpired(double timePassedSinceReset, double time)
   {
      return timePassedSinceReset > time;
   }

   public TimerSnapshot createSnapshot(double time)
   {
      return new TimerSnapshot(getElapsedTime(), time);
   }
}
