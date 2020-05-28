package us.ihmc.communication.util;

import us.ihmc.commons.time.Stopwatch;

public class SimpleTimer
{
   private final Stopwatch stopwatch = new Stopwatch();

   public void reset()
   {
      stopwatch.start();
   }

   public boolean isPastOrNaN(double time)
   {
      return Double.isNaN(timePassedSinceReset()) || timePassedSinceReset() > time;
   }

   public double timePassedSinceReset()
   {
      return stopwatch.totalElapsed();
   }
}
