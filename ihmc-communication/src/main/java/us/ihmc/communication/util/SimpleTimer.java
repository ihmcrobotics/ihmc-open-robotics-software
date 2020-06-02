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
      return isPastOrNaN(timePassedSinceReset(), time);
   }

   private boolean isPastOrNaN(double timePassedSinceReset, double time)
   {
      return Double.isNaN(timePassedSinceReset) || timePassedSinceReset > time;
   }

   public double timePassedSinceReset()
   {
      return stopwatch.totalElapsed();
   }

   public Status getStatus(double time)
   {
      double timePassedSinceReset = timePassedSinceReset();
      return new Status(isPastOrNaN(timePassedSinceReset, time), timePassedSinceReset);
   }

   public class Status
   {
      private final boolean isPastOrNaN;
      private final double timePassedSinceReset;

      public Status(boolean isPastOrNaN, double timePassedSinceReset)
      {
         this.isPastOrNaN = isPastOrNaN;
         this.timePassedSinceReset = timePassedSinceReset;
      }

      public boolean isPastOrNaN()
      {
         return isPastOrNaN;
      }

      public double getTimePassedSinceReset()
      {
         return timePassedSinceReset;
      }
   }
}
