package us.ihmc.tools.time;

import us.ihmc.commons.time.Stopwatch;

import java.util.ArrayDeque;

public class MovingAverageDurationCalculator
{
   private volatile double duration = Double.NaN;
   private final ArrayDeque<Double> deltas = new ArrayDeque<>();
   private final Stopwatch stopwatch = new Stopwatch().start();
   private final int windowSize;

   /**
    * Window size of one yields the latest calculation only.
    * @param windowSize 1 or more.
    */
   public MovingAverageDurationCalculator(int windowSize)
   {
      this.windowSize = windowSize;
   }

   public synchronized void pause()
   {
      stopwatch.suspend();
   }

   public void reset()
   {
      stopwatch.reset();
   }

   public synchronized void ping()
   {
      stopwatch.resume();

      double elapsed = stopwatch.lap();
      deltas.addLast(elapsed);

      while (deltas.size() > windowSize)
      {
         deltas.removeFirst();
      }

      double totalElapsed = 0.0;
      for (Double delta : deltas)
      {
         totalElapsed += delta;
      }
      duration = totalElapsed / deltas.size();
   }

   public synchronized double getDuration()
   {
      return duration;
   }
}
