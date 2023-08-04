package us.ihmc.tools.time;

import us.ihmc.commons.time.Stopwatch;

import java.util.ArrayDeque;

public class DurationCalculator
{
   private volatile double duration = Double.NaN;
   private final ArrayDeque<Double> deltas = new ArrayDeque<>();
   private final Stopwatch stopwatch = new Stopwatch().start();
   private final int history;

   /**
    * History of one yields the latest calculation only.
    * @param history 1 or more.
    */
   public DurationCalculator(int history)
   {
      this.history = history;
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

      while (deltas.size() > history)
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
