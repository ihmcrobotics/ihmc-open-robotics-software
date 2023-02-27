package us.ihmc.tools.time;

import us.ihmc.commons.time.Stopwatch;

import java.util.ArrayDeque;

public class DurationCalculator
{
   private volatile double duration;
   private final ArrayDeque<Double> deltas = new ArrayDeque<>();
   private final Stopwatch stopwatch = new Stopwatch().start();
   private int history = 3;

   public synchronized void pause()
   {
      stopwatch.suspend();
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
