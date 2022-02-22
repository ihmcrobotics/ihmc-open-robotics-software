package us.ihmc.tools.time;

import us.ihmc.commons.time.Stopwatch;
import us.ihmc.tools.UnitConversions;

import java.util.ArrayDeque;

public class FrequencyCalculator
{
   private volatile double frequency;
   private final ArrayDeque<Double> deltas = new ArrayDeque<>();
   private final Stopwatch stopwatch = new Stopwatch().start();
   private int history = 3;

   public void ping()
   {
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
      frequency = UnitConversions.secondsToHertz(totalElapsed / deltas.size());
   }

   public double getFrequency()
   {
      return frequency;
   }
}
