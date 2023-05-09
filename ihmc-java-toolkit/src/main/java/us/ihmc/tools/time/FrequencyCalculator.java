package us.ihmc.tools.time;

import us.ihmc.commons.time.Stopwatch;
import us.ihmc.tools.UnitConversions;

import java.util.ArrayDeque;

public class FrequencyCalculator
{
   private volatile double frequency;
   private final ArrayDeque<Double> deltas = new ArrayDeque<>();
   private final Stopwatch stopwatch = new Stopwatch().start();
   private final int history;

   /**
    * History of 3.
    */
   public FrequencyCalculator()
   {
      this(3);
   }

   /**
    * History if one yields the latest calculation only.
    * @param history 1 or more.
    */
   public FrequencyCalculator(int history)
   {
      this.history = history;
   }

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

   public boolean anyPingsYet()
   {
      return !deltas.isEmpty();
   }
}
