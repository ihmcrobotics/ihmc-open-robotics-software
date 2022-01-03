package us.ihmc.tools.time;

import us.ihmc.commons.time.Stopwatch;
import us.ihmc.log.LogTools;
import us.ihmc.tools.Timer;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.string.StringTools;
import us.ihmc.tools.thread.PausablePeriodicThread;

import java.util.ArrayDeque;

/**
 * This is designed to be functionally the same as ros topic hz but usable for lots of things.
 */
public class FrequencyStatisticPrinter
{
   boolean newEvents;

   private double averageRate;
   private double minDelay;
   private double maxDelay;
   private double standardDeviation;
   private int window;

   private final ArrayDeque<Double> deltas = new ArrayDeque<>();

   private final Stopwatch stopwatch = new Stopwatch();

   private PausablePeriodicThread pausablePeriodicThread;
   private final double expiration = 1.0;
   private final Timer expirationTimer = new Timer();

   public FrequencyStatisticPrinter()
   {
      this(null);
   }

   public FrequencyStatisticPrinter(Runnable onLogReport)
   {
      reset();

      boolean runAsDaemon = true;
      pausablePeriodicThread = new PausablePeriodicThread(getClass().getSimpleName(), 1.0, 0, runAsDaemon, () ->
      {
         logReport();
         if (onLogReport != null)
            onLogReport.run();
      });
      pausablePeriodicThread.start();
   }

   private void logReport()
   {
      if (!newEvents)
      {
         reset();
         LogTools.info("no new events");
      }
      else if (!expirationTimer.isRunning(expiration))
      {
         reset();
      }
      else
      {
         LogTools.info(StringTools.format3D("averate rate: {}\n        min: {}s max: {}s std dev: {}s window: {}",
                                            averageRate, minDelay, maxDelay, standardDeviation, window));
      }
   }

   public synchronized void ping()
   {
      newEvents = true;

      ++window;
      expirationTimer.reset();

      double elapsed = stopwatch.lap();

      if (Double.isNaN(minDelay) || elapsed < minDelay)
      {
         minDelay = elapsed;
      }

      if (Double.isNaN(maxDelay) || elapsed > maxDelay)
      {
         maxDelay = elapsed;
      }

      deltas.addLast(elapsed);

      while (deltas.size() > 10)
      {
         deltas.removeFirst();
      }

      double totalElapsed = 0.0;
      for (Double delta : deltas)
      {
         totalElapsed += delta;
      }
      averageRate = UnitConversions.secondsToHertz(totalElapsed / deltas.size());

      if (deltas.size() < 2)
      {
         standardDeviation = 0.0;
      }
      else
      {
         double average = stopwatch.averageLap();

         double sumOfSquares = 0.0;
         for (Double delta : deltas)
         {
            sumOfSquares += Math.pow(delta - average, 2.0);
         }

         standardDeviation = Math.sqrt(sumOfSquares / deltas.size());
      }
   }

   public synchronized void reset()
   {
      newEvents = false;

      minDelay = Double.NaN;
      maxDelay = Double.NaN;
      standardDeviation = Double.NaN;
      window = -1;

      stopwatch.reset();
   }

   public void destroy()
   {
      pausablePeriodicThread.destroy();
   }
}
