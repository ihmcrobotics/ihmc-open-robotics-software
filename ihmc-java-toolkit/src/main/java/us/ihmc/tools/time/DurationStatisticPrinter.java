package us.ihmc.tools.time;

import us.ihmc.commons.time.Stopwatch;
import us.ihmc.log.LogTools;
import us.ihmc.tools.Timer;
import us.ihmc.tools.string.StringTools;
import us.ihmc.tools.thread.PausablePeriodicThread;

import java.util.ArrayDeque;

/**
 * This is designed to be functionally the same as ros topic hz but usable for lots of things.
 */
public class DurationStatisticPrinter
{
   boolean newEvents;

   private double averageDuration;
   private double minDuration;
   private double maxDuration;
   private double standardDeviation;
   private int window;

   private final ArrayDeque<Double> deltas = new ArrayDeque<>();

   private final Stopwatch stopwatch = new Stopwatch();

   private final PausablePeriodicThread pausablePeriodicThread;
   private double expiration = 1.0;
   private final Timer expirationTimer = new Timer();
   private final int history;
   private String name;

   public DurationStatisticPrinter()
   {
      this(null, 10, 1.0, "");
   }

   public DurationStatisticPrinter(int history)
   {
      this(null, history, 1.0, "");
   }

   public DurationStatisticPrinter(Runnable onLogReport)
   {
      this(onLogReport, 10, 1.0, "");
   }

   public DurationStatisticPrinter(Runnable onLogReport, int history, double expirationTime, String name)
   {
      this(onLogReport, history, expirationTime, 1.0, name);
   }

   public DurationStatisticPrinter(Runnable onLogReport, int history, double expirationTime, double printFrequency, String name)
   {
      this.history = history;
      this.name = name;
      this.expiration = expirationTime;

      reset();

      boolean runAsDaemon = true;
      pausablePeriodicThread = new PausablePeriodicThread(getClass().getSimpleName(), printFrequency, 0, runAsDaemon, () ->
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
         LogTools.info(2, StringTools.format3SF(name + ": average duration: {}\n        min: {}s max: {}s std dev: {}s window: {}",
                                                averageDuration, minDuration, maxDuration, standardDeviation, window).get());
      }
   }

   public synchronized void before()
   {
      stopwatch.reset();
   }

   public synchronized double after()
   {
      newEvents = true;

      ++window;
      expirationTimer.reset();

      double elapsed = stopwatch.totalElapsed();

      if (Double.isNaN(minDuration) || elapsed < minDuration)
      {
         minDuration = elapsed;
      }

      if (Double.isNaN(maxDuration) || elapsed > maxDuration)
      {
         maxDuration = elapsed;
      }

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
      averageDuration = totalElapsed / deltas.size();

      if (deltas.size() < 2)
      {
         standardDeviation = 0.0;
      }
      else
      {
         double sumOfSquares = 0.0;
         for (Double delta : deltas)
         {
            sumOfSquares += Math.pow(delta - averageDuration, 2.0);
         }

         standardDeviation = Math.sqrt(sumOfSquares / deltas.size());
      }

      return elapsed;
   }

   public synchronized void reset()
   {
      newEvents = false;

      minDuration = Double.NaN;
      maxDuration = Double.NaN;
      standardDeviation = Double.NaN;
      window = -1;

      stopwatch.reset();
   }

   public void destroy()
   {
      pausablePeriodicThread.destroy();
   }
}
