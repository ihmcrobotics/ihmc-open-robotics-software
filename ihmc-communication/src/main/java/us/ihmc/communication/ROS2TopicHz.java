package us.ihmc.communication;

import us.ihmc.commons.time.Stopwatch;
import us.ihmc.log.LogTools;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.string.StringTools;
import us.ihmc.tools.thread.PausablePeriodicThread;

import java.util.ArrayList;
import java.util.List;

public class ROS2TopicHz
{
   boolean newMessages;

   private double averageRate;
   private double minDelay;
   private double maxDelay;
   private double standardDeviation;
   private int window;

   private final List<Double> deltas = new ArrayList<>();

   private Stopwatch stopwatch = new Stopwatch();

   public ROS2TopicHz()
   {
      reset();

      boolean runAsDaemon = true;
      new PausablePeriodicThread(getClass().getSimpleName(), 1.0, 0, runAsDaemon, this::logReport).start();
   }

   private void logReport()
   {
      if (!newMessages)
      {
         reset();
         LogTools.info("no new messages");
      }
      else
      {
         LogTools.info(StringTools.format3D("averate rate: {}\n        min: {}s max: {}s std dev: {}s window: {}",
                                            averageRate, minDelay, maxDelay, standardDeviation, window));
      }
   }

   public synchronized void ping()
   {
      newMessages = true;

      ++window;

      double elapsed = stopwatch.lap();
      averageRate = UnitConversions.secondsToHertz(stopwatch.averageLap());

      if (Double.isNaN(minDelay) || elapsed < minDelay)
      {
         minDelay = elapsed;
      }

      if (Double.isNaN(maxDelay) || elapsed > maxDelay)
      {
         maxDelay = elapsed;
      }


   }

   public synchronized void reset()
   {
      newMessages = false;

      minDelay = Double.NaN;
      maxDelay = Double.NaN;
      standardDeviation = Double.NaN;
      window = -1;

      stopwatch.reset();
   }
}
