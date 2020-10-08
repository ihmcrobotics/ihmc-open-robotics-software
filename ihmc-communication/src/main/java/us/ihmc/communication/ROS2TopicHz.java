package us.ihmc.communication;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.Timer;
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

   private final double expiration = 1.0;
   private Timer expirationTimer = new Timer();

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
      newMessages = true;

      ++window;
      expirationTimer.reset();

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

      deltas.add(elapsed);

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
      newMessages = false;

      minDelay = Double.NaN;
      maxDelay = Double.NaN;
      standardDeviation = Double.NaN;
      window = -1;

      stopwatch.reset();
   }

   public static void main(String[] args)
   {
      ROS2Topic<?> topic = ROS2Tools.D435_POINT_CLOUD;
      ROS2TopicHz hz = new ROS2TopicHz();
      ROS2Node node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "hz");
      new IHMCROS2Callback<>(node, topic, ROS2QosProfile.DEFAULT(), message -> hz.ping());

      ThreadTools.sleepForever();
   }
}
