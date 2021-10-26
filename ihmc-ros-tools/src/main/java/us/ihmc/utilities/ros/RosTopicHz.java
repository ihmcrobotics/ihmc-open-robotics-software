package us.ihmc.utilities.ros;

import org.apache.commons.lang3.mutable.MutableInt;
import sensor_msgs.PointCloud2;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.Timer;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.string.StringTools;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;

public class RosTopicHz
{
   boolean newMessages;

   private double averageRate;
   private double minDelay;
   private double maxDelay;
   private double standardDeviation;
   private int window;

   private final ArrayDeque<Double> deltas = new ArrayDeque<>();

   private Stopwatch stopwatch = new Stopwatch();

   private final double expiration = 1.0;
   private Timer expirationTimer = new Timer();

   public RosTopicHz()
   {
      this(null);
   }

   public RosTopicHz(Runnable onLogReport)
   {
      reset();

      boolean runAsDaemon = true;
      new PausablePeriodicThread(getClass().getSimpleName(), 1.0, 0, runAsDaemon, () ->
      {
         logReport();
         if (onLogReport != null)
            onLogReport.run();
      }).start();
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
      newMessages = false;

      minDelay = Double.NaN;
      maxDelay = Double.NaN;
      standardDeviation = Double.NaN;
      window = -1;

      stopwatch.reset();
   }

   public static void main(String[] args)
   {
      String topic = RosTools.L515_POINT_CLOUD;
      MutableInt numberOfPointsInAScan = new MutableInt();
      RosTopicHz hz = new RosTopicHz(() ->
      {
         LogTools.info("Number of points in a scan: {}", numberOfPointsInAScan.getValue());
      });
      ROS1Helper helper = new ROS1Helper("hz");
      AbstractRosTopicSubscriber<PointCloud2> subscriber = new AbstractRosTopicSubscriber<PointCloud2>(PointCloud2._TYPE)
      {
         @Override
         public void onNewMessage(PointCloud2 pointCloud2)
         {
            hz.ping();
            numberOfPointsInAScan.setValue(pointCloud2.getHeight() * pointCloud2.getWidth());
         }
      };
      helper.attachSubscriber(topic, subscriber);

      ThreadTools.sleepForever();
   }
}
