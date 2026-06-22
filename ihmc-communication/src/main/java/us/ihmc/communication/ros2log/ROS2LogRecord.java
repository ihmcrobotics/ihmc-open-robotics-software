package us.ihmc.communication.ros2log;

import toolbox_msgs.ROS2LogMessage;
import us.ihmc.communication.HumanoidROS2Topic;
import us.ihmc.log.LogTools;
import us.ihmc.jros2.ROS2Message;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Topic;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.LongSupplier;

public class ROS2LogRecord
{
   private static final String MODULE_NAME = "ros2_log";
   private static final int TIMEOUT_MILLIS = 10000;

   private final ROS2Node ros2Node;
   private final List<RecordTopicManager<?>> topicManagers = new ArrayList<>();
   private final AtomicBoolean stopRequested = new AtomicBoolean();
   private final ROS2LogSerialization serialization;
   private final LongSupplier timestampProvider;

   private Runnable runnable = null;
   private ScheduledThreadPoolExecutor executorService;
   private final AtomicLong lastReceivedTimestamp = new AtomicLong();

   public ROS2LogRecord(String robotName, List<ROS2Topic<?>> topicsToLog, ROS2LogTimeSource timeSource, ROS2LogSerialization serialization)
   {
      this(robotName, topicsToLog, timeSource, serialization, true);
   }

   public ROS2LogRecord(String robotName,
                        List<ROS2Topic<?>> topicsToLog,
                        ROS2LogTimeSource timeSource,
                        ROS2LogSerialization serialization,
                        boolean subscribeToTopics)
   {
      ros2Node = new ROS2Node("ihmc_ros2_logger");
      this.serialization = serialization;

      ros2Node.createSubscription(getROS2LogTopic(), reader ->
      {
         ROS2LogMessage message = reader.read();
         if (message == null)
            return;

         ROS2LoggerRequestedState requestedState = ROS2LoggerRequestedState.fromByte(message.getRequestedState());
         if (requestedState == ROS2LoggerRequestedState.START)
            start();
         else if (requestedState == ROS2LoggerRequestedState.FINISH)
            stopRequested.set(true);
      });
      timestampProvider = timeSource.createTimestampProvider(robotName, ros2Node);

      for (int i = 0; i < topicsToLog.size(); i++)
      {
         ROS2Topic<?> ros2Topic = topicsToLog.get(i);
         topicManagers.add(new RecordTopicManager<>(ros2Topic, ros2Node, timestampProvider, subscribeToTopics));
      }
   }

   public <T extends ROS2Message<T>> void setData(ROS2Topic<T> topic, T data)
   {
      RecordTopicManager<T> topicManager = getTopicManager(topic);
      if (topicManager == null)
      {
         LogTools.warn("Skipping data for unregistered ROS 2 log topic: {}", topic.getName());
         return;
      }

      topicManager.setData(timestampProvider.getAsLong(), data);
   }

   public void start()
   {
      if (runnable != null)
         return;

      LogTools.info("Starting ROS 2 logger");
      runnable = this::update;
      stopRequested.set(false);

      executorService = new ScheduledThreadPoolExecutor(1);
      executorService.scheduleAtFixedRate(runnable, 0, 1, TimeUnit.MILLISECONDS);
      lastReceivedTimestamp.set(System.currentTimeMillis());
   }

   public void stop()
   {
      stopRequested.set(true);
   }

   public void update()
   {
      if (runnable == null)
         return;

      boolean hasData = false;
      for (RecordTopicManager<?> topicManager : topicManagers)
      {
         hasData |= topicManager.update();
      }

      if (hasData)
         lastReceivedTimestamp.set(System.currentTimeMillis());
      else if (System.currentTimeMillis() - lastReceivedTimestamp.get() > TIMEOUT_MILLIS)
         stopRequested.set(true);

      if (stopRequested.getAndSet(false))
         stopAndFlush();
   }

   public void destroy()
   {
      if (executorService != null)
      {
         executorService.shutdownNow();
      }
      ros2Node.close();
   }

   public static ROS2Topic<ROS2LogMessage> getROS2LogTopic()
   {
      return HumanoidROS2Topic.IHMC_ROOT.withModule(MODULE_NAME).withTypeName(ROS2LogMessage.class);
   }

   @SuppressWarnings("unchecked")
   private <T extends ROS2Message<T>> RecordTopicManager<T> getTopicManager(ROS2Topic<T> topic)
   {
      for (RecordTopicManager<?> manager : topicManagers)
      {
         if (manager.getTopic().equals(topic))
            return (RecordTopicManager<T>) manager;
      }
      return null;
   }

   private void stopAndFlush()
   {
      LogTools.info("Stopping ROS 2 logger, writing to file...");

      ROS2LogIOTools.writeLogFile(topicManagers, serialization);
      runnable = null;
      executorService.shutdown();

      for (RecordTopicManager<?> topicManager : topicManagers)
      {
         topicManager.clear();
      }
   }
}
