package us.ihmc.communication.ros2log;

import com.google.common.base.CaseFormat;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Topic;

import java.io.File;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.LongSupplier;
import java.util.function.ObjLongConsumer;

public class ROS2LogReplay
{
   private final List<ReplayTopicManager<?>> topicManagers;
   private final Map<String, ReplayTopicManager<?>> topicManagersMap = new HashMap<>();
   private final File logFile;
   private final LongSupplier timestampSupplier;

   public ROS2LogReplay(String robotName, List<ROS2Topic<?>> topicsToLog, File logFile, ROS2LogTimeSource timeSource)
   {
      ROS2Node ros2Node = new ROS2NodeBuilder().build("ihmc_" + CaseFormat.UPPER_CAMEL.to(CaseFormat.LOWER_UNDERSCORE, getClass().getSimpleName()));
      topicManagers = ROS2LogIOTools.loadLogFile(ros2Node, topicsToLog, logFile);
      this.logFile = logFile;
      timestampSupplier = timeSource.createTimestampProvider(robotName, ros2Node);

      if (topicManagers == null)
         return;

      for (int i = 0; i < topicManagers.size(); i++)
      {
         topicManagersMap.put(topicManagers.get(i).getTopicName(), topicManagers.get(i));
      }

      if (timeSource == ROS2LogTimeSource.SIMULATION)
      {
         while (timestampSupplier.getAsLong() == -1)
         {
            LogTools.info("Waiting for simulation timestamp");
            ThreadTools.sleep(1000);
         }
      }
   }

   public void start()
   {
      LogTools.info("Starting replay of " + logFile.getName());
      startReplayInternal(topicManagers, timestampSupplier);
      LogTools.info("Finished replay");

      System.exit(0);
   }

   private void startReplayInternal(List<ReplayTopicManager<?>> topicManagers, LongSupplier timestampSupplier)
   {
      long startTime = timestampSupplier.getAsLong();

      while (true)
      {
         long now = timestampSupplier.getAsLong() - startTime;
         boolean isDone = true;

         for (int topic_idx = 0; topic_idx < topicManagers.size(); topic_idx++)
         {
            ReplayTopicManager<?> topicManager = topicManagers.get(topic_idx);
            isDone = topicManager.update(now) && isDone;
         }

         if (isDone)
            break;
      }
   }

   public <T> void addReplayMutator(ROS2Topic<T> topic, ObjLongConsumer<T> mutator)
   {
      topicManagersMap.get(topic.getName()).setMutator(mutator);
   }
}
