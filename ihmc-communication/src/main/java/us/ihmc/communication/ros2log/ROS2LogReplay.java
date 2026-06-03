package us.ihmc.communication.ros2log;

import com.google.common.base.CaseFormat;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.jros2.ROS2Message;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Topic;

import java.io.File;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.LongSupplier;
import java.util.function.ObjLongConsumer;

public class ROS2LogReplay
{
   private final ROS2Node ros2Node;
   private final ROS2LogTimeSource timeSource;
   private List<ReplayTopicManager<?>> topicManagers;
   private final Map<String, ReplayTopicManager<?>> topicManagersMap = new HashMap<>();
   private final LongSupplier timestampSupplier;
   private final List<ROS2Topic<?>> loggedTopics;
   private boolean paused = false;

   private boolean firstUpdate = true;
   private long startTime;
   private long lastReplayTime = 0L;
   private long pauseStartTime = 0L;
   private boolean pendingPause = false;
   private long totalPausedDuration = 0L;
   private double replaySpeed = 1.0;

   public ROS2LogReplay(String robotName, List<ROS2Topic<?>> loggedTopics, ROS2LogTimeSource timeSource)
   {
      this.timeSource = timeSource;
      this.loggedTopics = loggedTopics;

      ros2Node = new ROS2Node("ihmc_ros2_log_replay");
      timestampSupplier = timeSource.createTimestampProvider(robotName, ros2Node);
   }

   public ROS2LogReplay(String robotName, List<ROS2Topic<?>> loggedTopics, File logFile, ROS2LogTimeSource timeSource)
   {
      this.timeSource = timeSource;
      this.loggedTopics = loggedTopics;

      ros2Node = new ROS2Node("ihmc_ros2_log_replay");
      topicManagers = ROS2LogIOTools.loadLogFile(ros2Node, loggedTopics, logFile);
      timestampSupplier = timeSource.createTimestampProvider(robotName, ros2Node);

      populateTopicManagers();
   }

   public ROS2LogReplay(List<ROS2Topic<?>> loggedTopics, File logFile, Function<ROS2Topic, Consumer> messageConsumerGenerator, LongSupplier timestampSupplier)
   {
      this.ros2Node = null;
      this.timeSource = ROS2LogTimeSource.SIMULATION;
      this.timestampSupplier = timestampSupplier;
      this.loggedTopics = loggedTopics;

      topicManagers = ROS2LogIOTools.loadLogFile(logFile, loggedTopics, messageConsumerGenerator);

      populateTopicManagers();
   }

   public void load(File logFile)
   {
      if (logFile == null || !logFile.exists() || !logFile.isFile())
      {
         LogTools.warn("Log file not found: {}", logFile != null ? logFile.getAbsolutePath() : "null");
         return;
      }
      LogTools.info("Loading file {}", logFile.getName());
      topicManagers = ROS2LogIOTools.loadLogFile(ros2Node, loggedTopics, logFile);
      populateTopicManagers();
   }

   private void populateTopicManagers()
   {
      if (topicManagers == null)
         return;

      for (int i = 0; i < topicManagers.size(); i++)
      {
         topicManagersMap.put(topicManagers.get(i).getTopicName(), topicManagers.get(i));
      }
   }

   /**
    * This performs a single update on the current thread, intended for use within the simulation process.
    */
   public boolean doIncrementalReplay()
   {
      if (timeSource == ROS2LogTimeSource.SIMULATION && timestampSupplier.getAsLong() == -1)
      {
         return false;
      }
      else if (firstUpdate)
      {
         startTime = timestampSupplier.getAsLong();
         totalPausedDuration = 0L;
         firstUpdate = false;
      }

      // Handle pending pause now that time has advanced
      if (pendingPause && lastReplayTime > 0)
      {
         paused = true;
         pauseStartTime = timestampSupplier.getAsLong();
         pendingPause = false;
         LogTools.info("Replay paused (pending request fulfilled)");
      }

      if (paused)
      {
         // Keep publishing the last known message while paused
         for (ReplayTopicManager<?> topicManager : topicManagers)
         {
            topicManager.updateInternalRepeat(lastReplayTime);
         }
         return false; // replay not advancing while paused
      }

      long elapsed = timestampSupplier.getAsLong() - startTime - totalPausedDuration;
      long now = (long) (elapsed * replaySpeed);
      lastReplayTime = now;
      boolean isDone = true;

      for (int topic_idx = 0; topic_idx < topicManagers.size(); topic_idx++)
      {
         ReplayTopicManager<?> topicManager = topicManagers.get(topic_idx);
         isDone = topicManager.update(now) && isDone;
      }

      return isDone;
   }

   /**
    * This performs a complete replay on the current thread, and returns when the replay is complete.
    */
   public void doFullReplay()
   {
      if (timeSource == ROS2LogTimeSource.SIMULATION)
      {
         while (timestampSupplier.getAsLong() == -1)
         {
            LogTools.info("Waiting for simulation timestamp");
            ThreadTools.sleep(1000);
         }
      }

      startReplayInternal(topicManagers, timestampSupplier);
      System.exit(0);
   }

   public ROS2Node getROS2Node()
   {
      return ros2Node;
   }

   private void startReplayInternal(List<ReplayTopicManager<?>> topicManagers, LongSupplier timestampSupplier)
   {
      long startTime = timestampSupplier.getAsLong();
      while (true)
      {
         long elapsed = timestampSupplier.getAsLong() - startTime;
         long now = (long) (elapsed * replaySpeed);
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

   public void pauseReplay(boolean pause)
   {
      if (pause)
      {
         if (lastReplayTime > 0)
         {
            this.paused = true;
            pauseStartTime = timestampSupplier.getAsLong();
            LogTools.info("Replay paused");
         }
         else
         {
            this.pendingPause = true;
         }
      }
      else
      {
         if (paused)
         {
            totalPausedDuration += timestampSupplier.getAsLong() - pauseStartTime;
            this.paused = false;
            LogTools.info("Replay resumed");
         }
         this.pendingPause = false;
      }
   }

   public void reset()
   {
      if (isReady())
      {
         for (int topic_idx = 0; topic_idx < topicManagers.size(); topic_idx++)
         {
            ReplayTopicManager<?> topicManager = topicManagers.get(topic_idx);
            topicManager.reset();
         }
         firstUpdate = true;
         totalPausedDuration = 0L;
         pauseStartTime = 0L;
         pendingPause = false;
         paused = false;
         lastReplayTime = 0L;
         startTime = 0L;
      }
   }

   public boolean isReady()
   {
      return topicManagers != null;
   }

   public void destroy()
   {
      if (ros2Node != null)
         ros2Node.close();
   }

   public void setReplaySpeed(double replaySpeed)
   {
      if (replaySpeed <= 0.0)
      {
         LogTools.warn("Replay speed must be > 0.0, keeping previous value: {}", this.replaySpeed);
         return;
      }
      this.replaySpeed = replaySpeed;
      LogTools.info("Replay speed set to {}", replaySpeed);
   }

   public double getReplaySpeed()
   {
      return replaySpeed;
   }

   public <T extends ROS2Message<T>> void addReplayMutator(ROS2Topic<T> topic, ObjLongConsumer<T> mutator)
   {
      ReplayTopicManager<?> topicManager = topicManagersMap.get(topic.getName());
      if (topicManager == null)
      {
         LogTools.warn("Could not set replay mutator. Topic not found in loaded log: {}", topic.getName());
         return;
      }
      topicManager.setMutator(mutator);
   }
}
