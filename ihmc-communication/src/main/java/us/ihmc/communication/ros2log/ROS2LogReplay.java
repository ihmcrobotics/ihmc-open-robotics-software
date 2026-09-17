package us.ihmc.communication.ros2log;

import com.google.common.base.CaseFormat;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.jros2.ROS2Message;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.log.LogTools;

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
      topicManagersMap.clear();
      if (topicManagers == null)
         return;

      for (ReplayTopicManager<?> topicManager : topicManagers)
      {
         topicManagersMap.put(topicManager.getTopicName(), topicManager);
      }
   }

   /**
    * This performs a single update on the current thread, intended for use within the simulation process.
    */
   public boolean doIncrementalReplay()
   {
      if (!isReady())
      {
         LogTools.warn("No ROS 2 log loaded. Call load(...) before replay.");
         return false;
      }

      if (timeSource == ROS2LogTimeSource.SIMULATION && timestampSupplier.getAsLong() == -1)
      {
         return false;
      }
      initializeTimingIfNeeded();

      handlePendingPause();

      if (paused)
      {
         for (ReplayTopicManager<?> topicManager : topicManagers)
         {
            topicManager.updateInternalRepeat(lastReplayTime);
         }
         return false;
      }

      long now = computeReplayTime();
      lastReplayTime = now;
      return updateTopics(now);
   }

   /**
    * This performs a complete replay on the current thread, and returns when the replay is complete.
    */
   public void doFullReplay()
   {
      if (!isReady())
      {
         LogTools.warn("No ROS 2 log loaded. Call load(...) before replay.");
         return;
      }

      if (timeSource == ROS2LogTimeSource.SIMULATION)
      {
         while (timestampSupplier.getAsLong() == -1)
         {
            LogTools.info("Waiting for simulation timestamp");
            ThreadTools.sleep(1000);
         }
      }

      startReplayInternal(topicManagers, timestampSupplier);
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
         for (ReplayTopicManager<?> topicManager : topicManagers)
         {
            isDone &= topicManager.update(now);
         }

         if (isDone)
            break;

         sleepUntilNextReplayMessage(topicManagers, now);
      }
   }

   private void sleepUntilNextReplayMessage(List<ReplayTopicManager<?>> topicManagers, long replayTimeNow)
   {
      long replayDeltaMillis = Long.MAX_VALUE;
      for (ReplayTopicManager<?> topicManager : topicManagers)
      {
         replayDeltaMillis = Math.min(replayDeltaMillis, topicManager.replayTimeUntilNextMessage(replayTimeNow));
      }

      if (replayDeltaMillis == Long.MAX_VALUE || replayDeltaMillis <= 0L)
         return;

      long wallClockSleepMillis = Math.max(1L, (long) Math.floor(replayDeltaMillis / replaySpeed));
      ThreadTools.sleep(Math.min(wallClockSleepMillis, 25L));
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
         for (ReplayTopicManager<?> topicManager : topicManagers)
         {
            topicManager.reset();
         }
         resetPlaybackState();
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

   /**
    * 0-based index of the latest message sent on any loaded topic, or {@code -1} if none yet.
    */
   public int getCurrentStepIndex()
   {
      if (!isReady())
         return -1;

      int index = -1;
      for (ReplayTopicManager<?> topicManager : topicManagers)
         index = Math.max(index, topicManager.getLastSentIndex());
      return index;
   }

   /**
    * Number of logged messages on the densest topic (typical UI "total steps").
    */
   public int getNumberOfSteps()
   {
      if (!isReady())
         return 0;

      int count = 0;
      for (ReplayTopicManager<?> topicManager : topicManagers)
         count = Math.max(count, topicManager.getMessageCount());
      return count;
   }

   /**
    * Current replay clock in nanoseconds (paused time excluded, speed applied), or {@code 0} before start.
    */
   public long getCurrentReplayTimeNanos()
   {
      return lastReplayTime;
   }

   /**
    * Duration of the loaded log in nanoseconds (max end timestamp across topics).
    */
   public long getTotalDurationNanos()
   {
      if (!isReady())
         return 0L;

      long duration = 0L;
      for (ReplayTopicManager<?> topicManager : topicManagers)
         duration = Math.max(duration, topicManager.getEndTimestamp());
      return duration;
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

   private void initializeTimingIfNeeded()
   {
      if (!firstUpdate)
         return;

      startTime = timestampSupplier.getAsLong();
      totalPausedDuration = 0L;
      firstUpdate = false;
   }

   private void handlePendingPause()
   {
      if (!pendingPause || lastReplayTime <= 0)
         return;

      paused = true;
      pauseStartTime = timestampSupplier.getAsLong();
      pendingPause = false;
      LogTools.info("Replay paused (pending request fulfilled)");
   }

   private long computeReplayTime()
   {
      long elapsed = timestampSupplier.getAsLong() - startTime - totalPausedDuration;
      return (long) (elapsed * replaySpeed);
   }

   private boolean updateTopics(long replayTime)
   {
      boolean isDone = true;
      for (ReplayTopicManager<?> topicManager : topicManagers)
      {
         isDone &= topicManager.update(replayTime);
      }
      return isDone;
   }

   private void resetPlaybackState()
   {
      firstUpdate = true;
      totalPausedDuration = 0L;
      pauseStartTime = 0L;
      pendingPause = false;
      paused = false;
      lastReplayTime = 0L;
      startTime = 0L;
   }
}
