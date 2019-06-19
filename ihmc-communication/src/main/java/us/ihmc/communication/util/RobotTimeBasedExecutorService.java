package us.ihmc.communication.util;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.NewMessageListener;
import us.ihmc.ros2.RealtimeRos2Node;

/**
 * When scheduling a task with a desired rate that interacts with a robot simulation it needs to consider the real time
 * rate of the simulator. This class provides the tools to do this both with a simulation and with the real robot.
 * <p>
 * The robot simulation may have a varying real time rate. This class provides the tools to schedule a thread at a
 * desired rate based on robot time. E.g. when the desired rate of the thread is 1Hz but the simulation simulated at
 * 0.5x real time this will adjust the execution of the thread based on that real time rate.
 *
 * @author Georg Wiedebach
 */
public class RobotTimeBasedExecutorService
{
   private static final double alpha = 0.99;

   /**
    * Schedules the provided runnable to be executed periodically based on the timestamp received in a
    * {@link RobotConfigurationData} packet via the provided ROS2 node. This works well in simulation but can cause
    * delay when the ROS2 communication is not local but network based.
    *
    * @param ros2Node to attach a {@link RobotConfigurationData} subscriber to.
    * @param robotName used to create the topic name for the ROS2 node.
    * @param period at which the runnable should be executed in {@link TimeUnit} provided.
    * @param timeUnit of the period.
    * @param runnable will be called at the specified period.
    */
   public static void schedulePackageBased(RealtimeRos2Node ros2Node, String robotName, long period, TimeUnit timeUnit, Runnable runnable)
   {
      long periodInNanos = timeUnit.toNanos(period);
      MessageTopicNameGenerator topicNameGenerator = ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.HUMANOID_CONTROL_MODULE, ROS2TopicQualifier.OUTPUT);
      ROS2Tools.createCallbackSubscription(ros2Node, RobotConfigurationData.class, topicNameGenerator, new NewMessageListener<RobotConfigurationData>()
      {
         private final SampleInfo sampleInfo = new SampleInfo();
         private final RobotConfigurationData robotConfigurationData = new RobotConfigurationData();
         private long previousExecitionTimestamp = Long.MIN_VALUE;
         boolean warningPrinted = false;

         @Override
         public void onNewDataMessage(Subscriber<RobotConfigurationData> subscriber)
         {
            if (!subscriber.takeNextData(robotConfigurationData, sampleInfo))
            {
               LogTools.error("Failed to take " + RobotConfigurationData.class.getSimpleName());
               return;
            }
            long timestamp = robotConfigurationData.getMonotonicTime();
            long timeSinceLastExecution = timestamp - previousExecitionTimestamp;
            boolean firstRun = previousExecitionTimestamp == Long.MIN_VALUE;
            if (timeSinceLastExecution < periodInNanos && !firstRun)
            {
               return;
            }
            if (timeSinceLastExecution != periodInNanos && !firstRun && !warningPrinted)
            {
               warningPrinted = true;
               LogTools.warn("The desired execution rate of " + Conversions.nanosecondsToMilliseconds((double) periodInNanos)
                     + " ms was not achieved (possibly becase is is not divisible by the robot configuration data rate). The achieved rate was "
                     + Conversions.nanosecondsToMilliseconds((double) timeSinceLastExecution) + " ms. (Printing this warning only once)");
            }
            runnable.run();
            previousExecitionTimestamp = timestamp;
         }
      });
   }

   /**
    * Schedules the provided runnable based on the system clock. This should only be used if a real time rate of
    * {@code 1.0} is expected as this does not make any attempt at synchronizing the execution to the robot.
    *
    * @param period at which the runnable should be executed in {@link TimeUnit} provided.
    * @param timeUnit of the period.
    * @param runnable will be called at the specified period.
    */
   public static void scheduleSystemClockBased(long period, TimeUnit timeUnit, Runnable runnable)
   {
      ScheduledExecutorService processScheduler = Executors.newSingleThreadScheduledExecutor();
      ScheduledFuture<?> processFuture = processScheduler.scheduleAtFixedRate(runnable, 0, period, timeUnit);

      // Add a thread that simply waits for the main executor to finish and print out any exceptions that where thrown in the runnable.
      Executors.newSingleThreadExecutor().execute(() -> {
         try
         {
            processFuture.get();
         }
         catch (ExecutionException e)
         {
            e.getCause().printStackTrace();
         }
         catch (InterruptedException e)
         {
            LogTools.error("Runnable was interrupted.");
         }
      });
   }
}
