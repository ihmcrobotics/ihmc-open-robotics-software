package us.ihmc.communication.util;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

import com.google.common.util.concurrent.AtomicDouble;

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
import us.ihmc.ros2.Ros2Node;

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
      NewMessageListener<RobotConfigurationData> robotConfigurationDataListener = createListener(period, timeUnit, runnable);
      ROS2Tools.createCallbackSubscription(ros2Node, RobotConfigurationData.class, createTopicNameGenerator(robotName), robotConfigurationDataListener);
   }

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
   public static void schedulePackageBased(Ros2Node ros2Node, String robotName, long period, TimeUnit timeUnit, Runnable runnable)
   {
      NewMessageListener<RobotConfigurationData> robotConfigurationDataListener = createListener(period, timeUnit, runnable);
      ROS2Tools.createCallbackSubscription(ros2Node, RobotConfigurationData.class, createTopicNameGenerator(robotName), robotConfigurationDataListener);
   }

   private static MessageTopicNameGenerator createTopicNameGenerator(String robotName)
   {
      return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.HUMANOID_CONTROL_MODULE, ROS2TopicQualifier.OUTPUT);
   }

   private static NewMessageListener<RobotConfigurationData> createListener(long period, TimeUnit timeUnit, Runnable runnable)
   {
      long periodInNanos = timeUnit.toNanos(period);
      NewMessageListener<RobotConfigurationData> robotConfigurationDataListener = new NewMessageListener<RobotConfigurationData>()
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
      };
      return robotConfigurationDataListener;
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

   /**
    * Note: this is experimental and not tested.
    *
    * Schedules the provided runnable to be executed periodically based on the timestamp received in a
    * {@link RobotConfigurationData} packet via the provided ROS2 node. This attempts at estimating the real time rate
    * of the process sending the {@link RobotConfigurationData} to compensate for intermittent network delay.
    *
    * @param ros2Node to attach a {@link RobotConfigurationData} subscriber to.
    * @param robotName used to create the topic name for the ROS2 node.
    * @param expectedPackagePeriod at which the we expect a {@link RobotConfigurationData} packet.
    * @param packagePeriodTimeUnit of the expectedPackagePeriod.
    * @param period at which the runnable should be executed in {@link TimeUnit} provided.
    * @param timeUnit of the period.
    * @param runnable will be called at the specified period.
    */
   public static void schedulePackageBasedWithDelayCompensation(RealtimeRos2Node ros2Node, String robotName, long expectedPackagePeriod,
                                                                TimeUnit packagePeriodTimeUnit, long period, TimeUnit timeUnit, Runnable runnable)
   {
      long periodInNanos = timeUnit.toNanos(period);
      long expectedPackagePeriodInNanos = packagePeriodTimeUnit.toNanos(expectedPackagePeriod);
      AtomicDouble estimatedRealtimeRate = new AtomicDouble(1.0);

      // Create a thread that estimates the current realtime rate.
      MessageTopicNameGenerator topicNameGenerator = createTopicNameGenerator(robotName);
      ROS2Tools.createCallbackSubscription(ros2Node, RobotConfigurationData.class, topicNameGenerator, new NewMessageListener<RobotConfigurationData>()
      {
         private final SampleInfo sampleInfo = new SampleInfo();
         private final RobotConfigurationData robotConfigurationData = new RobotConfigurationData();
         private long previousPackageTimestamp = Long.MIN_VALUE;
         private long previousPackageArrivalTime = Long.MIN_VALUE;

         @Override
         public void onNewDataMessage(Subscriber<RobotConfigurationData> subscriber)
         {
            long packageArrivalTime = System.nanoTime();
            if (!subscriber.takeNextData(robotConfigurationData, sampleInfo))
            {
               LogTools.error("Failed to take " + RobotConfigurationData.class.getSimpleName());
               return;
            }
            long timestamp = robotConfigurationData.getMonotonicTime();
            long timestampDifference = timestamp - previousPackageTimestamp;
            long timeSinceLastPackage = packageArrivalTime - previousPackageArrivalTime;
            if (timestampDifference != expectedPackagePeriodInNanos)
            {
               previousPackageTimestamp = Long.MIN_VALUE;
               previousPackageArrivalTime = Long.MIN_VALUE;
               LogTools.warn("Missed package or packets out of order.");
            }
            if (previousPackageArrivalTime != Long.MIN_VALUE)
            {
               double current = (double) expectedPackagePeriodInNanos / (double) timeSinceLastPackage;
               double value = estimatedRealtimeRate.get();
               double newValue = alpha * value + (1.0 - alpha) * current;
               estimatedRealtimeRate.set(newValue);
            }
            previousPackageTimestamp = timestamp;
            previousPackageArrivalTime = packageArrivalTime;
         }
      });

      // Create a fast thread that call the runnable based on the system time combines with the estimated realtime rate.
      // Magic number. Run the fast thread faster by this factor to get a decent resolution.
      int divisor = 5;
      scheduleSystemClockBased(periodInNanos / divisor, TimeUnit.NANOSECONDS, new Runnable()
      {
         long previousExecutionTime = Long.MIN_VALUE;

         @Override
         public void run()
         {
            long time = System.nanoTime();
            double realtimeRate = estimatedRealtimeRate.get();
            long timePassed = time - previousExecutionTime;
            boolean firstRun = previousExecutionTime == Long.MIN_VALUE;
            if (realtimeRate * timePassed > periodInNanos || firstRun)
            {
               runnable.run();
               previousExecutionTime = time;
               if (realtimeRate * timePassed > 2.0 * periodInNanos && !firstRun)
               {
                  LogTools.warn("Estimated realtime rate is too high can not keep up.");
               }
            }
         }
      });
   }
}
