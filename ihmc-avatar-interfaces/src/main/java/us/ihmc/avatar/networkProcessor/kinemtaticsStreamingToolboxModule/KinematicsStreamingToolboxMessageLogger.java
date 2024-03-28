package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import com.google.common.base.CaseFormat;

import controller_msgs.msg.dds.*;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.RobotConfigurationDataPubSubType;
import toolbox_msgs.msg.dds.*;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.thread.CloseableAndDisposable;

public class KinematicsStreamingToolboxMessageLogger implements CloseableAndDisposable
{
   private static final long recordPeriodMillis = 5;
   private static final double maximumRecordTimeSeconds = 300.0;
   private static final SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
   private static final String logDirectory = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator;

   private final String robotName;

   static final String timestampName = "Timestamp";
   static final String robotConfigurationDataName = RobotConfigurationData.class.getSimpleName();
   static final String capturabilityBasedStatusName = CapturabilityBasedStatus.class.getSimpleName();
   static final String kinematicsToolboxConfigurationMessageName = KinematicsToolboxConfigurationMessage.class.getSimpleName();
   static final String kinematicsStreamingToolboxInputMessageName = KinematicsStreamingToolboxInputMessage.class.getSimpleName();
   static final String kinematicsToolboxOutputStatusName = KinematicsToolboxOutputStatus.class.getSimpleName();

   private final RealtimeROS2Node ros2Node;

   private final AtomicReference<RobotConfigurationData> robotConfigurationData = new AtomicReference<>();
   private final AtomicReference<CapturabilityBasedStatus> capturabilityBasedStatus = new AtomicReference<>();
   private final AtomicReference<KinematicsToolboxConfigurationMessage> kinematicsToolboxConfigurationMessage = new AtomicReference<>();
   private final AtomicReference<KinematicsStreamingToolboxInputMessage> kinematicsStreamingToolboxInputMessage = new AtomicReference<>();
   private final AtomicReference<KinematicsToolboxOutputStatus> kinematicsToolboxOutputStatus = new AtomicReference<>();
   private final AtomicBoolean firstMessage = new AtomicBoolean();
   private final AtomicBoolean stopRequested = new AtomicBoolean();

   private final JSONSerializer<RobotConfigurationData> robotConfigurationDataSerializer = new JSONSerializer<>(new RobotConfigurationDataPubSubType());
   private final JSONSerializer<CapturabilityBasedStatus> capturabilityBasedStatusSerializer = new JSONSerializer<>(new CapturabilityBasedStatusPubSubType());
   private final JSONSerializer<KinematicsToolboxConfigurationMessage> kinematicsToolboxConfigurationMessageSerializer = new JSONSerializer<>(new KinematicsToolboxConfigurationMessagePubSubType());
   private final JSONSerializer<KinematicsStreamingToolboxInputMessage> kinematicsStreamingToolboxInputMessageSerializer = new JSONSerializer<>(new KinematicsStreamingToolboxInputMessagePubSubType());
   private final JSONSerializer<KinematicsToolboxOutputStatus> kinematicsToolboxOutputStatusSerializer = new JSONSerializer<>(new KinematicsToolboxOutputStatusPubSubType());

   private final ScheduledThreadPoolExecutor executorService = new ScheduledThreadPoolExecutor(1);

   private long startTimeMillis;
   private FileOutputStream outputStream = null;
   private PrintStream printStream = null;
   private Runnable loggerRunnable = null;
   private ScheduledFuture<?> loggerTaskScheduled = null;

   public KinematicsStreamingToolboxMessageLogger(String robotName, PubSubImplementation pubSubImplementation)
   {
      this.robotName = robotName;
      ros2Node = ROS2Tools.createRealtimeROS2Node(pubSubImplementation,
                                                  "ihmc_" + CaseFormat.UPPER_CAMEL.to(CaseFormat.LOWER_UNDERSCORE, "KinematicsStreamingToolboxMessageLogger"));

      ROS2Topic controllerOutputTopic = HumanoidControllerAPI.getOutputTopic(robotName);
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, RobotConfigurationData.class, controllerOutputTopic, s -> robotConfigurationData.set(s.takeNextData()));
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    CapturabilityBasedStatus.class,
                                                    controllerOutputTopic,
                                           s -> capturabilityBasedStatus.set(s.takeNextData()));

      ROS2Topic toolboxInputTopic = KinematicsStreamingToolboxModule.getInputTopic(robotName);
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    ToolboxStateMessage.class,
                                                    toolboxInputTopic,
                                           s -> processToolboxStateMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    KinematicsToolboxConfigurationMessage.class,
                                                    toolboxInputTopic,
                                           s -> kinematicsToolboxConfigurationMessage.set(s.takeNextData()));
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    KinematicsStreamingToolboxInputMessage.class,
                                                    toolboxInputTopic,
                                           s -> kinematicsStreamingToolboxInputMessage.set(s.takeNextData()));

      ROS2Topic toolboxOutputTopic = KinematicsStreamingToolboxModule.getOutputTopic(robotName);
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    KinematicsToolboxOutputStatus.class,
                                                    toolboxOutputTopic,
                                           s -> kinematicsToolboxOutputStatus.set(s.takeNextData()));

      ros2Node.spin();
   }

   private void processToolboxStateMessage(ToolboxStateMessage message)
   {
      boolean loggingRequested = message.getRequestLogging();
      boolean sleepRequested = message.getRequestedToolboxState() == ToolboxStateMessage.SLEEP;
      if (!sleepRequested && loggingRequested)
         startLogging();
      else
         stopLogging();
   }

   public void startLogging()
   {
      LogTools.info("Starting logger...");

      if (loggerRunnable != null)
         return;

      String fileName = logDirectory + dateFormat.format(new Date()) + "_" + robotName + "KinematicsStreamingToolbox.json";
      try
      {
         outputStream = new FileOutputStream(fileName);
         printStream = new PrintStream(outputStream);
         loggerRunnable = this::logMessageFrame;
         startTimeMillis = System.currentTimeMillis();

         firstMessage.set(true);
         stopRequested.set(false);

         // start json array
         printStream.println("[");

         loggerTaskScheduled = executorService.scheduleAtFixedRate(loggerRunnable, 0, recordPeriodMillis, TimeUnit.MILLISECONDS);
      }
      catch (IOException e)
      {
         loggerRunnable = null;
         executorService.shutdownNow();

         e.printStackTrace();
      }
   }

   public void stopLogging()
   {
      if (loggerRunnable == null)
         return;

      stopRequested.set(true);
   }

   private void logMessageFrame()
   {
      if (!containsNewMessage())
         return;

      if (stopRequested.get() || System.currentTimeMillis() - startTimeMillis > Conversions.secondsToMilliseconds(maximumRecordTimeSeconds))
         closeLog();

      if (!firstMessage.get())
         printStream.println("},");

      printStream.println("{");
      printStream.print("\"" + timestampName + "\" : " + System.nanoTime());

      try
      {
         writeIfPresent(robotConfigurationData, robotConfigurationDataName, robotConfigurationDataSerializer, printStream);
         writeIfPresent(capturabilityBasedStatus, capturabilityBasedStatusName, capturabilityBasedStatusSerializer, printStream);
         writeIfPresent(kinematicsToolboxConfigurationMessage,
                        kinematicsToolboxConfigurationMessageName,
                        kinematicsToolboxConfigurationMessageSerializer,
                        printStream);
         writeIfPresent(kinematicsStreamingToolboxInputMessage,
                        kinematicsStreamingToolboxInputMessageName,
                        kinematicsStreamingToolboxInputMessageSerializer,
                        printStream);
         writeIfPresent(kinematicsToolboxOutputStatus, kinematicsToolboxOutputStatusName, kinematicsToolboxOutputStatusSerializer, printStream);
      }
      catch (IOException e)
      {
         LogTools.error("Error logging messages. Shutting down logging process");
         shutdown();
         return;
      }

      if (firstMessage.get())
         firstMessage.set(false);
   }

   private boolean containsNewMessage()
   {
      return robotConfigurationData.get() != null || capturabilityBasedStatus.get() != null || kinematicsToolboxConfigurationMessage.get() != null
            || kinematicsStreamingToolboxInputMessage.get() != null;
   }

   private void closeLog()
   {
      LogTools.info("Closing log...");

      printStream.println("}");
      printStream.println("]");

      printStream.flush();
      printStream.close();

      shutdown();
   }

   private void shutdown()
   {
      if (loggerTaskScheduled != null)
      {
         loggerTaskScheduled.cancel(true);
         loggerTaskScheduled = null;
      }
      loggerRunnable = null;
      printStream = null;
      outputStream = null;
   }

   private static <T extends Packet> void writeIfPresent(AtomicReference<T> messageReference, String messageName, JSONSerializer<T> serializer,
                                                         PrintStream printStream)
         throws IOException
   {
      T message = messageReference.getAndSet(null);
      if (message == null)
         return;

      printStream.println(",");
      printStream.println("\"" + messageName + "\" : ");
      printStream.write(serializer.serializeToBytes(message));
   }

   @Override
   public void closeAndDispose()
   {
      shutdown();
      ros2Node.destroy();
      executorService.shutdownNow();
   }

   public static void main(String[] args)
   {
      String robotName = "Valkyrie"; // "Atlas"; //

      new KinematicsStreamingToolboxMessageLogger(robotName, PubSubImplementation.FAST_RTPS);
   }
}
