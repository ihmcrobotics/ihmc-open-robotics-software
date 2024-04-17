package us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule;

import com.google.common.base.CaseFormat;
import controller_msgs.msg.dds.*;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.RobotConfigurationDataPubSubType;
import toolbox_msgs.msg.dds.ExternalForceEstimationConfigurationMessage;
import toolbox_msgs.msg.dds.ExternalForceEstimationConfigurationMessagePubSubType;
import toolbox_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;

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

public class ExternalForceEstimationMessageLogger
{
   private static final PubSubImplementation pubSubImplementation = PubSubImplementation.FAST_RTPS;
   private static final long recordPeriodMillis = 10;
   private static final double maximumRecordTimeSeconds = 120.0;
   private static final SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
   private static final String logDirectory = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator;

   private final String robotName;

   static final String timestampName = "Timestamp";
   static final String robotConfigurationDataName = RobotConfigurationData.class.getSimpleName();
   static final String robotDesiredConfigurationDataName = RobotDesiredConfigurationData.class.getSimpleName();
   static final String externalForceEstimationConfigName = ExternalForceEstimationConfigurationMessage.class.getSimpleName();

   private final RealtimeROS2Node ros2Node;
   private final AtomicBoolean firstMessage = new AtomicBoolean();
   private final AtomicBoolean stopRequested = new AtomicBoolean();

   private final AtomicReference<RobotConfigurationData> robotConfigurationData = new AtomicReference<>();
   private final AtomicReference<RobotDesiredConfigurationData> robotDesiredConfigurationData = new AtomicReference<>();
   private final AtomicReference<ExternalForceEstimationConfigurationMessage> externalForceEstimationConfigurationMessage = new AtomicReference<>();

   private final JSONSerializer<RobotConfigurationData> robotConfigurationDataSerializer = new JSONSerializer<>(new RobotConfigurationDataPubSubType());
   private final JSONSerializer<RobotDesiredConfigurationData> robotDesiredConfigurationDataSerializer = new JSONSerializer<>(new RobotDesiredConfigurationDataPubSubType());
   private final JSONSerializer<ExternalForceEstimationConfigurationMessage> externalForceEstimationConfigurationSerializer = new JSONSerializer<>(new ExternalForceEstimationConfigurationMessagePubSubType());

   private final ScheduledThreadPoolExecutor executorService = new ScheduledThreadPoolExecutor(1);

   private long startTimeMillis;
   private FileOutputStream outputStream = null;
   private PrintStream printStream = null;
   private Runnable loggerRunnable = null;
   private ScheduledFuture<?> loggerTaskScheduled = null;

   public ExternalForceEstimationMessageLogger(String robotName)
   {
      this.robotName = robotName;
      ros2Node = ROS2Tools.createRealtimeROS2Node(pubSubImplementation,
                                                  "ihmc_" + CaseFormat.UPPER_CAMEL.to(CaseFormat.LOWER_UNDERSCORE, "ExternalForceEstimationMessageLogger"));

      ROS2Topic<?> controllerOutputTopic = HumanoidControllerAPI.getOutputTopic(robotName);
      ros2Node.createSubscription(controllerOutputTopic.withTypeName(RobotConfigurationData.class),
                                  s -> robotConfigurationData.set(s.takeNextData()));
      ros2Node.createSubscription(controllerOutputTopic.withTypeName(RobotDesiredConfigurationData.class),
                                  s -> robotDesiredConfigurationData.set(s.takeNextData()));

      ROS2Topic<?> toolboxInputTopic = ExternalForceEstimationToolboxModule.getInputTopic(robotName);
      ros2Node.createSubscription(toolboxInputTopic.withTypeName(ToolboxStateMessage.class),
                                  s1 -> processToolboxStateMessage(s1.takeNextData()));
      ros2Node.createSubscription(toolboxInputTopic.withTypeName(ExternalForceEstimationConfigurationMessage.class),
                                  s -> externalForceEstimationConfigurationMessage.set(s.takeNextData()));

      ros2Node.spin();
   }

   private void processToolboxStateMessage(ToolboxStateMessage message)
   {
      boolean loggingRequested = message.getRequestLogging();
      boolean sleepRequested = message.getRequestedToolboxState() == ToolboxStateMessage.SLEEP;

      if (!sleepRequested && loggingRequested)
      {
         startLogging();
      }
      else
      {
         stopLogging();
      }
   }

   public void startLogging()
   {
      LogTools.info("Starting logger...");

      if (loggerRunnable != null)
         return;

      String fileName = logDirectory + dateFormat.format(new Date()) + "_" + robotName + "ExternalForceEstimationToolbox.json";
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
      if (stopRequested.get() || System.currentTimeMillis() - startTimeMillis > Conversions.secondsToMilliseconds(maximumRecordTimeSeconds))
         closeLog();

      if (!containsNewMessage())
         return;

      if (!firstMessage.get())
         printStream.println("},");

      printStream.println("{");
      printStream.print("\"" + timestampName + "\" : " + System.nanoTime());

      try
      {
         writeIfPresent(robotConfigurationData, robotConfigurationDataName, robotConfigurationDataSerializer, printStream);
         writeIfPresent(robotDesiredConfigurationData, robotDesiredConfigurationDataName, robotDesiredConfigurationDataSerializer, printStream);
         writeIfPresent(externalForceEstimationConfigurationMessage, externalForceEstimationConfigName, externalForceEstimationConfigurationSerializer, printStream);
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
      return robotConfigurationData.get() != null || robotDesiredConfigurationData.get() != null || externalForceEstimationConfigurationMessage.get() != null;
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
      loggerTaskScheduled.cancel(true);

      loggerTaskScheduled = null;
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

   public static void main(String[] args)
   {
      String robotName = "Valkyrie"; // "Atlas"; //

      new ExternalForceEstimationMessageLogger(robotName);
   }
}
