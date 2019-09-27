package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import com.google.common.base.CaseFormat;
import controller_msgs.msg.dds.*;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.packets.Packet;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.RealtimeRos2Node;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class KinematicsStreamingToolboxMessageLogger
{
   private static final PubSubImplementation pubSubImplementation = PubSubImplementation.FAST_RTPS;
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

   public KinematicsStreamingToolboxMessageLogger(String robotName, RealtimeRos2Node ros2Node)
   {
      this.robotName = robotName;

      MessageTopicNameGenerator controllerPubGenerator = ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      ROS2Tools.createCallbackSubscription(ros2Node, RobotConfigurationData.class, controllerPubGenerator, s -> robotConfigurationData.set(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(ros2Node, CapturabilityBasedStatus.class, controllerPubGenerator, s -> capturabilityBasedStatus.set(s.takeNextData()));

      MessageTopicNameGenerator toolboxSubTopicNameGenerator = KinematicsStreamingToolboxModule.getSubscriberTopicNameGenerator(robotName);
      ROS2Tools.createCallbackSubscription(ros2Node, KinematicsToolboxConfigurationMessage.class, toolboxSubTopicNameGenerator, s -> kinematicsToolboxConfigurationMessage.set(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(ros2Node, KinematicsStreamingToolboxInputMessage.class, toolboxSubTopicNameGenerator, s -> kinematicsStreamingToolboxInputMessage.set(s.takeNextData()));

      MessageTopicNameGenerator toolboxPubTopicNameGenerator = KinematicsStreamingToolboxModule.getPublisherTopicNameGenerator(robotName);
      ROS2Tools.createCallbackSubscription(ros2Node, KinematicsToolboxOutputStatus.class, toolboxPubTopicNameGenerator, s -> kinematicsToolboxOutputStatus.set(s.takeNextData()));
   }

   public void startLogging()
   {
      LogTools.info("Starting logger...");

      if(loggerRunnable != null)
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
      catch(IOException e)
      {
         loggerRunnable = null;
         executorService.shutdownNow();

         e.printStackTrace();
      }
   }

   public void stopLogging()
   {
      if(loggerRunnable == null)
         return;

      stopRequested.set(true);
   }

   private void logMessageFrame()
   {
      if(!containsNewMessage())
         return;

      if(stopRequested.get() || System.currentTimeMillis() - startTimeMillis > Conversions.secondsToMilliseconds(maximumRecordTimeSeconds))
         closeLog();

      if(!firstMessage.get())
         printStream.println("},");

      printStream.println("{");
      printStream.print("\"" + timestampName + "\" : " + System.nanoTime());

      try
      {
         writeIfPresent(robotConfigurationData, robotConfigurationDataName, robotConfigurationDataSerializer, printStream);
         writeIfPresent(capturabilityBasedStatus, capturabilityBasedStatusName, capturabilityBasedStatusSerializer, printStream);
         writeIfPresent(kinematicsToolboxConfigurationMessage, kinematicsToolboxConfigurationMessageName, kinematicsToolboxConfigurationMessageSerializer, printStream);
         writeIfPresent(kinematicsStreamingToolboxInputMessage, kinematicsStreamingToolboxInputMessageName, kinematicsStreamingToolboxInputMessageSerializer, printStream);
         writeIfPresent(kinematicsToolboxOutputStatus, kinematicsToolboxOutputStatusName, kinematicsToolboxOutputStatusSerializer, printStream);
      }
      catch(IOException e)
      {
         LogTools.error("Error logging messages. Shutting down logging process");
         shutdown();
         return;
      }

      if(firstMessage.get())
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
      loggerTaskScheduled.cancel(true);

      loggerTaskScheduled = null;
      loggerRunnable = null;
      printStream = null;
      outputStream = null;
   }

   private static <T extends Packet> void writeIfPresent(AtomicReference<T> messageReference, String messageName, JSONSerializer<T> serializer, PrintStream printStream) throws IOException
   {
      T message = messageReference.getAndSet(null);
      if(message == null)
         return;

      printStream.println(",");
      printStream.println("\"" + messageName + "\" : ");
      printStream.write(serializer.serializeToBytes(message));
   }

   public static void main(String[] args)
   {
      String robotName = "Valkyrie"; // "Atlas"; //
      RealtimeRos2Node ros2Node = ROS2Tools.createRealtimeRos2Node(pubSubImplementation, "ihmc_" + CaseFormat.UPPER_CAMEL.to(CaseFormat.LOWER_UNDERSCORE, "KinematicsStreamingToolboxMessageLogger"));

      KinematicsStreamingToolboxMessageLogger logger = new KinematicsStreamingToolboxMessageLogger(robotName, ros2Node);

      MessageTopicNameGenerator toolboxSubTopicNameGenerator = KinematicsStreamingToolboxModule.getSubscriberTopicNameGenerator(robotName);
      ROS2Tools.createCallbackSubscription(ros2Node, ToolboxStateMessage.class, toolboxSubTopicNameGenerator, s ->
      {
         ToolboxStateMessage toolboxStateMessage = s.takeNextData();
         if(toolboxStateMessage.getRequestedToolboxState() == ToolboxStateMessage.WAKE_UP)
         {
            logger.startLogging();
         }
         else if(toolboxStateMessage.getRequestedToolboxState() == ToolboxStateMessage.SLEEP)
         {
            logger.stopLogging();
         }
      });

      ros2Node.spin();
   }
}
