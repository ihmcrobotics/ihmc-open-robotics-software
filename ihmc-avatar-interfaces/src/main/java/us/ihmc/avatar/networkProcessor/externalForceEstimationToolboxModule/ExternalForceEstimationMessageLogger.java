package us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule;

import com.google.common.base.CaseFormat;
import controller_msgs.RobotConfigurationData;
import controller_msgs.RobotDesiredConfigurationData;
import toolbox_msgs.ExternalForceEstimationConfigurationMessage;
import toolbox_msgs.ToolboxStateMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.serialization.Ros2MessageCdrFileTools;
import us.ihmc.log.LogTools;
import us.ihmc.jros2.ROS2Message;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.jros2.AsyncROS2Node;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.Base64;
import java.util.Date;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class ExternalForceEstimationMessageLogger
{
   private static final long recordPeriodMillis = 10;
   private static final double maximumRecordTimeSeconds = 120.0;
   private static final SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
   private static final String logDirectory = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator;

   private final String robotName;

   static final String timestampName = "Timestamp";
   static final String robotConfigurationDataName = RobotConfigurationData.class.getSimpleName();
   static final String robotDesiredConfigurationDataName = RobotDesiredConfigurationData.class.getSimpleName();
   static final String externalForceEstimationConfigName = ExternalForceEstimationConfigurationMessage.class.getSimpleName();

   private final AsyncROS2Node ros2Node;
   private final AtomicBoolean firstMessage = new AtomicBoolean();
   private final AtomicBoolean stopRequested = new AtomicBoolean();

   private final AtomicReference<RobotConfigurationData> robotConfigurationData = new AtomicReference<>();
   private final AtomicReference<RobotDesiredConfigurationData> robotDesiredConfigurationData = new AtomicReference<>();
   private final AtomicReference<ExternalForceEstimationConfigurationMessage> externalForceEstimationConfigurationMessage = new AtomicReference<>();

   private final ScheduledThreadPoolExecutor executorService = new ScheduledThreadPoolExecutor(1);

   private long startTimeMillis;
   private FileOutputStream outputStream = null;
   private PrintStream printStream = null;
   private Runnable loggerRunnable = null;
   private ScheduledFuture<?> loggerTaskScheduled = null;

   public ExternalForceEstimationMessageLogger(String robotName)
   {
      this.robotName = robotName;
      ros2Node = new AsyncROS2Node("ihmc_" + CaseFormat.UPPER_CAMEL.to(CaseFormat.LOWER_UNDERSCORE, "ExternalForceEstimationMessageLogger"));

      ROS2Topic<?> controllerOutputTopic = HumanoidControllerAPI.getOutputTopic(robotName);
      ros2Node.createSubscription(controllerOutputTopic.withType(RobotConfigurationData.class),
                                  reader -> robotConfigurationData.set(reader.read()));
      ros2Node.createSubscription(controllerOutputTopic.withType(RobotDesiredConfigurationData.class),
                                  reader -> robotDesiredConfigurationData.set(reader.read()));

      ROS2Topic<?> toolboxInputTopic = ExternalForceEstimationToolboxModule.getInputTopic(robotName);
      ros2Node.createSubscription(toolboxInputTopic.withType(ToolboxStateMessage.class),
                                  reader -> processToolboxStateMessage(reader.read()));
      ros2Node.createSubscription(toolboxInputTopic.withType(ExternalForceEstimationConfigurationMessage.class),
                                  reader -> externalForceEstimationConfigurationMessage.set(reader.read()));

      ros2Node.spin();
   }

   private void processToolboxStateMessage(ToolboxStateMessage message)
   {
      if (message == null)
         return;

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
         writeIfPresent(robotConfigurationData, robotConfigurationDataName, printStream);
         writeIfPresent(robotDesiredConfigurationData, robotDesiredConfigurationDataName, printStream);
         writeIfPresent(externalForceEstimationConfigurationMessage, externalForceEstimationConfigName, printStream);
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

   private static <T extends ROS2Message<T>> void writeIfPresent(AtomicReference<T> messageReference, String messageName, PrintStream printStream)
         throws IOException
   {
      T message = messageReference.getAndSet(null);
      if (message == null)
         return;

      printStream.println(",");
      printStream.println("\"" + messageName + "\" : \"" + Base64.getEncoder().encodeToString(Ros2MessageCdrFileTools.serializeToBytes(message)) + "\"");
   }

   public static void main(String[] args)
   {
      String robotName = "Valkyrie"; // "Atlas"; //

      new ExternalForceEstimationMessageLogger(robotName);
   }
}
