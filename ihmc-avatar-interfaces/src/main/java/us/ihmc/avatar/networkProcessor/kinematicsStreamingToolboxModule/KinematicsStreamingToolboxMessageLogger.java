package us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule;

import com.google.common.base.CaseFormat;
import controller_msgs.CapturabilityBasedStatus;
import controller_msgs.CapturabilityBasedStatus;
import controller_msgs.RobotConfigurationData;
import toolbox_msgs.KinematicsStreamingToolboxInputMessage;
import toolbox_msgs.KinematicsToolboxConfigurationMessage;
import toolbox_msgs.KinematicsToolboxOutputStatus;
import toolbox_msgs.ToolboxStateMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.StateEstimatorAPI;
import us.ihmc.communication.serialization.Ros2MessageCdrFileTools;
import us.ihmc.jros2.ROS2Message;
import us.ihmc.log.LogTools;
import us.ihmc.jros2.AsyncROS2Node;
import us.ihmc.tools.thread.CloseableAndDisposable;

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

/**
 * Records KST-related ROS2 traffic to a timestamped JSON log for offline debugging and replay.
 */
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

   private final AsyncROS2Node ros2Node;

   private final AtomicReference<RobotConfigurationData> robotConfigurationData = new AtomicReference<>();
   private final AtomicReference<CapturabilityBasedStatus> capturabilityBasedStatus = new AtomicReference<>();
   private final AtomicReference<KinematicsToolboxConfigurationMessage> kinematicsToolboxConfigurationMessage = new AtomicReference<>();
   private final AtomicReference<KinematicsStreamingToolboxInputMessage> kinematicsStreamingToolboxInputMessage = new AtomicReference<>();
   private final AtomicReference<KinematicsToolboxOutputStatus> kinematicsToolboxOutputStatus = new AtomicReference<>();
   private final AtomicBoolean firstMessage = new AtomicBoolean();
   private final AtomicBoolean stopRequested = new AtomicBoolean();

   private final ScheduledThreadPoolExecutor executorService = new ScheduledThreadPoolExecutor(1);

   private long startTimeMillis;
   private FileOutputStream outputStream = null;
   private PrintStream printStream = null;
   private Runnable loggerRunnable = null;
   private ScheduledFuture<?> loggerTaskScheduled = null;

   public KinematicsStreamingToolboxMessageLogger(String robotName)
   {
      this.robotName = robotName;
      ros2Node = new AsyncROS2Node("ihmc_" + CaseFormat.UPPER_CAMEL.to(CaseFormat.LOWER_UNDERSCORE, "KinematicsStreamingToolboxMessageLogger"));

      ros2Node.createSubscription(StateEstimatorAPI.getRobotConfigurationDataTopic(robotName),
                                  reader -> robotConfigurationData.set(reader.read()));
      ros2Node.createSubscription(HumanoidControllerAPI.getTopic(CapturabilityBasedStatus.class, robotName),
                                  reader -> capturabilityBasedStatus.set(reader.read()));

      ros2Node.createSubscription(KinematicsStreamingToolboxModule.getInputTopic(robotName).withType(ToolboxStateMessage.class),
                                  reader -> processToolboxStateMessage(reader.read()));
      ros2Node.createSubscription(KinematicsStreamingToolboxModule.getInputTopic(robotName).withType(KinematicsToolboxConfigurationMessage.class),
                                  reader -> kinematicsToolboxConfigurationMessage.set(reader.read()));
      ros2Node.createSubscription(KinematicsStreamingToolboxModule.getInputTopic(robotName).withType(KinematicsStreamingToolboxInputMessage.class),
                                  reader -> kinematicsStreamingToolboxInputMessage.set(reader.read()));

      ros2Node.createSubscription(KinematicsStreamingToolboxModule.getOutputTopic(robotName).withType(KinematicsToolboxOutputStatus.class),
                                  reader -> kinematicsToolboxOutputStatus.set(reader.read()));

      ros2Node.spin();
   }

   private void processToolboxStateMessage(ToolboxStateMessage message)
   {
      if (message == null)
         return;

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
         writeIfPresent(robotConfigurationData, robotConfigurationDataName, printStream);
         writeIfPresent(capturabilityBasedStatus, capturabilityBasedStatusName, printStream);
         writeIfPresent(kinematicsToolboxConfigurationMessage, kinematicsToolboxConfigurationMessageName, printStream);
         writeIfPresent(kinematicsStreamingToolboxInputMessage, kinematicsStreamingToolboxInputMessageName, printStream);
         writeIfPresent(kinematicsToolboxOutputStatus, kinematicsToolboxOutputStatusName, printStream);
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

   private static <T extends ROS2Message<T>> void writeIfPresent(AtomicReference<T> messageReference, String messageName, PrintStream printStream)
         throws IOException
   {
      T message = messageReference.getAndSet(null);
      if (message == null)
         return;

      printStream.println(",");
      printStream.println("\"" + messageName + "\" : \"" + Base64.getEncoder().encodeToString(Ros2MessageCdrFileTools.serializeToBytes(message)) + "\"");
   }

   @Override
   public void closeAndDispose()
   {
      shutdown();
      ros2Node.close();
      executorService.shutdownNow();
   }

   public static void main(String[] args)
   {
      String robotName = "Valkyrie"; // "Atlas"; //

      new KinematicsStreamingToolboxMessageLogger(robotName);
   }
}
