package us.ihmc.missionControl.systemd;

import com.google.common.collect.Lists;
import mission_control_msgs.msg.dds.SystemServiceStatusMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.missionControl.MissionControlTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.processManagement.ProcessTools;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;

import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import java.util.UUID;
import java.util.function.Consumer;

public class SystemdServiceMonitor implements Consumer<List<String>>
{
   private static final int MAX_LOG_LINE_MESSAGE_LENGTH = 5;

   private final String serviceName;
   private final ROS2Node ros2Node;
   private final IHMCROS2Publisher<SystemServiceStatusMessage> serviceStatusPublisher;
   private final ExceptionHandlingThreadScheduler systemServiceStatusPublisherScheduler;
   private final JournalCtlReader reader;

   public SystemdServiceMonitor(UUID instanceId, String serviceName, ROS2Node ros2Node)
   {
      this.serviceName = serviceName;
      this.ros2Node = ros2Node;
      serviceStatusPublisher = ROS2Tools.createPublisher(ros2Node,
                                                         ROS2Tools.getSystemServiceStatusTopic(instanceId),
                                                         ROS2Tools.getSystemServiceStatusQosProfile());
      systemServiceStatusPublisherScheduler = new ExceptionHandlingThreadScheduler("SystemServiceStatusPublisherScheduler");
      systemServiceStatusPublisherScheduler.schedule(this::publishStatus, 2.0);

      reader = new JournalCtlReader(serviceName, this);
      reader.start();
   }

   public void start()
   {
      ProcessTools.execSimpleCommandSafe("sudo systemctl start " + serviceName);
   }

   public void stop()
   {
      ProcessTools.execSimpleCommandSafe("sudo systemctl stop " + serviceName);
   }

   public void restart()
   {
      ProcessTools.execSimpleCommandSafe("sudo systemctl restart " + serviceName);
   }

   public void kill()
   {
      ProcessTools.execSimpleCommandSafe("sudo systemctl kill " + serviceName);
   }

   public void publishStatus()
   {
      publishStatus(Lists.newArrayList());
   }

   private void publishStatus(List<String> logLines)
   {
      if (logLines.size() > MAX_LOG_LINE_MESSAGE_LENGTH)
      {
         LogTools.error("Cannot publish log lines with length greater than " + MAX_LOG_LINE_MESSAGE_LENGTH);
         return;
      }

      SystemServiceStatusMessage message = new SystemServiceStatusMessage();

      message.setServiceName(serviceName);

      String status = MissionControlTools.getServiceStatus(serviceName);
      message.setStatus(status);

      StringBuilder builder = new StringBuilder();

      for (String logLine : logLines)
         builder.append(logLine + "\n");

      byte[] logBytes = builder.toString().getBytes(StandardCharsets.US_ASCII); // Use ASCII - UTF8 causes issues when publishing over DDS
      message.getLogData().addAll(logBytes);

      serviceStatusPublisher.publish(message);
   }

   public void destroy()
   {
      reader.stop();
      systemServiceStatusPublisherScheduler.shutdown();
      serviceStatusPublisher.destroy();
      ros2Node.destroy();
   }

   @Override
   public void accept(List<String> logLines)
   {
      for (List<String> logLinesSplit : splitLogLines(logLines, MAX_LOG_LINE_MESSAGE_LENGTH))
         publishStatus(logLinesSplit);
   }

   /**
    * Split a string list (of log lines) into multiple lists of max size maxListSize
    */
   private static List<List<String>> splitLogLines(List<String> logLines, int maxListSize)
   {
      List<List<String>> splitEntries = new ArrayList<>();
      for (int i = 0; i < logLines.size(); i = i + maxListSize)
      {
         if (i + maxListSize < logLines.size())
            splitEntries.add(logLines.subList(i, i + maxListSize));
         else
            splitEntries.add(logLines.subList(i, logLines.size()));
      }
      return splitEntries;
   }
}
