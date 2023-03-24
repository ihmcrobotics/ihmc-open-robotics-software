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

import java.util.ArrayList;
import java.util.List;
import java.util.UUID;
import java.util.function.Consumer;

public class SystemdServiceMonitor implements Consumer<List<String>>
{
   private static final int MAX_LOG_LINE_MESSAGE_LENGTH = 25;

   private final String serviceName;
   private final ROS2Node ros2Node;
   private final JournalCtlReader reader;
   private final IHMCROS2Publisher<SystemServiceStatusMessage> serviceStatusPublisher;

   private final ExceptionHandlingThreadScheduler systemServiceStatusPublisherScheduler;

   public SystemdServiceMonitor(UUID instanceId, String serviceName, ROS2Node ros2Node)
   {
      this.serviceName = serviceName;
      this.ros2Node = ros2Node;
      reader = new JournalCtlReader(serviceName, this);
      reader.start();
      serviceStatusPublisher = ROS2Tools.createPublisher(ros2Node,
                                                         ROS2Tools.getSystemServiceStatusTopic(instanceId),
                                                         ROS2Tools.getSystemServiceStatusQosProfile());
      systemServiceStatusPublisherScheduler = new ExceptionHandlingThreadScheduler("SystemServiceStatusPublisherScheduler");
      systemServiceStatusPublisherScheduler.schedule(this::publishStatus, 1.0);
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

   public void publishStatus()
   {
      publishStatus(Lists.newArrayList());
   }

   private void publishStatus(List<String> logLines)
   {
      if (logLines.size() > MAX_LOG_LINE_MESSAGE_LENGTH)
      {
         LogTools.error("Cannot publish log lines with length greater than 25");
         return;
      }

      SystemServiceStatusMessage message = new SystemServiceStatusMessage();

      message.setServiceName(serviceName);

      String status = MissionControlTools.getServiceStatus(serviceName);
      message.setStatus(status);

      message.setLogLineCount(logLines.size());

      for (String logLine : logLines)
         message.getLogLines().add(logLine);

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
