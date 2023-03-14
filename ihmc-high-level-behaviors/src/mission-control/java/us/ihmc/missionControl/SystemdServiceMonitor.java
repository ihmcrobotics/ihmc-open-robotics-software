package us.ihmc.missionControl;

import com.google.common.collect.Lists;
import mission_control_msgs.msg.dds.SystemServiceStatusMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

public class SystemdServiceMonitor implements Consumer<List<String>>
{
   private final String serviceName;
   private final JournalCtlReader reader;
   private final ROS2Node ros2Node;
   private final IHMCROS2Publisher<SystemServiceStatusMessage> serviceStatusPublisher;

   private final ExceptionHandlingThreadScheduler systemServiceStatusPublisherScheduler;

   public SystemdServiceMonitor(String serviceName)
   {
      this.serviceName = serviceName;
      reader = new JournalCtlReader(serviceName, this);
      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "mission_control_daemon_service");
      serviceStatusPublisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.SYSTEM_SERVICE_STATUS);
      systemServiceStatusPublisherScheduler = new ExceptionHandlingThreadScheduler("SystemServiceStatusPublisherScheduler");
      systemServiceStatusPublisherScheduler.schedule(this::publishStatus, 1.00);
   }

   public void publishStatus()
   {
      publishStatus(Lists.newArrayList());
   }

   private void publishStatus(List<String> logLines)
   {
      if (logLines.size() > 25)
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
      systemServiceStatusPublisherScheduler.shutdown();
      reader.stop();
      serviceStatusPublisher.destroy();
      ros2Node.destroy();
   }

   @Override
   public void accept(List<String> logLines)
   {
      for (List<String> logLinesSplit : splitLogLines(logLines, 25))
         publishStatus(logLinesSplit);
   }

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
