package us.ihmc.communication;

import std_msgs.Empty;
import system_monitor_msgs.SystemResourceUsageMessage;
import system_monitor_msgs.SystemServiceActionMessage;
import system_monitor_msgs.SystemServiceLogRefreshMessage;
import system_monitor_msgs.SystemServiceStatusMessage;
import us.ihmc.jros2.ROS2Topic;

import java.util.UUID;

public final class SystemMonitorAPI
{
   /**
    * Get system resource usage topic
    * @param instanceId an identifier unique per machine
    * @return the ROS2Topic the daemon will use for system resource usage messages
    */
   public static ROS2Topic<SystemResourceUsageMessage> getSystemResourceUsageTopic(UUID instanceId)
   {
      String topicId = instanceId.toString().replace("-", ""); // ROS2 topic names cannot have dashes
      return HumanoidROS2Topic.IHMC_ROOT.withModule("system_monitor").withSuffix(topicId).withTypeName(SystemResourceUsageMessage.class);
   }

   /**
    * Get system service status topic
    * @param instanceId an identifier unique per machine
    * @return the ROS2Topic the daemon will use for system service status messages
    */
   public static ROS2Topic<SystemServiceStatusMessage> getSystemServiceStatusTopic(UUID instanceId)
   {
      String topicId = instanceId.toString().replace("-", ""); // ROS2 topic names cannot have dashes
      return HumanoidROS2Topic.IHMC_ROOT.withModule("system_monitor").withSuffix(topicId).withTypeName(SystemServiceStatusMessage.class);
   }

   public static ROS2Topic<SystemServiceActionMessage> getSystemServiceActionTopic(UUID instanceId)
   {
      String topicId = instanceId.toString().replace("-", ""); // ROS2 topic names cannot have dashes
      return HumanoidROS2Topic.IHMC_ROOT.withModule("system_monitor").withSuffix(topicId).withTypeName(SystemServiceActionMessage.class);
   }

   public static ROS2Topic<Empty> getSystemRebootTopic(UUID instanceId)
   {
      String topicId = instanceId.toString().replace("-", ""); // ROS2 topic names cannot have dashes
      return HumanoidROS2Topic.IHMC_ROOT.withModule("system_monitor").withSuffix(topicId).withTypeName(Empty.class);
   }

   public static ROS2Topic<SystemServiceLogRefreshMessage> getSystemServiceLogRefreshTopic(UUID instanceId)
   {
      String topicId = instanceId.toString().replace("-", ""); // ROS2 topic names cannot have dashes
      return HumanoidROS2Topic.IHMC_ROOT.withModule("system_monitor").withSuffix(topicId).withTypeName(SystemServiceLogRefreshMessage.class);
   }
}
