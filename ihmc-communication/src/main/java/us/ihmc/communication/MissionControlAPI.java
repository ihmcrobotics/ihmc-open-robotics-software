package us.ihmc.communication;

import mission_control_msgs.msg.dds.SystemResourceUsageMessage;
import mission_control_msgs.msg.dds.SystemServiceActionMessage;
import mission_control_msgs.msg.dds.SystemServiceLogRefreshMessage;
import mission_control_msgs.msg.dds.SystemServiceStatusMessage;
import std_msgs.msg.dds.Empty;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;

import java.util.UUID;

public class MissionControlAPI
{
   /**
    * Get system resource usage topic for Mission Control
    * @param instanceId of the Mission Control Daemon
    * @return the ROS2Topic the daemon will use for system resource usage messages
    */
   public static ROS2Topic<SystemResourceUsageMessage> getSystemResourceUsageTopic(UUID instanceId)
   {
      String topicId = instanceId.toString().replace("-", ""); // ROS2 topic names cannot have dashes
      return ROS2Tools.IHMC_ROOT.withModule("mission_control").withSuffix(topicId).withTypeName(SystemResourceUsageMessage.class);
   }

   /**
    * Get system service status topic for Mission Control
    * @param instanceId of the Mission Control Daemon
    * @return the ROS2Topic the daemon will use for system service status messages
    */
   public static ROS2Topic<SystemServiceStatusMessage> getSystemServiceStatusTopic(UUID instanceId)
   {
      String topicId = instanceId.toString().replace("-", ""); // ROS2 topic names cannot have dashes
      return ROS2Tools.IHMC_ROOT.withModule("mission_control").withSuffix(topicId).withQoS(ROS2QosProfile.RELIABLE()).withTypeName(SystemServiceStatusMessage.class);
   }

   public static ROS2Topic<SystemServiceActionMessage> getSystemServiceActionTopic(UUID instanceId)
   {
      String topicId = instanceId.toString().replace("-", ""); // ROS2 topic names cannot have dashes
      return ROS2Tools.IHMC_ROOT.withModule("mission_control").withSuffix(topicId).withTypeName(SystemServiceActionMessage.class);
   }

   public static ROS2Topic<Empty> getSystemRebootTopic(UUID instanceId)
   {
      String topicId = instanceId.toString().replace("-", ""); // ROS2 topic names cannot have dashes
      return ROS2Tools.IHMC_ROOT.withModule("mission_control").withSuffix(topicId).withTypeName(Empty.class);
   }

   public static ROS2Topic<SystemServiceLogRefreshMessage> getSystemServiceLogRefreshTopic(UUID instanceId)
   {
      String topicId = instanceId.toString().replace("-", ""); // ROS2 topic names cannot have dashes
      return ROS2Tools.IHMC_ROOT.withModule("mission_control").withSuffix(topicId).withTypeName(SystemServiceLogRefreshMessage.class);
   }
}
