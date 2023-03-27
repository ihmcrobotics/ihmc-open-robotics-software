package us.ihmc.communication.ros2;

import us.ihmc.ros2.ROS2Topic;

public class ROS2IOTopicPair<T>
{
   private final ROS2Topic<T> commandTopic;
   private final ROS2Topic<T> statusTopic;

   public ROS2IOTopicPair(ROS2Topic<T> baseTopic)
   {
      commandTopic = baseTopic.withIOQualifier("command");
      statusTopic = baseTopic.withIOQualifier("status");
   }

   public ROS2Topic<T> getCommandTopic()
   {
      return commandTopic;
   }

   public ROS2Topic<T> getStatusTopic()
   {
      return statusTopic;
   }
}
