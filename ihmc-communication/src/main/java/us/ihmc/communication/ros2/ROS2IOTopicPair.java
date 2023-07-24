package us.ihmc.communication.ros2;

import us.ihmc.ros2.ROS2Topic;

/**
 * A class to group a bidirectional topic. We used the terminology "command" and "status"
 * as opposed to "input" and "output" to clarify the most common use case of this, where
 * the input is a command that effects a change of the underlying process and the output
 * is really a status to make others aware about what has been most recently commanded.
 */
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

   public ROS2Topic<T> getTopic(ROS2IOTopicQualifier ioQualifier)
   {
      return switch (ioQualifier)
      {
         case COMMAND -> commandTopic;
         case STATUS -> statusTopic;
      };
   }
}
