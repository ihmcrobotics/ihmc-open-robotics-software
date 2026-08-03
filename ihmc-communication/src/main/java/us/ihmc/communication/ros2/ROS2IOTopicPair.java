package us.ihmc.communication.ros2;

import us.ihmc.communication.HumanoidROS2Topic;
import us.ihmc.jros2.ROS2Message;
import us.ihmc.jros2.ROS2Topic;

/**
 * A class to group a bidirectional topic. We used the terminology "command" and "status"
 * as opposed to "input" and "output" to clarify the most common use case of this, where
 * the input is a command that effects a change of the underlying process and the output
 * is really a status to make others aware about what has been most recently commanded.
 */
public class ROS2IOTopicPair<T extends ROS2Message<T>>
{
   private final ROS2Topic<T> commandTopic;
   private final ROS2Topic<T> statusTopic;

   public ROS2IOTopicPair(ROS2Topic<T> baseTopic)
   {
      if (!(baseTopic instanceof HumanoidROS2Topic<?> humanoidBase))
         throw new IllegalArgumentException("ROS2IOTopicPair requires a HumanoidROS2Topic base: " + baseTopic);

      @SuppressWarnings("unchecked")
      HumanoidROS2Topic<T> typedBase = (HumanoidROS2Topic<T>) humanoidBase;
      commandTopic = typedBase.withIOQualifier("command");
      statusTopic = typedBase.withIOQualifier("status");
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
