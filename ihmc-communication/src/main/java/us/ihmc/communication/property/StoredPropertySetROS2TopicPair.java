package us.ihmc.communication.property;

import ihmc_common_msgs.msg.dds.StoredPropertySetMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2IOTopicPair;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.property.StoredPropertySetReadOnly;
import us.ihmc.tools.string.StringTools;

/**
 * Command corresponds to the operator changing a parameter in the UI.
 * Status is for a periodic output of the remote process's current parameters
 * so the operator can stay informed.
 */
public class StoredPropertySetROS2TopicPair
{
   private final ROS2IOTopicPair<StoredPropertySetMessage> topicPair;

   public StoredPropertySetROS2TopicPair(String moduleTopicName, StoredPropertySetReadOnly storedPropertySetReadOnly)
   {
      this(moduleTopicName, StringTools.titleToSnakeCase(storedPropertySetReadOnly.getTitle()));
   }

   public StoredPropertySetROS2TopicPair(String moduleTopicName, String topicNameSuffix)
   {
      ROS2Topic<?> baseTopic = ROS2Tools.IHMC_ROOT.withModule(moduleTopicName);
      ROS2Topic<StoredPropertySetMessage> propertySetTopic = baseTopic.withType(StoredPropertySetMessage.class).withSuffix(topicNameSuffix);
      topicPair = new ROS2IOTopicPair<>(propertySetTopic);
   }

   public ROS2Topic<StoredPropertySetMessage> getCommandTopic()
   {
      return topicPair.getCommandTopic();
   }

   public ROS2Topic<StoredPropertySetMessage> getStatusTopic()
   {
      return topicPair.getStatusTopic();
   }
}
