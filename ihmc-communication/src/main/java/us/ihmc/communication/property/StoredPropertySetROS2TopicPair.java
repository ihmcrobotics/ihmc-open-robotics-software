package us.ihmc.communication.property;

import ihmc_common_msgs.PrimitiveDataVectorMessage;
import us.ihmc.communication.HumanoidROS2Topic;
import us.ihmc.communication.ros2.ROS2IOTopicPair;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.tools.property.StoredPropertySetReadOnly;
import us.ihmc.tools.string.StringTools;

/**
 * Command corresponds to the operator changing a parameter in the UI.
 * Status is for a periodic output of the remote process's current parameters
 * so the operator can stay informed.
 */
public class StoredPropertySetROS2TopicPair
{
   private final ROS2IOTopicPair<PrimitiveDataVectorMessage> topicPair;

   public StoredPropertySetROS2TopicPair(String moduleTopicName, StoredPropertySetReadOnly storedPropertySetReadOnly)
   {
      this(moduleTopicName, StringTools.titleToSnakeCase(storedPropertySetReadOnly.getTitle()));
   }

   public StoredPropertySetROS2TopicPair(String moduleTopicName, String topicNameSuffix)
   {
      HumanoidROS2Topic<?> baseTopic = HumanoidROS2Topic.IHMC_ROOT.withModule(moduleTopicName);
      ROS2Topic<PrimitiveDataVectorMessage> propertySetTopic = baseTopic.withType(PrimitiveDataVectorMessage.class).withSuffix(topicNameSuffix);
      topicPair = new ROS2IOTopicPair<>(propertySetTopic);
   }

   public ROS2Topic<PrimitiveDataVectorMessage> getCommandTopic()
   {
      return topicPair.getCommandTopic();
   }

   public ROS2Topic<PrimitiveDataVectorMessage> getStatusTopic()
   {
      return topicPair.getStatusTopic();
   }
}
