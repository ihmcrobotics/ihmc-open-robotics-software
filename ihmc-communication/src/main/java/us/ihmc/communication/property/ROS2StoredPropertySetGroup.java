package us.ihmc.communication.property;

import us.ihmc.jros2.ROS2Node;
import us.ihmc.tools.property.StoredPropertySetBasics;

import java.util.ArrayList;

/**
 * Manages sending and receiving multiple StoredPropertySets over ROS2.
 */
public class ROS2StoredPropertySetGroup
{
   private final ROS2Node ros2Node;
   private final ArrayList<ROS2StoredPropertySet<?>> ros2StoredPropertySets = new ArrayList<>();

   public ROS2StoredPropertySetGroup(ROS2Node ros2Node)
   {
      this.ros2Node = ros2Node;
   }

   /**
    * Registers a StoredPropertySet to be published and received to over ROS2 with the given topic
    *
    * @param topicPair         is the topic name in which the publisher and subscriber should talk to
    * @param storedPropertySet is the parameters that will be sent and received with ROS2
    */
   public void registerStoredPropertySet(StoredPropertySetROS2TopicPair topicPair, StoredPropertySetBasics storedPropertySet)
   {
      ROS2StoredPropertySet<?> ros2StoredPropertySet = new ROS2StoredPropertySet<>(ros2Node, topicPair, storedPropertySet);
      ros2StoredPropertySets.add(ros2StoredPropertySet);
   }

   /**
    * This updates the parameters from the remote, so if parameters were changes this will update that property set here
    */
   public void update()
   {
      for (ROS2StoredPropertySet<?> ros2StoredPropertySet : ros2StoredPropertySets)
      {
         ros2StoredPropertySet.updateAndPublishThrottledStatus();
      }
   }
}
