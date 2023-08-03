package us.ihmc.communication.property;

import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.tools.property.StoredPropertySetBasics;

import java.util.ArrayList;

/**
 * Helps manage multiple ROS 2 stored property sets.
 */
public class ROS2StoredPropertySetGroup
{
   private final ROS2PublishSubscribeAPI ros2PublishSubscribeAPI;
   private final ArrayList<ROS2StoredPropertySet<?>> ros2StoredPropertySets = new ArrayList<>();

   public ROS2StoredPropertySetGroup(ROS2PublishSubscribeAPI ros2PublishSubscribeAPI)
   {
      this.ros2PublishSubscribeAPI = ros2PublishSubscribeAPI;
   }

   public ROS2StoredPropertySet<?> registerStoredPropertySet(StoredPropertySetROS2TopicPair topicPair, StoredPropertySetBasics storedPropertySet)
   {
      ROS2StoredPropertySet<?> ros2StoredPropertySet = new ROS2StoredPropertySet<>(ros2PublishSubscribeAPI, topicPair, storedPropertySet);
      ros2StoredPropertySets.add(ros2StoredPropertySet);
      return ros2StoredPropertySet;
   }

   public void update()
   {
      for (ROS2StoredPropertySet<?> ros2StoredPropertySet : ros2StoredPropertySets)
      {
         ros2StoredPropertySet.updateAndPublishThrottledStatus();
      }
   }
}
