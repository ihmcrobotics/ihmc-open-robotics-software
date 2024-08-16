package us.ihmc.avatar.ros2;

import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ros2.ROS2PublisherMap;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.ROS2NodeInterface;

import java.util.HashMap;

public class ROS2ControllerPublisherMap
{
   private final String robotName;
   private final ROS2PublisherMap publisherMap;
   private final HashMap<Class, ROS2Topic> topicMap = new HashMap<>();

   public ROS2ControllerPublisherMap(ROS2NodeInterface ros2Node, String robotName)
   {
      this(robotName, new ROS2PublisherMap(ros2Node));
   }

   public ROS2ControllerPublisherMap(String robotName, ROS2PublisherMap ros2PublisherMap)
   {
      this.robotName = robotName;
      this.publisherMap = ros2PublisherMap;
   }

   public void publish(Object message)
   {
      ROS2Topic topic = topicMap.get(message.getClass());
      if (topic == null)
      {
         topic = HumanoidControllerAPI.getTopic(message.getClass(), robotName);
         topicMap.put(message.getClass(), topic);
      }
      publisherMap.publish(topic, message);
   }
}
