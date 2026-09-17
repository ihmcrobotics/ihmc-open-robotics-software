package us.ihmc.avatar.ros2;

import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ros2.ROS2PublisherMap;
import us.ihmc.jros2.ROS2Message;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Topic;

import java.util.HashMap;

public class ROS2ControllerPublisherMap
{
   private final String robotName;
   private final ROS2PublisherMap publisherMap;
   private final HashMap<Class<?>, ROS2Topic<?>> topicMap = new HashMap<>();

   public ROS2ControllerPublisherMap(ROS2Node ros2Node, String robotName)
   {
      this(robotName, new ROS2PublisherMap(ros2Node));
   }

   public ROS2ControllerPublisherMap(String robotName, ROS2PublisherMap ros2PublisherMap)
   {
      this.robotName = robotName;
      this.publisherMap = ros2PublisherMap;
   }

   public <T extends ROS2Message<T>> void publish(T message)
   {
      @SuppressWarnings("unchecked")
      Class<T> messageClass = (Class<T>) message.getClass();
      @SuppressWarnings("unchecked")
      ROS2Topic<T> topic = (ROS2Topic<T>) topicMap.get(messageClass);
      if (topic == null)
      {
         topic = HumanoidControllerAPI.getTopic(messageClass, robotName);
         topicMap.put(messageClass, topic);
      }
      publisherMap.publish(topic, message);
   }
}
