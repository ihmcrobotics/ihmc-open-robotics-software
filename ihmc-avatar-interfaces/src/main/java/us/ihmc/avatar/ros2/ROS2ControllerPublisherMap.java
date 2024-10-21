package us.ihmc.avatar.ros2;

import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.controllerAPI.ControllerAPI;
import us.ihmc.communication.ros2.ROS2PublisherMap;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.ROS2NodeInterface;

import java.util.HashMap;

public class ROS2ControllerPublisherMap
{
   private final ROS2PublisherMap publisherMap;
   private final HashMap<Class, ROS2Topic> topicMap = new HashMap<>();
   private final ROS2Topic<?> controllerInputTopic;

   public ROS2ControllerPublisherMap(ROS2NodeInterface ros2Node, String robotName)
   {
      this(robotName, new ROS2PublisherMap(ros2Node));
   }

   public ROS2ControllerPublisherMap(String robotName, ROS2PublisherMap ros2PublisherMap)
   {
      this.publisherMap = ros2PublisherMap;

      controllerInputTopic = HumanoidControllerAPI.getInputTopic(robotName);
   }

   public void publish(Object message)
   {
      ROS2Topic topic = topicMap.get(message.getClass());
      if (topic == null)
      {
         topic = ControllerAPI.getTopic(controllerInputTopic, message.getClass());
         topicMap.put(message.getClass(), topic);
      }
      publisherMap.publish(topic, message);
   }
}
