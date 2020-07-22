package us.ihmc.humanoidBehaviors.tools.ros2;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.Ros2NodeInterface;

import java.util.HashMap;

public class ROS2ControllerPublisherMap
{
   private final String robotName;
   private final ROS2PublisherMap publisherMap;
   private final HashMap<Class, ROS2Topic> topicMap = new HashMap<>();

   public ROS2ControllerPublisherMap(Ros2NodeInterface ros2Node, String robotName)
   {
      this.robotName = robotName;
      publisherMap = new ROS2PublisherMap(ros2Node);
   }

   public void publish(Object message)
   {
      ROS2Topic topic = topicMap.computeIfAbsent(message.getClass(), messageType -> ControllerAPIDefinition.getTopic(messageType, robotName));
      publisherMap.publish(topic, message);
   }
}
