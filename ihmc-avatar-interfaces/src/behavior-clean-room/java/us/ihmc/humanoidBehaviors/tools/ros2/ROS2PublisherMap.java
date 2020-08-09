package us.ihmc.humanoidBehaviors.tools.ros2;

import std_msgs.msg.dds.Empty;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.Ros2NodeInterface;

import java.util.HashMap;

/**
 * Creates publishers automatically. Just lets you publish stuff.
 */
public class ROS2PublisherMap
{
   private final Ros2NodeInterface ros2Node;

   private HashMap<Class, IHMCROS2Publisher> map = new HashMap<>();

   public ROS2PublisherMap(Ros2NodeInterface ros2Node)
   {
      this.ros2Node = ros2Node;
   }

   public <T> void publish(ROS2Topic<T> topic, T message)
   {
      IHMCROS2Publisher publisher = map.computeIfAbsent(message.getClass(), key -> new IHMCROS2Publisher<>(ros2Node, message.getClass(), topic));
      publisher.publish(message);
   }

   public void publish(ROS2Topic<Pose3D> topic, Pose3D message)
   {
      IHMCROS2Publisher publisher = map.computeIfAbsent(Pose3D.class, key -> IHMCROS2Publisher.newPose3DPublisher(ros2Node, topic));
      publisher.publish(message);
   }

   public void publish(ROS2Topic<Empty> topic)
   {
      publish(topic, new Empty());
   }
}
