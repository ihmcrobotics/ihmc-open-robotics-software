package us.ihmc.communication.ros2;

import std_msgs.msg.dds.Empty;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.ROS2NodeInterface;

import java.util.HashMap;

/**
 * Creates publishers automatically. Just lets you publish stuff.
 */
public class ROS2PublisherMap
{
   private final ROS2NodeInterface ros2Node;

   private HashMap<ROS2Topic, IHMCROS2Publisher> map = new HashMap<>();

   public ROS2PublisherMap(ROS2NodeInterface ros2Node)
   {
      this.ros2Node = ros2Node;
   }

   public <T> void publish(ROS2Topic<T> topic, T message)
   {
      IHMCROS2Publisher publisher = map.computeIfAbsent(topic, key -> new IHMCROS2Publisher<>(ros2Node, topic));
      publisher.publish(message);
   }

   public void publish(ROS2Topic<Pose3D> topic, Pose3D message)
   {
      IHMCROS2Publisher publisher = map.computeIfAbsent(topic, key -> IHMCROS2Publisher.newPose3DPublisher(ros2Node, topic));
      publisher.publish(message);
   }

   public void publish(ROS2Topic<Empty> topic)
   {
      Empty message = new Empty();
      IHMCROS2Publisher publisher = map.computeIfAbsent(topic, key -> new IHMCROS2Publisher<>(ros2Node, topic));
      publisher.publish(message);
   }
}
