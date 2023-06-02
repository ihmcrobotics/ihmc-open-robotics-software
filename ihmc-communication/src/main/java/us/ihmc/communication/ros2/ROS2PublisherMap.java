package us.ihmc.communication.ros2;

import std_msgs.msg.dds.Bool;
import std_msgs.msg.dds.Empty;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.packets.MessageTools;
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
   private final HashMap<ROS2Topic, IHMCROS2Publisher> map = new HashMap<>();
   private final Empty emptyMessage = new Empty();

   public ROS2PublisherMap(ROS2NodeInterface ros2Node)
   {
      this.ros2Node = ros2Node;
   }

   public <T> IHMCROS2Publisher getOrCreatePublisher(ROS2Topic<T> topic)
   {
      IHMCROS2Publisher publisher = map.get(topic);
      if (publisher == null)
      {
         if (topic.getType().equals(Pose3D.class))
         {
            publisher = IHMCROS2Publisher.newPose3DPublisher(ros2Node, topic);
         }
         else
         {
            publisher = new IHMCROS2Publisher<>(ros2Node, topic);
         }
         map.put(topic, publisher);
      }

      return publisher;
   }

   public <T> void publish(ROS2Topic<T> topic, T message)
   {
      IHMCROS2Publisher publisher = getOrCreatePublisher(topic);
      publisher.publish(message);
   }

   public void publish(ROS2Topic<Pose3D> topic, Pose3D message)
   {
      getOrCreatePublisher(topic).publish(message);
   }

   public void publish(ROS2Topic<Empty> topic)
   {
      getOrCreatePublisher(topic).publish(emptyMessage);
   }

   public void publish(ROS2Topic<Bool> topic, boolean message)
   {
      getOrCreatePublisher(topic).publish(MessageTools.createBoolMessage(message));
   }
}
