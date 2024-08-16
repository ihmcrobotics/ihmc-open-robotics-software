package us.ihmc.communication.ros2;

import std_msgs.msg.dds.Bool;
import std_msgs.msg.dds.Empty;
import us.ihmc.ros2.ROS2PublisherBasics;
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
   private final HashMap<ROS2Topic, ROS2PublisherBasics> map = new HashMap<>();
   private final Empty emptyMessage = new Empty();

   public ROS2PublisherMap(ROS2NodeInterface ros2Node)
   {
      this.ros2Node = ros2Node;
   }

   public <T> ROS2PublisherBasics getOrCreatePublisher(ROS2Topic<T> topic)
   {
      ROS2PublisherBasics publisher = map.get(topic);
      if (publisher == null)
      {
         publisher = ros2Node.createPublisher(topic);
         map.put(topic, publisher);
      }

      return publisher;
   }

   public <T> void publish(ROS2Topic<T> topic, T message)
   {
      getOrCreatePublisher(topic).publish(message);
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
