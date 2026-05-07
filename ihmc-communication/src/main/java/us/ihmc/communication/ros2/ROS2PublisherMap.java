package us.ihmc.communication.ros2;

import std_msgs.Bool;
import std_msgs.Empty;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Topic;

import java.util.HashMap;

/**
 * Creates publishers automatically. Just lets you publish stuff.
 */
public class ROS2PublisherMap
{
   private final ROS2Node ros2Node;
   private final HashMap<ROS2Topic, ROS2Publisher> map = new HashMap<>();
   private final Empty emptyMessage = new Empty();

   public ROS2PublisherMap(ROS2Node ros2Node)
   {
      this.ros2Node = ros2Node;
   }

   public <T> ROS2Publisher getOrCreatePublisher(ROS2Topic<T> topic)
   {
      ROS2Publisher publisher = map.get(topic);
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
