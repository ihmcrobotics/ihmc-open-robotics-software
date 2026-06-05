package us.ihmc.communication.ros2;

import std_msgs.Bool;
import std_msgs.Empty;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.jros2.ROS2Message;
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

   public <T extends ROS2Message<T>> ROS2Publisher getOrCreatePublisher(ROS2Topic<T> topic)
   {
      ROS2Publisher publisher = map.get(topic);
      if (publisher == null)
      {
         publisher = ros2Node.createPublisher(topic, ROS2Tools.getTopicQoS(topic));
         map.put(topic, publisher);
      }

      return publisher;
   }

   public <T extends ROS2Message<T>> void publish(ROS2Topic<T> topic, T message)
   {
      getOrCreatePublisher(topic).publish(message);
   }

   // Pose3D publishing requires a custom ROS2Message wrapper (see jros2 examples/custom-message-class).

   public void publish(ROS2Topic<Empty> topic)
   {
      getOrCreatePublisher(topic).publish(emptyMessage);
   }

   public void publish(ROS2Topic<Bool> topic, boolean message)
   {
      getOrCreatePublisher(topic).publish(MessageTools.createBoolMessage(message));
   }
}
