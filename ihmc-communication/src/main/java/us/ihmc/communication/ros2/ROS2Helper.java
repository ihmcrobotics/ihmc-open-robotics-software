package us.ihmc.communication.ros2;

import std_msgs.msg.dds.Empty;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.ros2.ROS2Callback;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;

import java.util.function.Consumer;
import java.util.function.Function;

/**
 * Supports:
 * - Publishing on the fly without having to first create publishers
 * - Disabling and enabling all the publishers and subscribers created here
 */
public class ROS2Helper implements ROS2PublishSubscribeAPI
{
   protected final ManagedROS2Node managedROS2Node;
   protected final ROS2PublisherMap ros2PublisherMap;

   public ROS2Helper(ROS2NodeInterface ros2Node)
   {
      managedROS2Node = new ManagedROS2Node(ros2Node);
      ros2PublisherMap = new ROS2PublisherMap(managedROS2Node);
   }

   public void setCommunicationCallbacksEnabled(boolean enabled)
   {
      managedROS2Node.setEnabled(enabled);
   }

   @Override
   public <T> void subscribeViaCallback(ROS2Topic<T> topic, Consumer<T> callback)
   {
      ROS2Tools.createCallbackSubscription2(managedROS2Node, topic, callback);
   }

   @Override
   public void subscribeViaCallback(ROS2Topic<Empty> topic, Runnable callback)
   {
      ROS2Tools.createCallbackSubscription2(managedROS2Node, topic, message -> callback.run());
   }

   @Override
   public <T> ROS2Input<T> subscribe(ROS2Topic<T> topic)
   {
      return new ROS2Input<>(managedROS2Node, topic.getType(), topic);
   }

   @Override
   public ROS2TypelessInput subscribeTypeless(ROS2Topic<Empty> topic)
   {
      return new ROS2TypelessInput(managedROS2Node, topic);
   }

   @Override
   public Notification subscribeViaNotification(ROS2Topic<Empty> topic)
   {
      Notification notification = new Notification();
      new ROS2Callback<>(managedROS2Node, Empty.class, topic, message -> notification.set());
      return notification;
   }

   @Override
   public <T> void publish(ROS2Topic<T> topic, T message)
   {
      ros2PublisherMap.publish(topic, message);
   }

   @Override
   public void publish(ROS2Topic<Pose3D> topic, Pose3D message)
   {
      ros2PublisherMap.publish(topic, message);
   }

   @Override
   public void publish(ROS2Topic<Empty> topic)
   {
      ros2PublisherMap.publish(topic);
   }

   public ROS2NodeInterface getROS2NodeInterface()
   {
      return managedROS2Node;
   }
}
