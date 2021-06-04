package us.ihmc.communication.ros2;

import std_msgs.msg.dds.Empty;
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.ros2.ROS2Topic;

import java.util.function.Consumer;

public interface ROS2PublishSubscribeAPI
{
   public <T> void subscribeViaCallback(ROS2Topic<T> topic, Consumer<T> callback);

   public void subscribeViaCallback(ROS2Topic<Empty> topic, Runnable callback);

   public <T> ROS2Input<T> subscribe(ROS2Topic<T> topic);

   public ROS2TypelessInput subscribeTypeless(ROS2Topic<Empty> topic);

   public Notification subscribeViaNotification(ROS2Topic<Empty> topic);

   public <T> void publish(ROS2Topic<T> topic, T message);

   public void publish(ROS2Topic<Pose3D> topic, Pose3D message);

   public void publish(ROS2Topic<Empty> topic);
}
