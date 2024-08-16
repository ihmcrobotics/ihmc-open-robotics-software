package us.ihmc.communication.ros2;

import std_msgs.msg.dds.Bool;
import std_msgs.msg.dds.Empty;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.SwapReference;

import java.util.function.Consumer;

/**
 * TODO: This needs to extend ROS2NodeInterface
 */
public interface ROS2PublishSubscribeAPI
{
   public <T> void subscribeViaCallback(ROS2Topic<T> topic, Consumer<T> callback);

   /**
    * Allocation free callback where the user only has access to the message in the callback.
    * The user should not take up any significat time in the callback to not slow down the ROS 2
    * thread.
    */
   public <T> void subscribeViaVolatileCallback(ROS2Topic<T> topic, Consumer<T> callback);

   /** Use when you only need the latest message and need allocation free. */
   public <T> SwapReference<T> subscribeViaSwapReference(ROS2Topic<T> topic, Notification callback);

   /** Allocation free version with size 16 ring buffer. */
   public <T> ConcurrentRingBuffer<T> subscribeViaQueue(ROS2Topic<T> topic);

   public void subscribeViaCallback(ROS2Topic<Empty> topic, Runnable callback);

   public <T> ROS2Input<T> subscribe(ROS2Topic<T> topic);

   public <T> ROS2Input<T> subscribe(ROS2Topic<T> topic, ROS2Input.MessageFilter<T> messageFilter);

   public ROS2TypelessInput subscribeTypeless(ROS2Topic<Empty> topic);

   public Notification subscribeViaNotification(ROS2Topic<Empty> topic);

   public <T> TypedNotification<T> subscribeViaTypedNotification(ROS2Topic<T> topic);

   public TypedNotification<Boolean> subscribeViaBooleanNotification(ROS2Topic<Bool> topic);

   <T> void createPublisher(ROS2Topic<T> topic);

   public <T> void publish(ROS2Topic<T> topic, T message);

   public void publish(ROS2Topic<std_msgs.msg.dds.String> topic, String message);

   public void publish(ROS2Topic<Pose3D> topic, Pose3D message);

   public void publish(ROS2Topic<Empty> topic);

   public void publish(ROS2Topic<Bool> topic, boolean message);
}
