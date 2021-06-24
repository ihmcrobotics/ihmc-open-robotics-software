package us.ihmc.avatar.ros2;

import us.ihmc.commons.thread.Notification;
import us.ihmc.ros2.ROS2Topic;

import java.util.function.Consumer;
import java.util.function.Function;

public interface ROS2ControllerPublishSubscribeAPI
{
   public void publishToController(Object message);

   public <T> void publish(Function<String, ROS2Topic<T>> topic, T message);

   public <T> void subscribeViaCallback(Function<String, ROS2Topic<T>> topicFunction, Consumer<T> callback);

   public <T> void subscribeToControllerViaCallback(Class<T> messageClass, Consumer<T> callback);

   public Notification subscribeToWalkingCompletedViaNotification();
}
