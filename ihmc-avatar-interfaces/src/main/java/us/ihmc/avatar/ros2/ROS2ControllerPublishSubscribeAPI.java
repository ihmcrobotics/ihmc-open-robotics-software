package us.ihmc.avatar.ros2;

import us.ihmc.commons.thread.Notification;

import java.util.function.Consumer;

public interface ROS2ControllerPublishSubscribeAPI
{
   public void publishToController(Object message);

   public <T> void subscribeToControllerViaCallback(Class<T> messageClass, Consumer<T> callback);

   public Notification subscribeToWalkingCompletedViaNotification();
}
