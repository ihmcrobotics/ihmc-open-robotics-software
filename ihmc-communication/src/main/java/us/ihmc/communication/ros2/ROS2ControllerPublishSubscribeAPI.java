package us.ihmc.communication.ros2;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.ros2.ROS2Topic;

import java.util.function.Consumer;
import java.util.function.Function;

public interface ROS2ControllerPublishSubscribeAPI extends ROS2PublishSubscribeAPI
{
   public void publishToController(Object message);

   public <T> void publish(Function<String, ROS2Topic<T>> topic, T message);

   public <T> void subscribeViaCallback(Function<String, ROS2Topic<T>> topicFunction, Consumer<T> callback);

   public <T> IHMCROS2Input<T> subscribeToController(Class<T> messageClass);

   public IHMCROS2Input<RobotConfigurationData> subscribeToRobotConfigurationData();

   public <T> void subscribeToControllerViaCallback(Class<T> messageClass, Consumer<T> callback);

   public Notification subscribeToWalkingCompletedViaNotification();

   public String getRobotName();
}
