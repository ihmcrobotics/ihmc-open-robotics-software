package us.ihmc.humanoidBehaviors.tools.interfaces;

import us.ihmc.ros2.ROS2Topic;

public interface ROS2TypedPublisherInterface
{
   <T> void publishROS2(ROS2Topic<T> topic, T message);
}
