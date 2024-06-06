package us.ihmc.rdx.ui.graphics.ros2;

import us.ihmc.ros2.ROS2Topic;

public interface ROS2TopicHolder<T>
{
   ROS2Topic<T> getTopic();
}
