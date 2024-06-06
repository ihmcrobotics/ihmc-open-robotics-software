package us.ihmc.rdx.ui.graphics.ros2;

import us.ihmc.ros2.ROS2Topic;

import java.util.List;

public interface ROS2MultiTopicHolder<T>
{
   List<ROS2Topic<T>> getTopics();
}
