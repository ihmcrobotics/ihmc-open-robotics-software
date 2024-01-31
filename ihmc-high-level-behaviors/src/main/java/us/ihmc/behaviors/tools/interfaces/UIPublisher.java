package us.ihmc.behaviors.tools.interfaces;

import behavior_msgs.msg.dds.StatusLogMessage;
import us.ihmc.ros2.ROS2Topic;

public interface UIPublisher
{
   void publishToUI(ROS2Topic<StatusLogMessage> topic, StatusLogMessage message);
}
