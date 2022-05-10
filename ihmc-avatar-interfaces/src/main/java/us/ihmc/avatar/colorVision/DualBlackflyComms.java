package us.ihmc.avatar.colorVision;

import std_msgs.msg.dds.Empty;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2Topic;

public class DualBlackflyComms
{
   private static final ROS2Topic<?> BASE_TOPIC = ROS2Tools.IHMC_ROOT.withModule("dual_blackfly");
   public static final ROS2Topic<Empty> RECONNECT_ROS1_NODE = BASE_TOPIC.withType(Empty.class).withSuffix("reconnect_ros1_node");
}
