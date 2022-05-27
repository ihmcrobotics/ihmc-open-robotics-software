package us.ihmc.avatar.colorVision;

import std_msgs.msg.dds.Empty;
import std_msgs.msg.dds.Float64;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Topic;

public class DualBlackflyComms
{
   private static final ROS2Topic<?> BASE_TOPIC = ROS2Tools.IHMC_ROOT.withModule("dual_blackfly");
   public static final ROS2Topic<Empty> RECONNECT_ROS1_NODE = BASE_TOPIC.withType(Empty.class).withSuffix("reconnect_ros1_node");
   public static final ROS2Topic<Float64> LEFT_PUBLISH_RATE = BASE_TOPIC.withType(Float64.class).withSuffix("left_publish_rate");
   public static final ROS2Topic<Float64> RIGHT_PUBLISH_RATE = BASE_TOPIC.withType(Float64.class).withSuffix("right_publish_rate");
   public static final SideDependentList<ROS2Topic<Float64>> PUBLISH_RATE = new SideDependentList<>(LEFT_PUBLISH_RATE, RIGHT_PUBLISH_RATE);
}
