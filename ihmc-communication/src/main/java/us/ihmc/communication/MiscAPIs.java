package us.ihmc.communication;

import ihmc_common_msgs.msg.dds.TextToSpeechPacket;
import std_msgs.msg.dds.Empty;
import std_msgs.msg.dds.Float64;
import us.ihmc.ros2.ROS2Topic;

public class MiscAPIs
{
   public static final ROS2Topic<Empty> KINEMATICS_SIMULATION_HEARTBEAT
         = ROS2Tools.IHMC_ROOT.withModule("kinematics_simulation").withOutput().withSuffix("heartbeat").withType(Empty.class);
   public static final ROS2Topic<Float64> BOX_MASS = ROS2Tools.IHMC_ROOT.withSuffix("box_mass").withType(Float64.class);
   public static final ROS2Topic<TextToSpeechPacket> TEXT_STATUS = ROS2Tools.IHMC_ROOT.withTypeName(TextToSpeechPacket.class);
}
