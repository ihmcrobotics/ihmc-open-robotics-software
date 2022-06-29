package us.ihmc.avatar.colorVision;

import com.eclipsesource.v8.utils.typedarrays.Float64Array;
import controller_msgs.msg.dds.ArUcoMarkerPoses;
import controller_msgs.msg.dds.StoredPropertySetMessage;

import std_msgs.msg.dds.Float64;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Topic;

import java.util.ArrayList;

public class DualBlackflyComms
{
   private static final ROS2Topic<?> BASE_TOPIC = ROS2Tools.IHMC_ROOT.withModule("dual_blackfly");
   public static final ROS2Topic<Float64> LEFT_PUBLISH_RATE = BASE_TOPIC.withType(Float64.class).withSuffix("left_publish_rate");
   public static final ROS2Topic<Float64> RIGHT_PUBLISH_RATE = BASE_TOPIC.withType(Float64.class).withSuffix("right_publish_rate");
   public static final SideDependentList<ROS2Topic<Float64>> PUBLISH_RATE = new SideDependentList<>(LEFT_PUBLISH_RATE, RIGHT_PUBLISH_RATE);
   public static final ROS2Topic<Float64> GET_IMAGE_DURATION = BASE_TOPIC.withType(Float64.class).withSuffix("right_get_image_duration");
   public static final ROS2Topic<Float64> CONVERT_COLOR_DURATION = BASE_TOPIC.withType(Float64.class).withSuffix("right_convert_color_duration");
   public static final ROS2Topic<Float64> ENCODING_DURATION = BASE_TOPIC.withType(Float64.class).withSuffix("right_encoding_duration");
   public static final ROS2Topic<Float64> COPY_DURATION = BASE_TOPIC.withType(Float64.class).withSuffix("right_copy_duration");



   public static final ROS2Topic<ArUcoMarkerPoses> FRAME_POSE = BASE_TOPIC.withType(ArUcoMarkerPoses.class).withSuffix("frame_pose");

}
