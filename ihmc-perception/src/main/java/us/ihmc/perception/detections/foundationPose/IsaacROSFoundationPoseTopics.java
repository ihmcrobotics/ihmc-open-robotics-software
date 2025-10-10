package us.ihmc.perception.detections.foundationPose;

import sensor_msgs.msg.dds.CameraInfo;
import sensor_msgs.msg.dds.Image;
import std_msgs.msg.dds.Empty;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import vision_msgs.msg.dds.Detection3DArray;

public class IsaacROSFoundationPoseTopics
{
   public static final ROS2Topic<?> FOUNDATION_POSE_TOPIC = new ROS2Topic<>().withPrefix("foundationpose").withQoS(ROS2QosProfile.RELIABLE());
   public static final ROS2Topic<Image> POSE_ESTIMATION_DEPTH_IMAGE = FOUNDATION_POSE_TOPIC.withSuffix("pose_estimation/depth_image").withType(Image.class);
   public static final ROS2Topic<Image> POSE_ESTIMATION_RGB_IMAGE = FOUNDATION_POSE_TOPIC.withSuffix("pose_estimation/image").withType(Image.class);
   public static final ROS2Topic<Image> POSE_ESTIMATION_SEGMENTATION = FOUNDATION_POSE_TOPIC.withSuffix("pose_estimation/segmentation").withType(Image.class);
   public static final ROS2Topic<CameraInfo> POSE_ESTIMATION_CAMERA_INFO = FOUNDATION_POSE_TOPIC.withSuffix("pose_estimation/camera_info")
                                                                                                 .withType(CameraInfo.class);
   public static final ROS2Topic<Detection3DArray> POSE_ESTIMATION_OUTPUT = FOUNDATION_POSE_TOPIC.withSuffix("pose_estimation/output")
                                                                                                  .withType(Detection3DArray.class);

   public static final ROS2Topic<Image> TRACKING_DEPTH_IMAGE = FOUNDATION_POSE_TOPIC.withSuffix("tracking/depth_image").withType(Image.class);
   public static final ROS2Topic<Image> TRACKING_RGB_IMAGE = FOUNDATION_POSE_TOPIC.withSuffix("tracking/image").withType(Image.class);
   public static final ROS2Topic<CameraInfo> TRACKING_CAMERA_INFO = FOUNDATION_POSE_TOPIC.withSuffix("tracking/camera_info").withType(CameraInfo.class);
   public static final ROS2Topic<Detection3DArray> TRACKING_OUTPUT = FOUNDATION_POSE_TOPIC.withSuffix("tracking/output").withType(Detection3DArray.class);

   public static final ROS2Topic<Image> DEPTH_IMAGE = FOUNDATION_POSE_TOPIC.withSuffix("depth_image").withType(Image.class);
   public static final ROS2Topic<Image> RGB_IMAGE = FOUNDATION_POSE_TOPIC.withSuffix("image").withType(Image.class);
   public static final ROS2Topic<Image> SEGMENTATION = FOUNDATION_POSE_TOPIC.withSuffix("segmentation").withType(Image.class);
   public static final ROS2Topic<CameraInfo> CAMERA_INFO = FOUNDATION_POSE_TOPIC.withSuffix("camera_info").withType(CameraInfo.class);
   public static final ROS2Topic<Empty> RESET = FOUNDATION_POSE_TOPIC.withSuffix("reset").withType(Empty.class);
}
