package us.ihmc.perception.detections.foundationPose;

import ihmc_common_msgs.Box3DMessage;
import perception_msgs.FoundationPoseParameters;
import sensor_msgs.CameraInfo;
import sensor_msgs.Image;
import std_msgs.Byte;
import std_msgs.Empty;
import us.ihmc.jros2.ROS2Topic;
import vision_msgs.Detection3DArray;

public class IsaacROSFoundationPoseAPI
{
   // Base topic
   static final ROS2Topic<?> FOUNDATION_POSE_TOPIC = new ROS2Topic<>().prependedWith("foundationpose");

   // Pose estimation topics
   static final ROS2Topic<Image> POSE_ESTIMATION_DEPTH_IMAGE = FOUNDATION_POSE_TOPIC.appendedWith("pose_estimation/depth_image").withType(Image.class);
   static final ROS2Topic<Image> POSE_ESTIMATION_RGB_IMAGE = FOUNDATION_POSE_TOPIC.appendedWith("pose_estimation/image").withType(Image.class);
   static final ROS2Topic<Image> POSE_ESTIMATION_SEGMENTATION = FOUNDATION_POSE_TOPIC.appendedWith("pose_estimation/segmentation").withType(Image.class);
   static final ROS2Topic<CameraInfo> POSE_ESTIMATION_CAMERA_INFO = FOUNDATION_POSE_TOPIC.appendedWith("pose_estimation/camera_info").withType(CameraInfo.class);
   static final ROS2Topic<Detection3DArray> POSE_ESTIMATION_OUTPUT = FOUNDATION_POSE_TOPIC.appendedWith("pose_estimation/output")
                                                                                          .withType(Detection3DArray.class);

   // Tracking topics
   static final ROS2Topic<Image> TRACKING_DEPTH_IMAGE = FOUNDATION_POSE_TOPIC.appendedWith("tracking/depth_image").withType(Image.class);
   static final ROS2Topic<Image> TRACKING_RGB_IMAGE = FOUNDATION_POSE_TOPIC.appendedWith("tracking/image").withType(Image.class);
   static final ROS2Topic<CameraInfo> TRACKING_CAMERA_INFO = FOUNDATION_POSE_TOPIC.appendedWith("tracking/camera_info").withType(CameraInfo.class);
   static final ROS2Topic<Detection3DArray> TRACKING_OUTPUT = FOUNDATION_POSE_TOPIC.appendedWith("tracking/output").withType(Detection3DArray.class);

   // Selector node topics
   static final ROS2Topic<Image> DEPTH_IMAGE = FOUNDATION_POSE_TOPIC.appendedWith("depth_image").withType(Image.class);
   static final ROS2Topic<Image> RGB_IMAGE = FOUNDATION_POSE_TOPIC.appendedWith("image").withType(Image.class);
   static final ROS2Topic<Image> SEGMENTATION = FOUNDATION_POSE_TOPIC.appendedWith("segmentation").withType(Image.class);
   static final ROS2Topic<CameraInfo> CAMERA_INFO = FOUNDATION_POSE_TOPIC.appendedWith("camera_info").withType(CameraInfo.class);
   static final ROS2Topic<Empty> RESET = FOUNDATION_POSE_TOPIC.appendedWith("reset").withType(Empty.class);

   // IHMC topics
   public static final ROS2Topic<Box3DMessage> IHMC_RESULT = FOUNDATION_POSE_TOPIC.appendedWith("ihmc/result").withType(Box3DMessage.class);
   public static final ROS2Topic<Byte> IHMC_STATE = FOUNDATION_POSE_TOPIC.appendedWith("ihmc/state").withType(Byte.class);
   public static final ROS2Topic<FoundationPoseParameters> IHMC_PARAMETERS = FOUNDATION_POSE_TOPIC.appendedWith("ihmc/parameters")
                                                                                                  .withType(FoundationPoseParameters.class);
}
