package us.ihmc.perception.detections.foundationPose;

import ihmc_common_msgs.msg.dds.Box3DMessage;
import perception_msgs.msg.dds.FoundationPoseParameters;
import sensor_msgs.msg.dds.CameraInfo;
import sensor_msgs.msg.dds.Image;
import std_msgs.msg.dds.Empty;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import vision_msgs.msg.dds.Detection3DArray;

public class CategoryLevelFoundationPoseAPI
{
   static final ROS2Topic<?> FOUNDATION_POSE_TOPIC =
         new ROS2Topic<>().withPrefix("foundationpose").withQoS(ROS2QosProfile.RELIABLE());

   public static CategoryLevelFoundationPoseTopics topics(String category, String instance)
   {
      ROS2Topic<?> base = FOUNDATION_POSE_TOPIC.withModule(category + "/" + instance);
      return new CategoryLevelFoundationPoseTopics(base);
   }

   public record CategoryLevelFoundationPoseTopics(
         ROS2Topic<Image> poseEstimationDepthImage,
         ROS2Topic<Image> poseEstimationRGBImage,
         ROS2Topic<Image> poseEstimationSegmentation,
         ROS2Topic<CameraInfo> poseEstimationCameraInfo,
         ROS2Topic<Detection3DArray> poseEstimationOutput,

         ROS2Topic<Image> trackingDepthImage,
         ROS2Topic<Image> trackingRGBImage,
         ROS2Topic<CameraInfo> trackingCameraInfo,
         ROS2Topic<Detection3DArray> trackingOutput,

         ROS2Topic<Image> depthImage,
         ROS2Topic<Image> rgbImage,
         ROS2Topic<Image> segmentation,
         ROS2Topic<CameraInfo> cameraInfo,
         ROS2Topic<Empty> reset,

         ROS2Topic<Box3DMessage> ihmcResult,
         ROS2Topic<FoundationPoseParameters> ihmcParameters)
   {
      public CategoryLevelFoundationPoseTopics(ROS2Topic<?> base)
      {
         this(base.withSuffix("pose_estimation/depth_image").withType(Image.class),
              base.withSuffix("pose_estimation/image").withType(Image.class),
              base.withSuffix("pose_estimation/segmentation").withType(Image.class),
              base.withSuffix("pose_estimation/camera_info").withType(CameraInfo.class),
              base.withSuffix("pose_estimation/output").withType(Detection3DArray.class),

              base.withSuffix("tracking/depth_image").withType(Image.class),
              base.withSuffix("tracking/image").withType(Image.class),
              base.withSuffix("tracking/camera_info").withType(CameraInfo.class),
              base.withSuffix("tracking/output").withType(Detection3DArray.class),

              base.withSuffix("depth_image").withType(Image.class),
              base.withSuffix("image").withType(Image.class),
              base.withSuffix("segmentation").withType(Image.class),
              base.withSuffix("camera_info").withType(CameraInfo.class),
              base.withSuffix("reset").withType(Empty.class),

              base.withSuffix("ihmc/result").withType(Box3DMessage.class),
              base.withSuffix("ihmc/parameters").withType(FoundationPoseParameters.class));
      }
   }
}
