package us.ihmc.perception.detections.foundationPose;

import sensor_msgs.msg.dds.CameraInfo;
import sensor_msgs.msg.dds.Image;
import std_msgs.msg.dds.Empty;
import us.ihmc.ros2.ROS2Topic;
import vision_msgs.msg.dds.Detection3DArray;

import static us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseTopics.*;

public enum IsaacROSFoundationPoseObject
{
   MUSTARD("mustard", "bottle"), TRAFFIC_BARRIER("barrier", "traffic_barrier"), EXPLOSIVE_CHARGE("charge", "charge"), DOOR_PANEL("door", "door_panel");

   public final String meshName;
   public final String yoloClass;
   public final FoundationPoseTopics topics;

   IsaacROSFoundationPoseObject(String meshName, String yoloClass)
   {
      this.meshName = meshName;
      this.yoloClass = yoloClass;
      topics = new FoundationPoseTopics(meshName);
   }

   public record FoundationPoseTopics(ROS2Topic<Image> poseEstimationDepthImage, ROS2Topic<Image> poseEstimationRGBImage,
                                      ROS2Topic<Image> poseEstimationSegmentation, ROS2Topic<CameraInfo> poseEstimationCameraInfo,
                                      ROS2Topic<Detection3DArray> poseEstimationOutput, ROS2Topic<Image> trackingDepthImage, ROS2Topic<Image> trackingRGBImage,
                                      ROS2Topic<CameraInfo> trackingCameraInfo, ROS2Topic<Detection3DArray> trackingOutput, ROS2Topic<Image> depthImage,
                                      ROS2Topic<Image> rgbImage, ROS2Topic<Image> segmentation, ROS2Topic<CameraInfo> cameraInfo, ROS2Topic<Empty> reset)
   {
      public FoundationPoseTopics(String object)
      {
         this(POSE_ESTIMATION_DEPTH_IMAGE.withModule(object),
              POSE_ESTIMATION_RGB_IMAGE.withModule(object),
              POSE_ESTIMATION_SEGMENTATION.withModule(object),
              POSE_ESTIMATION_CAMERA_INFO.withModule(object),
              POSE_ESTIMATION_OUTPUT.withModule(object),
              TRACKING_DEPTH_IMAGE.withModule(object),
              TRACKING_RGB_IMAGE.withModule(object),
              TRACKING_CAMERA_INFO.withModule(object),
              TRACKING_OUTPUT.withModule(object),
              DEPTH_IMAGE.withModule(object),
              RGB_IMAGE.withModule(object),
              SEGMENTATION.withModule(object),
              CAMERA_INFO.withModule(object),
              RESET.withModule(object));
      }
   }
}
