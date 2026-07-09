package us.ihmc.perception.detections.foundationPose;

import ihmc_common_msgs.msg.dds.Box3DMessage;
import perception_msgs.msg.dds.FoundationPoseParameters;
import sensor_msgs.msg.dds.CameraInfo;
import sensor_msgs.msg.dds.Image;
import std_msgs.msg.dds.Empty;
import us.ihmc.ros2.ROS2Topic;
import vision_msgs.msg.dds.Detection3DArray;

import static us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseAPI.*;

public enum IsaacROSFoundationPoseObject
{
   MUSTARD("Mustard", "mustard", "bottle"),
   TRAFFIC_BARRIER("Traffic Barrier", "traffic_barrier", "traffic_barrier"),
   EXPLOSIVE_CHARGE("Explosive Charge", "charge", "charge"),
   DOOR_PANEL("Door Panel", "door_panel", "door_panel"),
   DOOR_HANDLE("Door Lever", "door_handle", "door_lever"),
   DOOR_LEVER("Door Lever", "door_lever", "door_lever"),
   DOOR_KNOB("Door Knob", "door_knob", "door_knob"),
   DOOR_PULL_HANDLE("Door Pull Handle", "door_pull_handle", "door_pull_handle"); // TODO: Add model

   public static final IsaacROSFoundationPoseObject[] values = values();

   public final String titleCaseName;
   public final String meshDirectory; // Name of mesh directory in robotlabfiles/ihmc-isaac-ros/meshes and .glb in environmentObjects
   public final String yoloClass;
   public final FoundationPoseTopics topics;

   IsaacROSFoundationPoseObject(String titleCaseName, String meshDirectory, String yoloClass)
   {
      this.titleCaseName = titleCaseName;
      this.meshDirectory = meshDirectory;
      this.yoloClass = yoloClass;
      topics = new FoundationPoseTopics(meshDirectory);
   }

   public record FoundationPoseTopics(ROS2Topic<Image> poseEstimationDepthImage, ROS2Topic<Image> poseEstimationRGBImage,
                                      ROS2Topic<Image> poseEstimationSegmentation, ROS2Topic<CameraInfo> poseEstimationCameraInfo,
                                      ROS2Topic<Detection3DArray> poseEstimationOutput, ROS2Topic<Image> trackingDepthImage, ROS2Topic<Image> trackingRGBImage,
                                      ROS2Topic<CameraInfo> trackingCameraInfo, ROS2Topic<Detection3DArray> trackingOutput, ROS2Topic<Image> depthImage,
                                      ROS2Topic<Image> rgbImage, ROS2Topic<Image> segmentation, ROS2Topic<CameraInfo> cameraInfo, ROS2Topic<Empty> reset,
                                      ROS2Topic<Box3DMessage> ihmcResult, ROS2Topic<std_msgs.msg.dds.Byte> ihmcState,
                                      ROS2Topic<FoundationPoseParameters> ihmcParameters)
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
              RESET.withModule(object),
              IHMC_RESULT.withModule(object),
              IHMC_STATE.withModule(object),
              IHMC_PARAMETERS.withModule(object));
      }
   }
}
