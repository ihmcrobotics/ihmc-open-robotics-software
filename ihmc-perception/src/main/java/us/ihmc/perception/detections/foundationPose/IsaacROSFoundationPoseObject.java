package us.ihmc.perception.detections.foundationPose;

import ihmc_common_msgs.Box3DMessage;
import perception_msgs.FoundationPoseParameters;
import sensor_msgs.CameraInfo;
import sensor_msgs.Image;
import std_msgs.Empty;
import us.ihmc.jros2.ROS2Topic;
import vision_msgs.Detection3DArray;

import static us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseAPI.*;

public enum IsaacROSFoundationPoseObject
{
   MUSTARD("Mustard", "mustard", "bottle"),
   TRAFFIC_BARRIER("Traffic Barrier", "traffic_barrier", "traffic_barrier"),
   EXPLOSIVE_CHARGE("Explosive Charge", "charge", "charge"),
   DOOR_PANEL("Door Panel", "door_panel", "door_panel"),
   DOOR_HANDLE("Door Lever", "door_handle", "door_lever"),
   DOOR_LEVER_WAVY("Door Lever Wavy", "door_lever_wavy", "door_lever"),
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
                                      ROS2Topic<Box3DMessage> ihmcResult, ROS2Topic<std_msgs.Byte> ihmcState,
                                      ROS2Topic<FoundationPoseParameters> ihmcParameters)
   {
      public FoundationPoseTopics(String object)
      {
         this(POSE_ESTIMATION_DEPTH_IMAGE.appendedWith(object),
              POSE_ESTIMATION_RGB_IMAGE.appendedWith(object),
              POSE_ESTIMATION_SEGMENTATION.appendedWith(object),
              POSE_ESTIMATION_CAMERA_INFO.appendedWith(object),
              POSE_ESTIMATION_OUTPUT.appendedWith(object),
              TRACKING_DEPTH_IMAGE.appendedWith(object),
              TRACKING_RGB_IMAGE.appendedWith(object),
              TRACKING_CAMERA_INFO.appendedWith(object),
              TRACKING_OUTPUT.appendedWith(object),
              DEPTH_IMAGE.appendedWith(object),
              RGB_IMAGE.appendedWith(object),
              SEGMENTATION.appendedWith(object),
              CAMERA_INFO.appendedWith(object),
              RESET.appendedWith(object),
              IHMC_RESULT.appendedWith(object),
              IHMC_STATE.appendedWith(object),
              IHMC_PARAMETERS.appendedWith(object));
      }
   }
}
