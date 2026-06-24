package us.ihmc.perception.detections.foundationPose;

import static us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseAPI.*;

import ihmc_common_msgs.Box3DMessage;
import perception_msgs.FoundationPoseParameters;
import sensor_msgs.CameraInfo;
import sensor_msgs.Image;
import std_msgs.Empty;
import us.ihmc.communication.HumanoidROS2Topic;
import us.ihmc.jros2.ROS2Topic;
import vision_msgs.Detection3DArray;

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
         this(withObjectSuffix(POSE_ESTIMATION_DEPTH_IMAGE, object),
              withObjectSuffix(POSE_ESTIMATION_RGB_IMAGE, object),
              withObjectSuffix(POSE_ESTIMATION_SEGMENTATION, object),
              withObjectSuffix(POSE_ESTIMATION_CAMERA_INFO, object),
              withObjectSuffix(POSE_ESTIMATION_OUTPUT, object),
              withObjectSuffix(TRACKING_DEPTH_IMAGE, object),
              withObjectSuffix(TRACKING_RGB_IMAGE, object),
              withObjectSuffix(TRACKING_CAMERA_INFO, object),
              withObjectSuffix(TRACKING_OUTPUT, object),
              withObjectSuffix(DEPTH_IMAGE, object),
              withObjectSuffix(RGB_IMAGE, object),
              withObjectSuffix(SEGMENTATION, object),
              withObjectSuffix(CAMERA_INFO, object),
              withObjectSuffix(RESET, object),
              withObjectSuffix(IHMC_RESULT, object),
              withObjectSuffix(IHMC_STATE, object),
              withObjectSuffix(IHMC_PARAMETERS, object));
      }
   }

   @SuppressWarnings("unchecked")
   private static <T extends us.ihmc.jros2.ROS2Message<T>> ROS2Topic<T> withObjectSuffix(ROS2Topic<T> baseTopic, String object)
   {
      return ((HumanoidROS2Topic<T>) baseTopic).withSuffix(object);
   }
}
