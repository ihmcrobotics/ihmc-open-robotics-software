package us.ihmc.perception.detections.supervisePose;

import ihmc_common_msgs.Box3DMessage;
import perception_msgs.FoundationPoseParameters;
import perception_msgs.ImageMessage;
import sensor_msgs.CameraInfo;
import sensor_msgs.Image;
import std_msgs.Byte_;
import std_msgs.Empty;
import us.ihmc.jros2.ROS2QoSProfile;
import us.ihmc.jros2.ROS2Topic;
import vision_msgs.Detection3DArray;

public class SupervisePoseAPI
{
   /**
    * Temporary per-object FoundationPose communication topics.
    */
   private static final ROS2Topic<?> SUPERVISE_POSE_TOPIC = new ROS2Topic<>("/foundationpose").withQoS(ROS2QoSProfile.RELIABLE);

   /**
    * Global combined overlay:
    *
    * /ihmc/supervisepose/overlayed_image
    */
   public static final ROS2Topic<ImageMessage> SUPERVISE_POSE_OVERLAY_IMAGE =
         new ROS2Topic<ImageMessage>("/ihmc")

               .appendedWith("supervisepose")
               .appendedWith("overlayed_image")
               .withType(ImageMessage.class)
               .withQoS(ROS2QoSProfile.RELIABLE);

   public static SupervisePoseTopics topics(String category, String instance)
   {
      ROS2Topic<?> foundationPoseBase = SUPERVISE_POSE_TOPIC.appendedWith(category).appendedWith(instance);

      ROS2Topic<ImageMessage> perObjectOverlay =
            new ROS2Topic<ImageMessage>("/ihmc")

                  .appendedWith("supervisepose")
                  .appendedWith(category)
                  .appendedWith(instance)
                  .appendedWith("overlayed_image")
                  .withType(ImageMessage.class)
                  .withQoS(ROS2QoSProfile.RELIABLE);

      return new SupervisePoseTopics(foundationPoseBase, perObjectOverlay);
   }

   public record SupervisePoseTopics(
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

         ROS2Topic<ImageMessage> overlayedImage,

         ROS2Topic<Box3DMessage> ihmcResult,
         ROS2Topic<Byte_> ihmcState,
         ROS2Topic<FoundationPoseParameters> ihmcParameters)
   {
      public SupervisePoseTopics(ROS2Topic<?> base, ROS2Topic<ImageMessage> overlayedImage)
      {
         this(base.appendedWith("pose_estimation/depth_image").withType(Image.class),

              base.appendedWith("pose_estimation/image").withType(Image.class),

              base.appendedWith("pose_estimation/segmentation").withType(Image.class),

              base.appendedWith("pose_estimation/camera_info").withType(CameraInfo.class),

              base.appendedWith("pose_estimation/output").withType(Detection3DArray.class),

              base.appendedWith("tracking/depth_image").withType(Image.class),

              base.appendedWith("tracking/image").withType(Image.class),

              base.appendedWith("tracking/camera_info").withType(CameraInfo.class),

              base.appendedWith("tracking/output").withType(Detection3DArray.class),

              base.appendedWith("depth_image").withType(Image.class),

              base.appendedWith("image").withType(Image.class),

              base.appendedWith("segmentation").withType(Image.class),

              base.appendedWith("camera_info").withType(CameraInfo.class),

              base.appendedWith("reset").withType(Empty.class),

              overlayedImage,

              base.appendedWith("ihmc/result").withType(Box3DMessage.class),

              base.appendedWith("ihmc/state").withType(Byte_.class),

              base.appendedWith("ihmc/parameters").withType(FoundationPoseParameters.class));
      }
   }
}