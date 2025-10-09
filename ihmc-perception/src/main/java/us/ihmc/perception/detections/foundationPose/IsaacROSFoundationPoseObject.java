package us.ihmc.perception.detections.foundationPose;

import sensor_msgs.msg.dds.CameraInfo;
import sensor_msgs.msg.dds.Image;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import vision_msgs.msg.dds.Detection3DArray;

public enum IsaacROSFoundationPoseObject
{
   MUSTARD("mustard", "bottle"), TRAFFIC_BARRIER("barrier", "traffic_barrier"), EXPLOSIVE_CHARGE("charge", "charge"), DOOR_PANEL("door", "door_panel");

   private static final ROS2Topic<?> FOUNDATION_POSE_TOPIC = new ROS2Topic<>().withPrefix("foundationpose").withQoS(ROS2QosProfile.RELIABLE());
   private static final ROS2Topic<Image> POSE_ESTIMATION_DEPTH_IMAGE = FOUNDATION_POSE_TOPIC.withSuffix("pose_estimation/depth_image").withType(Image.class);
   private static final ROS2Topic<Image> POSE_ESTIMATION_RGB_IMAGE = FOUNDATION_POSE_TOPIC.withSuffix("pose_estimation/image").withType(Image.class);
   private static final ROS2Topic<Image> POSE_ESTIMATION_SEGMENTATION = FOUNDATION_POSE_TOPIC.withSuffix("pose_estimation/segmentation").withType(Image.class);
   private static final ROS2Topic<CameraInfo> POSE_ESTIMATION_CAMERA_INFO = FOUNDATION_POSE_TOPIC.withSuffix("pose_estimation/camera_info")
                                                                                                 .withType(CameraInfo.class);
   private static final ROS2Topic<Detection3DArray> POSE_ESTIMATION_OUTPUT = FOUNDATION_POSE_TOPIC.withSuffix("pose_estimation/output")
                                                                                                  .withType(Detection3DArray.class);

   private static final ROS2Topic<Image> TRACKING_DEPTH_IMAGE = FOUNDATION_POSE_TOPIC.withSuffix("tracking/depth_image").withType(Image.class);
   private static final ROS2Topic<Image> TRACKING_RGB_IMAGE = FOUNDATION_POSE_TOPIC.withSuffix("tracking/image").withType(Image.class);
   private static final ROS2Topic<CameraInfo> TRACKING_CAMERA_INFO = FOUNDATION_POSE_TOPIC.withSuffix("tracking/camera_info").withType(CameraInfo.class);
   private static final ROS2Topic<Detection3DArray> TRACKING_OUTPUT = FOUNDATION_POSE_TOPIC.withSuffix("tracking/output").withType(Detection3DArray.class);

   private static final ROS2Topic<Image> DEPTH_IMAGE = FOUNDATION_POSE_TOPIC.withSuffix("depth_image").withType(Image.class);
   private static final ROS2Topic<Image> RGB_IMAGE = FOUNDATION_POSE_TOPIC.withSuffix("image").withType(Image.class);
   private static final ROS2Topic<Image> SEGMENTATION = FOUNDATION_POSE_TOPIC.withSuffix("segmentation").withType(Image.class);
   private static final ROS2Topic<CameraInfo> CAMERA_INFO = FOUNDATION_POSE_TOPIC.withSuffix("camera_info").withType(CameraInfo.class);

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
                                      ROS2Topic<Image> rgbImage, ROS2Topic<Image> segmentation, ROS2Topic<CameraInfo> cameraInfo)
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
              CAMERA_INFO.withModule(object));
      }
   }
}
