package us.ihmc.communication;

import controller_msgs.RigidBodyTransformMessage;
import perception_msgs.ArUcoMarkerPoses;
import perception_msgs.ChunkMessage;
import perception_msgs.FramePlanarRegionsListMessage;
import perception_msgs.HeightMapMessage;
import perception_msgs.ImageMessage;
import perception_msgs.PlanarRegionsListMessage;
import perception_msgs.TerrainMapMessage;
import perception_msgs.YOLOv8ExecutorParameters;
import perception_msgs.YOLOv8AnnotationInfoList;
import perception_msgs.ZEDSVOCurrentFileMessage;
import sensor_msgs.CameraInfo;
import sensor_msgs.Image;
import std_msgs.Empty;
import std_msgs.Int64;
import us.ihmc.communication.ros2.ROS2IOTopicPair;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.jros2.ROS2Topic;

/**
 * ROS 2 topics relating to perception
 */
public final class PerceptionAPI
{
   public static final HumanoidROS2Topic<?> IHMC_ROOT = ROS2Tools.IHMC_ROOT;
   public static final HumanoidROS2Topic<?> PERCEPTION_MODULE = IHMC_ROOT.withModule("perception");
   public static final HumanoidROS2Topic<?> HEIGHT_MAP_MODULE = IHMC_ROOT.withModule("height_map");
   public static final HumanoidROS2Topic<?> TERRAIN_MAP_MODULE = IHMC_ROOT.withModule("terrain_map");

   private static final HumanoidROS2Topic<?> BEST_EFFORT = IHMC_ROOT;
   private static final HumanoidROS2Topic<?> RELIABLE = IHMC_ROOT;

   /*
    * ZED SVO
    */
   public static final ROS2Topic<ZEDSVOCurrentFileMessage> ZED_SVO_CURRENT_FILE = PERCEPTION_MODULE.withSuffix("zed_svo_current_file")
                                                                                                   .withType(ZEDSVOCurrentFileMessage.class);
   public static final ROS2Topic<Empty> ZED_SVO_PAUSE = PERCEPTION_MODULE.withSuffix("zed_svo_pause").withType(Empty.class);
   public static final ROS2Topic<Empty> ZED_SVO_PLAY = PERCEPTION_MODULE.withSuffix("zed_svo_play").withType(Empty.class);
   public static final ROS2Topic<Int64> ZED_SVO_SET_POSITION = PERCEPTION_MODULE.withSuffix("zed_svo_set_position").withType(Int64.class);

   /*
    * YOLO
    */
   public static final ROS2Topic<YOLOv8AnnotationInfoList> YOLO_ANNOTATION_INFO = PERCEPTION_MODULE.withModule("yolo")
                                                                                                   .withType(YOLOv8AnnotationInfoList.class)
                                                                                                   .withSuffix("annotation_info_list");
   public static final ROS2Topic<YOLOv8ExecutorParameters> YOLO_PARAMETERS = IHMC_ROOT.withModule("yolo")
                                                                                      .withSuffix("settings")
                                                                                      .withType(YOLOv8ExecutorParameters.class);
   public static final ROS2Topic<Image> YOLO_VLM_ANNOTATED_IMAGE = IHMC_ROOT.withModule("yolo")
                                                                            .withSuffix("vlm/image")
                                                                            .withType(Image.class);
   public static final ROS2Topic<CameraInfo> YOLO_VML_ANNOTATED_IMAGE_CAMERA_INFO = IHMC_ROOT.withModule("yolo")
                                                                                             .withSuffix("vlm/camera_Info")
                                                                                             .withType(CameraInfo.class);

   /*
    * Aruco markers
    */
   public static final ROS2Topic<Empty> REQUEST_ARUCO = PERCEPTION_MODULE.withSuffix("request_aruco").withType(Empty.class);
   public static final ROS2Topic<ArUcoMarkerPoses> ARUCO_MARKER_POSES = PERCEPTION_MODULE.withType(ArUcoMarkerPoses.class).withSuffix("aruco_marker_poses");

   /*
    * Camera topic bases
    */
   private static final HumanoidROS2Topic<?> EXPERIMENTAL_CAMERA = BEST_EFFORT.withPrefix("experimental_camera");
   private static final HumanoidROS2Topic<?> ROS2_EXPERIMENTAL_CAMERA = RELIABLE.withPrefix("ros2_experimental_camera");
   private static final HumanoidROS2Topic<?> STEPPING_CAMERA = BEST_EFFORT.withPrefix("stepping_camera");
   private static final HumanoidROS2Topic<?> ROS2_STEPPING_CAMERA = RELIABLE.withPrefix("ros2_stepping_camera");

   /*
    * Experimental ZED image topics (IHMC ImageMessage type)
    */
   private static final HumanoidROS2Topic<?> EXPERIMENTAL_ZED = EXPERIMENTAL_CAMERA.withModule("zed");
   public static final ROS2Topic<Empty> REQUEST_EXPERIMENTAL_ZED = EXPERIMENTAL_ZED.withSuffix("request").withType(Empty.class);
   public static final ROS2Topic<Empty> REQUEST_EXPERIMENTAL_ZED_PUBLICATION = EXPERIMENTAL_ZED.withSuffix("request_publication").withType(Empty.class);
   public static final ROS2Topic<ImageMessage> EXPERIMENTAL_ZED_DEPTH = EXPERIMENTAL_ZED.withType(ImageMessage.class).withSuffix("depth");
   public static final SideDependentList<ROS2Topic<ImageMessage>> EXPERIMENTAL_ZED_COLOR
         = new SideDependentList<>(EXPERIMENTAL_ZED.withType(ImageMessage.class).withSuffix("left_color"),
                                   EXPERIMENTAL_ZED.withType(ImageMessage.class).withSuffix("right_color"));

   /*
    * Experimental ZED image topics (official ROS 2 Image and CameraInfo types)
    */
   private static final HumanoidROS2Topic<?> ROS2_EXPERIMENTAL_ZED = ROS2_EXPERIMENTAL_CAMERA.withModule("zed");
   public static final SideDependentList<ROS2Topic<Image>> ROS2_EXPERIMENTAL_ZED_COLOR
         = new SideDependentList<>(ROS2_EXPERIMENTAL_ZED.withSuffix("color/left/image").withType(Image.class),
                                   ROS2_EXPERIMENTAL_ZED.withSuffix("color/right/image").withType(Image.class));
   public static final SideDependentList<ROS2Topic<CameraInfo>> ROS2_EXPERIMENTAL_ZED_COLOR_CAMERA_INFO
         = new SideDependentList<>(ROS2_EXPERIMENTAL_ZED.withSuffix("color/left/camera_info").withType(CameraInfo.class),
                                   ROS2_EXPERIMENTAL_ZED.withSuffix("color/right/camera_info").withType(CameraInfo.class));
   public static final ROS2Topic<Image> ROS2_EXPERIMENTAL_ZED_DEPTH = ROS2_EXPERIMENTAL_ZED.withSuffix("depth/image").withType(Image.class);
   public static final ROS2Topic<CameraInfo> ROS2_EXPERIMENTAL_ZED_DEPTH_CAMERA_INFO = ROS2_EXPERIMENTAL_ZED.withSuffix("depth/camera_info")
                                                                                                            .withType(CameraInfo.class);

   /*
    * Stepping ZED image topics  (IHMC ImageMessage type)
    */
   private static final HumanoidROS2Topic<?> STEPPING_ZED = STEPPING_CAMERA.withModule("zed");
   public static final ROS2Topic<Empty> REQUEST_STEPPING_ZED = STEPPING_ZED.withSuffix("request").withType(Empty.class);
   public static final ROS2Topic<Empty> REQUEST_STEPPING_ZED_PUBLICATION = STEPPING_ZED.withSuffix("request_publication").withType(Empty.class);
   public static final ROS2Topic<ImageMessage> STEPPING_ZED_DEPTH = STEPPING_ZED.withType(ImageMessage.class).withSuffix("depth");
   public static final SideDependentList<ROS2Topic<ImageMessage>> STEPPING_ZED_COLOR
         = new SideDependentList<>(STEPPING_ZED.withType(ImageMessage.class).withSuffix("left_color"),
                                   STEPPING_ZED.withType(ImageMessage.class).withSuffix("right_color"));

   /*
    * Stepping ZED image topics (official ROS 2 Image and CameraInfo types)
    */
   private static final HumanoidROS2Topic<?> ROS2_STEPPING_ZED = ROS2_STEPPING_CAMERA.withModule("zed");
   public static final SideDependentList<ROS2Topic<Image>> ROS2_STEPPING_ZED_COLOR
         = new SideDependentList<>(ROS2_STEPPING_ZED.withSuffix("color/left/image").withType(Image.class),
                                   ROS2_STEPPING_ZED.withSuffix("color/right/image").withType(Image.class));
   public static final SideDependentList<ROS2Topic<CameraInfo>> ROS2_STEPPING_ZED_COLOR_CAMERA_INFO
         = new SideDependentList<>(ROS2_STEPPING_ZED.withSuffix("color/left/camera_info").withType(CameraInfo.class),
                                   ROS2_STEPPING_ZED.withSuffix("color/right/camera_info").withType(CameraInfo.class));
   public static final ROS2Topic<Image> ROS2_STEPPING_ZED_DEPTH = ROS2_STEPPING_ZED.withSuffix("depth/image").withType(Image.class);
   public static final ROS2Topic<CameraInfo> ROS2_STEPPING_ZED_DEPTH_CAMERA_INFO = ROS2_STEPPING_ZED.withSuffix("depth/camera_info").withType(CameraInfo.class);

   /*
    * Stepping RealSense image topics (IHMC ImageMessage type)
    */
   private static final HumanoidROS2Topic<?> STEPPING_REALSENSE = STEPPING_CAMERA.withModule("realsense");
   public static final ROS2Topic<Empty> REQUEST_STEPPING_REALSENSE = STEPPING_REALSENSE.withSuffix("request").withType(Empty.class);
   public static final ROS2Topic<Empty> REQUEST_STEPPING_REALSENSE_PUBLICATION = STEPPING_REALSENSE.withSuffix("request_publication").withType(Empty.class);
   public static final ROS2Topic<ImageMessage> STEPPING_REALSENSE_DEPTH = STEPPING_REALSENSE.withType(ImageMessage.class).withSuffix("depth");
   public static final ROS2Topic<ImageMessage> STEPPING_REALSENSE_COLOR = STEPPING_REALSENSE.withType(ImageMessage.class).withSuffix("color");
   public static final ROS2Topic<ImageMessage> STEPPING_REALSENSE_DEPTH_FILTERED = STEPPING_REALSENSE.withType(ImageMessage.class).withSuffix("depth_filtered");

   /*
    * Stepping RealSense image topics (official ROS 2 Image and CameraInfo types)
    */
   private static final HumanoidROS2Topic<?> ROS2_STEPPING_REALSENSE = new HumanoidROS2Topic<>().withPrefix("realsense");
   public static final ROS2Topic<Image> ROS2_STEPPING_REALSENSE_COLOR = ROS2_STEPPING_REALSENSE.withSuffix("color/image").withType(Image.class);
   public static final ROS2Topic<CameraInfo> ROS2_STEPPING_REALSENSE_COLOR_CAMERA_INFO = ROS2_STEPPING_REALSENSE.withSuffix("color/camera_info")
                                                                                                                .withType(CameraInfo.class);
   public static final ROS2Topic<Image> ROS2_STEPPING_REALSENSE_DEPTH = ROS2_STEPPING_REALSENSE.withSuffix("depth/image").withType(Image.class);
   public static final ROS2Topic<CameraInfo> ROS2_STEPPING_REALSENSE_DEPTH_CAMERA_INFO = ROS2_STEPPING_REALSENSE.withSuffix("depth/camera_info")
                                                                                                                .withType(CameraInfo.class);

   /*
    * Planar regions
    */
   public static final ROS2Topic<Empty> REQUEST_PLANAR_REGIONS = IHMC_ROOT.withModule("planar_regions").withSuffix("request").withType(Empty.class);
   public static final ROS2Topic<FramePlanarRegionsListMessage> PERSPECTIVE_RAPID_REGIONS = PERCEPTION_MODULE.withOutput()
                                                                                                             .withTypeName(FramePlanarRegionsListMessage.class)
                                                                                                             .withSuffix("perspective");
   public static final ROS2Topic<FramePlanarRegionsListMessage> SPHERICAL_RAPID_REGIONS_WITH_POSE = PERCEPTION_MODULE.withOutput()
                                                                                                                     .withTypeName(FramePlanarRegionsListMessage.class)
                                                                                                                     .withSuffix("spherical");

   /*
    * Height map
    */
   public static final ROS2Topic<Empty> REQUEST_CHUNK_MAP = PERCEPTION_MODULE.withSuffix("request_chunk_map").withType(Empty.class);
   public static final ROS2Topic<Empty> REQUEST_HEIGHT_MAP = PERCEPTION_MODULE.withSuffix("request_height_map").withType(Empty.class);
   public static final ROS2Topic<Empty> REQUEST_HEIGHT_MAP_FOR_CONTROLLER = PERCEPTION_MODULE.withSuffix("request_height_map_for_controller").withType(Empty.class);
   public static final ROS2Topic<Empty> REQUEST_TERRAIN_MAP = PERCEPTION_MODULE.withSuffix("request_terrain_map").withType(Empty.class);
   public static final ROS2Topic<Empty> REQUEST_YOLO_HEIGHT_MAP = PERCEPTION_MODULE.withSuffix("request_yolo_height_map").withType(Empty.class);
   public static final ROS2Topic<HeightMapMessage> HEIGHT_MAP_MESSAGE = HEIGHT_MAP_MODULE.withOutput().withTypeName(HeightMapMessage.class);
   public static final ROS2Topic<HeightMapMessage> YOLO_HEIGHT_MAP = HEIGHT_MAP_MODULE.withSuffix("yolo").withOutput().withTypeName(HeightMapMessage.class);
   public static final ROS2Topic<ChunkMessage> CHUNK = PERCEPTION_MODULE.withOutput().withTypeName(ChunkMessage.class).withSuffix("chunk");
   public static final ROS2Topic<Empty> RESET_HEIGHT_MAP = PERCEPTION_MODULE.withSuffix("reset_height_map").withType(Empty.class);
   public static final ROS2Topic<Empty> LOWER_HEIGHT_MAP_BACKDROP = PERCEPTION_MODULE.withSuffix("lower_height_map_backdrop").withType(Empty.class);

   /*
    * Terrain map
    */
   public static final ROS2Topic<TerrainMapMessage> TERRAIN_MAP_MESSAGE = TERRAIN_MAP_MODULE.withOutput().withTypeName(TerrainMapMessage.class);
   public static final ROS2Topic<TerrainMapMessage> YOLO_TERRAIN_MAP = TERRAIN_MAP_MODULE.withSuffix("yolo").withOutput().withTypeName(TerrainMapMessage.class);

   /*
    * SLAM (old, not used)
    */
   public static final ROS2Topic<PlanarRegionsListMessage> SLAM_OUTPUT_RAPID_REGIONS = PERCEPTION_MODULE.withOutput()
                                                                                                        .withTypeName(PlanarRegionsListMessage.class)
                                                                                                        .withSuffix("slam_output");

   /*
    * Transform tuning
    */
   private static final HumanoidROS2Topic<RigidBodyTransformMessage> TRANSFORM_TUNING_BASE_TOPIC = IHMC_ROOT.withTypeName(RigidBodyTransformMessage.class)
                                                                                                            .withModule("transform_tuning");
   public static final ROS2IOTopicPair<RigidBodyTransformMessage> STEPPING_CAMERA_TO_PARENT_TUNING = new ROS2IOTopicPair<>(TRANSFORM_TUNING_BASE_TOPIC.withSuffix(
         "stepping_camera_to_parent"));
   public static final ROS2IOTopicPair<RigidBodyTransformMessage> EXPERIMENTAL_CAMERA_TO_PARENT_TUNING = new ROS2IOTopicPair<>(TRANSFORM_TUNING_BASE_TOPIC.withSuffix(
         "experimental_camera_to_parent"));
   public static final SideDependentList<ROS2IOTopicPair<RigidBodyTransformMessage>> SITUATIONAL_AWARENESS_CAMERA_TO_PARENT_TUNING = new SideDependentList<>();

   static
   {
      SITUATIONAL_AWARENESS_CAMERA_TO_PARENT_TUNING.set(RobotSide.LEFT,
                                                        new ROS2IOTopicPair<>(TRANSFORM_TUNING_BASE_TOPIC.withSuffix(
                                                              "situational_awareness_left_camera_to_parent")));
      SITUATIONAL_AWARENESS_CAMERA_TO_PARENT_TUNING.set(RobotSide.RIGHT,
                                                        new ROS2IOTopicPair<>(TRANSFORM_TUNING_BASE_TOPIC.withSuffix(
                                                              "situational_awareness_right_camera_to_parent")));
   }

   /*
    * YOLO
    */
   public static final ROS2Topic<Empty> REQUEST_YOLO = PERCEPTION_MODULE.withSuffix("request_yolo").withType(Empty.class);
   public static final ROS2Topic<Empty> REQUEST_YOLO_ANNOTATED_IMAGE = PERCEPTION_MODULE.withSuffix("request_yolo_image").withType(Empty.class);

   /*
    * Mocap
    */
   // TODO: Pose3D requires custom ROS2Message wrapper implementation (see jros2 examples/custom-message-class)
   // public static final ROS2Topic<Pose3D> MOCAP_RIGID_BODY = IHMC_ROOT.withTypeName(Pose3D.class).withModule("frame_update").withSuffix("mocap");
}
