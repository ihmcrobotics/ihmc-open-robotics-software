package us.ihmc.communication;

import controller_msgs.msg.dds.RigidBodyTransformMessage;
import perception_msgs.msg.dds.ArUcoMarkerPoses;
import perception_msgs.msg.dds.ChunkMessage;
import perception_msgs.msg.dds.DetectedDoorListMessage;
import perception_msgs.msg.dds.DetectedObjectPacket;
import perception_msgs.msg.dds.FramePlanarRegionsListMessage;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.ImageMessage;
import perception_msgs.msg.dds.IterativeClosestPointRequest;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import perception_msgs.msg.dds.SceneGraphMessage;
import perception_msgs.msg.dds.TerrainMapMessage;
import perception_msgs.msg.dds.YOLOv8ExecutorParameters;
import perception_msgs.msg.dds.ZEDSVOCurrentFileMessage;
import sensor_msgs.msg.dds.CameraInfo;
import sensor_msgs.msg.dds.Image;
import std_msgs.msg.dds.Empty;
import std_msgs.msg.dds.Int64;
import us.ihmc.communication.property.StoredPropertySetROS2TopicPair;
import us.ihmc.communication.ros2.ROS2IOTopicPair;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;

/**
 * ROS 2 topics relating to perception
 */
public final class PerceptionAPI
{
   public static final ROS2Topic<?> IHMC_ROOT = ROS2Tools.IHMC_ROOT;
   public static final ROS2Topic<?> PERCEPTION_MODULE = IHMC_ROOT.withModule("perception");
   public static final ROS2Topic<?> SCENE_GRAPH_MODULE = IHMC_ROOT.withModule("scene_graph");
   public static final ROS2Topic<?> HEIGHT_MAP_MODULE = IHMC_ROOT.withModule("height_map");
   public static final ROS2Topic<?> TERRAIN_MAP_MODULE = IHMC_ROOT.withModule("terrain_map");

   public static final ROS2Topic<?> BEST_EFFORT = IHMC_ROOT.withQoS(ROS2QosProfile.BEST_EFFORT());

   /*
    * Scene graph
    */
   public static final ROS2IOTopicPair<SceneGraphMessage> SCENE_GRAPH = new ROS2IOTopicPair<>(SCENE_GRAPH_MODULE.withTypeName(SceneGraphMessage.class));

   /*
    * Doors
    */
   public static final ROS2Topic<DetectedDoorListMessage> DETECTED_DOORS = IHMC_ROOT.withModule("door_detection").withType(DetectedDoorListMessage.class);

   /*
    * Detection manager
    */
   public static final StoredPropertySetROS2TopicPair DETECTION_MANAGER_SETTINGS = new StoredPropertySetROS2TopicPair("detections", "settings");

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
   public static final ROS2Topic<ImageMessage> YOLO_ANNOTATED_IMAGE = PERCEPTION_MODULE.withModule("yolo")
                                                                                       .withType(ImageMessage.class)
                                                                                       .withSuffix("annotated_image");
   public static final ROS2Topic<YOLOv8ExecutorParameters> YOLO_PARAMETERS = IHMC_ROOT.withModule("yolo")
                                                                                      .withSuffix("settings")
                                                                                      .withType(YOLOv8ExecutorParameters.class);

   /*
    * Aruco markers
    */
   public static final ROS2Topic<Empty> REQUEST_ARUCO = PERCEPTION_MODULE.withSuffix("request_aruco").withType(Empty.class);
   public static final ROS2Topic<ArUcoMarkerPoses> ARUCO_MARKER_POSES = PERCEPTION_MODULE.withType(ArUcoMarkerPoses.class).withSuffix("aruco_marker_poses");

   /*
    * ZED image topics (IHMC ImageMessage type)
    */
   public static final ROS2Topic<Empty> REQUEST_ZED = PERCEPTION_MODULE.withSuffix("request_zed").withType(Empty.class);
   public static final ROS2Topic<Empty> REQUEST_ZED_PUBLICATION = PERCEPTION_MODULE.withSuffix("request_zed_publication").withType(Empty.class);
   public static final ROS2Topic<ImageMessage> ZED_DEPTH = BEST_EFFORT.withModule("zed").withType(ImageMessage.class).withSuffix("depth");
   public static final SideDependentList<ROS2Topic<ImageMessage>> ZED_COLOR_IMAGES = new SideDependentList<>(BEST_EFFORT.withModule("zed")
                                                                                                                        .withType(ImageMessage.class)
                                                                                                                        .withSuffix("left_color"),
                                                                                                             BEST_EFFORT.withModule("zed")
                                                                                                                        .withType(ImageMessage.class)
                                                                                                                        .withSuffix("right_color"));

   /*
    * ZED image topics (official ROS 2 Image and CameraInfo types)
    */
   private static final ROS2Topic<?> ROS2_ZED = new ROS2Topic<>().withPrefix("zed").withQoS(ROS2QosProfile.RELIABLE());
   private static final ROS2Topic<?> ROS2_ZED_COLOR = ROS2_ZED.withModule("color");
   public static final SideDependentList<ROS2Topic<Image>> ROS2_ZED_COLOR_IMAGES = new SideDependentList<>(ROS2_ZED_COLOR.withSuffix("left/image")
                                                                                                                         .withType(Image.class),
                                                                                                           ROS2_ZED_COLOR.withSuffix("right/image")
                                                                                                                         .withType(Image.class));
   public static final SideDependentList<ROS2Topic<CameraInfo>> ROS2_ZED_COLOR_CAMERA_INFOS = new SideDependentList<>(ROS2_ZED_COLOR.withSuffix(
         "left/camera_info").withType(CameraInfo.class), ROS2_ZED_COLOR.withSuffix("right/camera_info").withType(CameraInfo.class));
   private static final ROS2Topic<?> ROS2_ZED_DEPTH = ROS2_ZED.withModule("depth");
   public static final ROS2Topic<Image> ROS2_ZED_DEPTH_IMAGE = ROS2_ZED_DEPTH.withSuffix("image").withType(Image.class);
   public static final ROS2Topic<CameraInfo> ROS2_ZED_DEPTH_CAMERA_INFO = ROS2_ZED_DEPTH.withSuffix("camera_info").withType(CameraInfo.class);

   /*
    * RealSense image topics (IHMC ImageMessage type)
    */
   public static final ROS2Topic<Empty> REQUEST_REALSENSE = PERCEPTION_MODULE.withSuffix("request_realsense").withType(Empty.class);
   public static final ROS2Topic<Empty> REQUEST_REALSENSE_PUBLICATION = PERCEPTION_MODULE.withSuffix("request_realsense_publication").withType(Empty.class);
   public static final ROS2Topic<ImageMessage> D455_DEPTH_IMAGE = BEST_EFFORT.withModule("d455").withTypeName(ImageMessage.class).withSuffix("depth");
   public static final ROS2Topic<ImageMessage> D455_COLOR_IMAGE = BEST_EFFORT.withModule("d455").withTypeName(ImageMessage.class).withSuffix("color");
   public static final ROS2Topic<ImageMessage> REALSENSE_DEPTH_FILTERED_IMAGE = BEST_EFFORT.withModule("realsense")
                                                                                           .withTypeName(ImageMessage.class)
                                                                                           .withSuffix("depth_filtered");

   /*
    * RealSense image topics (official ROS 2 Image and CameraInfo types)
    */
   private static final ROS2Topic<?> ROS2_REALSENSE = new ROS2Topic<>().withPrefix("realsense").withQoS(ROS2QosProfile.RELIABLE());
   private static final ROS2Topic<?> ROS2_REALSENSE_COLOR = ROS2_REALSENSE.withModule("color");
   public static final ROS2Topic<Image> ROS2_REALSENSE_COLOR_IMAGE = ROS2_REALSENSE_COLOR.withSuffix("image").withType(Image.class);
   public static final ROS2Topic<CameraInfo> ROS2_REALSENSE_COLOR_CAMERA_INFO = ROS2_REALSENSE_COLOR.withSuffix("camera_info").withType(CameraInfo.class);
   private static final ROS2Topic<?> ROS2_REALSENSE_DEPTH = ROS2_REALSENSE.withModule("depth");
   public static final ROS2Topic<Image> ROS2_REALSENSE_DEPTH_IMAGE = ROS2_REALSENSE_DEPTH.withSuffix("image").withType(Image.class);
   public static final ROS2Topic<CameraInfo> ROS2_REALSENSE_DEPTH_CAMERA_INFO = ROS2_REALSENSE_DEPTH.withSuffix("camera_info").withType(CameraInfo.class);

   /*
    * Planar regions
    */
   public static final ROS2Topic<Empty> REQUEST_PLANAR_REGIONS = IHMC_ROOT.withModule("planar_regions").withSuffix("request").withType(Empty.class);
   public static final ROS2Topic<FramePlanarRegionsListMessage> PERSPECTIVE_RAPID_REGIONS = PERCEPTION_MODULE.withOutput()
                                                                                                             .withTypeName(FramePlanarRegionsListMessage.class)
                                                                                                             .withSuffix("perspective")
                                                                                                             .withQoS(ROS2QosProfile.BEST_EFFORT());
   public static final ROS2Topic<FramePlanarRegionsListMessage> SPHERICAL_RAPID_REGIONS_WITH_POSE = PERCEPTION_MODULE.withOutput()
                                                                                                                     .withTypeName(FramePlanarRegionsListMessage.class)
                                                                                                                     .withSuffix("spherical");

   /*
    * Height map
    */
   public static final ROS2Topic<Empty> REQUEST_HEIGHT_MAP = PERCEPTION_MODULE.withSuffix("request_height_map").withType(Empty.class);
   public static final ROS2Topic<Empty> REQUEST_HEIGHT_MAP_FOR_CONTROLLER = PERCEPTION_MODULE.withSuffix("request_height_map_for_controller").withType(Empty.class);
   public static final ROS2Topic<Empty> REQUEST_TERRAIN_MAP = PERCEPTION_MODULE.withSuffix("request_terrain_map").withType(Empty.class);
   public static final ROS2Topic<HeightMapMessage> HEIGHT_MAP_MESSAGE = HEIGHT_MAP_MODULE.withOutput().withTypeName(HeightMapMessage.class);
   public static final ROS2Topic<ChunkMessage> CHUNK = PERCEPTION_MODULE.withOutput().withTypeName(ChunkMessage.class).withSuffix("chunk");
   public static final ROS2Topic<Empty> RESET_HEIGHT_MAP = PERCEPTION_MODULE.withSuffix("reset_height_map").withType(Empty.class);
   public static final ROS2Topic<Empty> LOWER_HEIGHT_MAP_BACKDROP = PERCEPTION_MODULE.withSuffix("lower_height_map_backdrop").withType(Empty.class);

   /*
    * Terrain map
    */
   public static final ROS2Topic<TerrainMapMessage> TERRAIN_MAP_MESSAGE = TERRAIN_MAP_MODULE.withOutput().withTypeName(TerrainMapMessage.class);

   /*
    * SLAM (old, not used)
    */
   public static final ROS2Topic<PlanarRegionsListMessage> SLAM_OUTPUT_RAPID_REGIONS = PERCEPTION_MODULE.withOutput()
                                                                                                        .withTypeName(PlanarRegionsListMessage.class)
                                                                                                        .withSuffix("slam_output");

   /*
    * Transform tuning
    */
   private static final ROS2Topic<RigidBodyTransformMessage> TRANSFORM_TUNING_BASE_TOPIC = IHMC_ROOT.withTypeName(RigidBodyTransformMessage.class)
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
    * ICP (old, not used)
    */
   public static final ROS2Topic<IterativeClosestPointRequest> ICP_REQUEST = IHMC_ROOT.withModule("iterative_closest_point")
                                                                                      .withSuffix("request")
                                                                                      .withType(IterativeClosestPointRequest.class);
   public static final ROS2Topic<DetectedObjectPacket> ICP_RESULT = IHMC_ROOT.withModule("iterative_closest_point")
                                                                             .withSuffix("result")
                                                                             .withType(DetectedObjectPacket.class);

   /*
    * YOLO
    */
   public static final ROS2Topic<Empty> REQUEST_YOLO_REALSENSE = PERCEPTION_MODULE.withSuffix("request_yolo_realsense").withType(Empty.class);
   public static final ROS2Topic<Empty> REQUEST_YOLO_ZED = PERCEPTION_MODULE.withSuffix("request_yolo_zed").withType(Empty.class);
   public static final ROS2Topic<Empty> REQUEST_YOLO_ANNOTATED_IMAGE = PERCEPTION_MODULE.withSuffix("request_yolo_image").withType(Empty.class);

   /*
    * Mocap
    */
   public static final ROS2Topic<Pose3D> MOCAP_RIGID_BODY = IHMC_ROOT.withTypeName(Pose3D.class).withModule("frame_update").withSuffix("mocap");
}
