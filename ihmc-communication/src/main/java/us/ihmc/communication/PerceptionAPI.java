package us.ihmc.communication;

import controller_msgs.msg.dds.BipedalSupportPlanarRegionParametersMessage;
import controller_msgs.msg.dds.ConcaveHullFactoryParametersStringMessage;
import controller_msgs.msg.dds.RigidBodyTransformMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import ihmc_common_msgs.msg.dds.StampedPosePacket;
import ihmc_common_msgs.msg.dds.TextToSpeechPacket;
import perception_msgs.msg.dds.ArUcoMarkerPoses;
import perception_msgs.msg.dds.BigVideoPacket;
import perception_msgs.msg.dds.DetectedObjectPacket;
import perception_msgs.msg.dds.DoorLocationPacket;
import perception_msgs.msg.dds.FramePlanarRegionsListMessage;
import perception_msgs.msg.dds.FusedSensorHeadPointCloudMessage;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.HeightMapStateRequestMessage;
import perception_msgs.msg.dds.ImageMessage;
import perception_msgs.msg.dds.IterativeClosestPointRequest;
import perception_msgs.msg.dds.LidarScanMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import perception_msgs.msg.dds.PolygonizerParametersStringMessage;
import perception_msgs.msg.dds.REAStateRequestMessage;
import perception_msgs.msg.dds.SceneGraphMessage;
import perception_msgs.msg.dds.VideoPacket;
import std_msgs.msg.dds.Empty;
import std_msgs.msg.dds.Float64;
import us.ihmc.communication.ros2.ROS2IOTopicPair;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;

public class PerceptionAPI
{
   public static final String IHMC_TOPIC_PREFIX = "ihmc";

   public static final String REA_NODE_NAME = "REA_module";
   public static final String GPU_REA_NODE_NAME = "GPU_based_REA_module";
   public static final String MAPPING_MODULE_NODE_NAME = "mapping_module";
   public static final String STEREO_REA_NODE_NAME = "SREA_module";

   public static final String HEIGHT_QUADTREE_TOOLBOX_MODULE_NAME = "toolbox/height_quad_tree";
   public static final String FIDUCIAL_MODULE_NAME = "toolbox/fiducial_detector";
   public static final String OBJECT_DETECTOR_MODULE_NAME = "toolbox/object_detector";

   public static final String SCENE_GRAPH_MODULE_NAME = "scene_graph";
   public static final String REA_MODULE_NAME = "rea";
   public static final String MAPPING_MODULE_NAME = "map";
   public static final String PERCEPTION_MODULE_NAME = "perception";
   public static final String REALSENSE_SLAM_MODULE_NAME = "slam";
   public static final String MAPSENSE_MODULE_NAME = "mapsense";
   public static final String HEIGHT_MAP_MODULE_NAME = "height_map";

   public static final String REA_CUSTOM_REGION_NAME = "custom_region";
   public static final String D435_NAME = "d435";
   public static final String ZED2_NAME = "zed2";
   public static final String L515_NAME = "l515";
   public static final String D455_NAME = "d455";
   public static final String BLACKFLY_NAME = "blackfly";
   public static final String T265_NAME = "t265";
   public static final String MULTISENSE_NAME = "multisense";

   public static final ROS2Topic<?> IHMC_ROOT = new ROS2Topic<>().withPrefix(IHMC_TOPIC_PREFIX);
   public static final ROS2Topic<?> HEIGHT_QUADTREE_TOOLBOX = IHMC_ROOT.withModule(HEIGHT_QUADTREE_TOOLBOX_MODULE_NAME);
   public static final ROS2Topic<?> FIDUCIAL_DETECTOR_TOOLBOX = IHMC_ROOT.withModule(FIDUCIAL_MODULE_NAME);
   public static final ROS2Topic<?> FIDUCIAL_DETECTOR_TOOLBOX_INPUT = FIDUCIAL_DETECTOR_TOOLBOX.withInput();
   public static final ROS2Topic<?> FIDUCIAL_DETECTOR_TOOLBOX_OUTPUT = FIDUCIAL_DETECTOR_TOOLBOX.withOutput();

   public static final ROS2Topic<?> OBJECT_DETECTOR_TOOLBOX = IHMC_ROOT.withModule(OBJECT_DETECTOR_MODULE_NAME);
   public static final ROS2Topic<?> OBJECT_DETECTOR_TOOLBOX_INPUT = OBJECT_DETECTOR_TOOLBOX.withInput();
   public static final ROS2Topic<?> OBJECT_DETECTOR_TOOLBOX_OUTPUT = OBJECT_DETECTOR_TOOLBOX.withOutput();

   public static final ROS2Topic<?> SCENE_GRAPH_MODULE = IHMC_ROOT.withModule(SCENE_GRAPH_MODULE_NAME);
   public static final ROS2Topic<?> REA = IHMC_ROOT.withModule(REA_MODULE_NAME);
   public static final ROS2Topic<?> MAPPING_MODULE = IHMC_ROOT.withModule(MAPPING_MODULE_NAME);
   public static final ROS2Topic<?> PERCEPTION_MODULE = IHMC_ROOT.withModule(PERCEPTION_MODULE_NAME);
   public static final ROS2Topic<?> REALSENSE_SLAM_MODULE = IHMC_ROOT.withModule(REALSENSE_SLAM_MODULE_NAME);
   public static final ROS2Topic<?> MAPSENSE_MODULE = IHMC_ROOT.withModule(MAPPING_MODULE_NAME);
   public static final ROS2Topic<?> HEIGHT_MAP_MODULE = IHMC_ROOT.withModule(HEIGHT_MAP_MODULE_NAME);

   public static final String BIPED_SUPPORT_REGION_PUBLISHER_MODULE_NAME = "bipedal_support_region_publisher";
   public static final ROS2Topic<?> BIPED_SUPPORT_REGION_PUBLISHER = IHMC_ROOT.withModule(BIPED_SUPPORT_REGION_PUBLISHER_MODULE_NAME);

   public static final ROS2Topic<TextToSpeechPacket> TEXT_STATUS = IHMC_ROOT.withTypeName(TextToSpeechPacket.class);

   public static final ROS2Topic<?> REA_SUPPORT_REGIONS = REA.withSuffix(REA_CUSTOM_REGION_NAME);
   public static final ROS2Topic<PlanarRegionsListMessage> REA_SUPPORT_REGIONS_INPUT = REA.withRobot(null)
                                                                                          .withInput()
                                                                                          .withType(PlanarRegionsListMessage.class)
                                                                                          .withSuffix(PerceptionAPI.REA_CUSTOM_REGION_NAME);
   public static final ROS2Topic<REAStateRequestMessage> REA_STATE_REQUEST = REA.withInput().withTypeName(REAStateRequestMessage.class);
   public static final ROS2Topic<ConcaveHullFactoryParametersStringMessage> CONCAVE_HULL_FACTORY_PARAMETERS = REA.withInput()
                                                                                                                 .withType(ConcaveHullFactoryParametersStringMessage.class)
                                                                                                                 .withSuffix("concave_hull_factory_parameters");
   public static final ROS2Topic<PolygonizerParametersStringMessage> POLYGONIZER_PARAMETERS = REA.withInput()
                                                                                                 .withType(PolygonizerParametersStringMessage.class)
                                                                                                 .withSuffix("polygonizer_parameters");
   public static final ROS2Topic<?> BEST_EFFORT = IHMC_ROOT.withQoS(ROS2QosProfile.BEST_EFFORT());
   public static final ROS2Topic<VideoPacket> VIDEO = BEST_EFFORT.withTypeName(VideoPacket.class);
   public static final ROS2Topic<BigVideoPacket> BIG_VIDEO = BEST_EFFORT.withTypeName(BigVideoPacket.class).withQoS(ROS2QosProfile.BEST_EFFORT());
   public static final ROS2Topic<VideoPacket> D435_VIDEO = BEST_EFFORT.withModule(D435_NAME).withType(VideoPacket.class).withSuffix("video");
   public static final ROS2Topic<ImageMessage> D435_COLOR_IMAGE = BEST_EFFORT.withModule(D435_NAME).withType(ImageMessage.class).withSuffix("video");
   public static final ROS2Topic<ImageMessage> D435_DEPTH_IMAGE = BEST_EFFORT.withModule(D435_NAME).withType(ImageMessage.class).withSuffix("depth");
   public static final ROS2Topic<VideoPacket> D435_DEPTH = BEST_EFFORT.withModule(D435_NAME).withType(VideoPacket.class).withSuffix("depth");
   public static final ROS2Topic<VideoPacket> L515_VIDEO = BEST_EFFORT.withModule(L515_NAME).withType(VideoPacket.class).withSuffix("video");
   public static final ROS2Topic<ImageMessage> L515_COLOR_IMAGE = BEST_EFFORT.withModule(L515_NAME).withTypeName(ImageMessage.class).withSuffix("color");
   public static final ROS2Topic<BigVideoPacket> L515_DEPTH_LARGE = BEST_EFFORT.withModule(L515_NAME).withType(BigVideoPacket.class).withSuffix("depth");
   public static final ROS2Topic<VideoPacket> L515_DEPTH = BEST_EFFORT.withModule(L515_NAME).withType(VideoPacket.class).withSuffix("depth");
   public static final ROS2Topic<ImageMessage> L515_DEPTH_IMAGE = BEST_EFFORT.withModule(L515_NAME).withTypeName(ImageMessage.class).withSuffix("depth");
   public static final ROS2Topic<ImageMessage> D455_DEPTH_IMAGE = BEST_EFFORT.withModule(D455_NAME).withTypeName(ImageMessage.class).withSuffix("depth");
   public static final ROS2Topic<ImageMessage> D455_COLOR_IMAGE = BEST_EFFORT.withModule(D455_NAME).withTypeName(ImageMessage.class).withSuffix("color");
   public static final ROS2Topic<BigVideoPacket> D455_DEPTH_LARGE = BEST_EFFORT.withModule(D455_NAME).withType(BigVideoPacket.class).withSuffix("depth");

   public static final ROS2Topic<ImageMessage> TERRAIN_DEBUG_IMAGE
         = BEST_EFFORT.withModule(L515_NAME).withType(ImageMessage.class).withSuffix("terrain_debug_image");
   public static final ROS2Topic<BigVideoPacket> L515_DEBUG_EXTRACTION = BEST_EFFORT.withModule(L515_NAME)
                                                                                  .withType(BigVideoPacket.class)
                                                                                  .withSuffix("debug_extraction");
   public static final ROS2Topic<Empty> REQUEST_REALSENSE_POINT_CLOUD = PERCEPTION_MODULE.withSuffix("request_realsense_point_cloud").withType(Empty.class);
   public static final SideDependentList<ROS2Topic<BigVideoPacket>> BLACKFLY_VIDEO = new SideDependentList<>(BEST_EFFORT.withModule(BLACKFLY_NAME + "left")
                                                                                                                        .withType(BigVideoPacket.class)
                                                                                                                        .withSuffix("video"),
                                                                                                             BEST_EFFORT.withModule(BLACKFLY_NAME + "right")
                                                                                                                        .withType(BigVideoPacket.class)
                                                                                                                        .withSuffix("video"));
   public static final SideDependentList<ROS2Topic<ImageMessage>> BLACKFLY_FISHEYE_COLOR_IMAGE
         = new SideDependentList<>(BEST_EFFORT.withModule(BLACKFLY_NAME + "_fisheye")
                                              .withType(ImageMessage.class)
                                              .withSuffix("left"),
                                   BEST_EFFORT.withModule(BLACKFLY_NAME + "_fisheye")
                                              .withType(ImageMessage.class)
                                              .withSuffix("right"));
   public static final SideDependentList<ROS2Topic<Empty>> REQUEST_BLACKFLY_COLOR_IMAGE
         = new SideDependentList<>(PERCEPTION_MODULE.withSuffix("request_left_blackfly_color").withType(Empty.class),
                                   PERCEPTION_MODULE.withSuffix("request_right_blackfly_color").withType(Empty.class));

   public static final ROS2Topic<BigVideoPacket> OUSTER_DEPTH_LARGE = BEST_EFFORT.withModule("ouster").withType(BigVideoPacket.class).withSuffix("depth");
   public static final ROS2Topic<VideoPacket> OUSTER_DEPTH = BEST_EFFORT.withModule("ouster").withType(VideoPacket.class).withSuffix("depth");
   public static final ROS2Topic<ImageMessage> OUSTER_DEPTH_IMAGE = BEST_EFFORT.withModule("ouster")
                                                                               .withTypeName(ImageMessage.class)
                                                                               .withSuffix("depth")
                                                                               .withQoS(ROS2QosProfile.BEST_EFFORT());
   public static final ROS2Topic<BigVideoPacket> BIG_VIDEO_TEST = BEST_EFFORT.withModule(BLACKFLY_NAME).withType(BigVideoPacket.class).withSuffix("test");
   public static final ROS2Topic<Empty> REQUEST_OUSTER_DEPTH = PERCEPTION_MODULE.withSuffix("request_ouster_depth").withType(Empty.class);

   public static final ROS2Topic<Empty> REQUEST_ZED_COLOR = PERCEPTION_MODULE.withSuffix("request_zed_color").withType(Empty.class);
   public static final ROS2Topic<Empty> REQUEST_ZED_DEPTH = PERCEPTION_MODULE.withSuffix("request_zed_depth").withType(Empty.class);
   public static final ROS2Topic<Empty> REQUEST_ZED_POINT_CLOUD = PERCEPTION_MODULE.withSuffix("request_zed_point_cloud").withType(Empty.class);
   public static final ROS2Topic<Empty> REQUEST_CENTERPOSE = PERCEPTION_MODULE.withSuffix("request_centerpose").withType(Empty.class);
   public static final ROS2Topic<DetectedObjectPacket> CENTERPOSE_DETECTED_OBJECT = IHMC_ROOT.withModule("centerpose").withType(DetectedObjectPacket.class);
   public static final ROS2Topic<ImageMessage> ZED2_STEREO_COLOR = IHMC_ROOT.withModule(ZED2_NAME).withType(ImageMessage.class).withSuffix("color_stereo");
   public static final SideDependentList<ROS2Topic<ImageMessage>> ZED2_COLOR_IMAGES = new SideDependentList<>(BEST_EFFORT.withModule(ZED2_NAME)
                                                                                                                         .withType(ImageMessage.class)
                                                                                                                         .withSuffix("left_color"),
                                                                                                              BEST_EFFORT.withModule(ZED2_NAME)
                                                                                                                         .withType(ImageMessage.class)
                                                                                                                         .withSuffix("right_color"));
   public static final ROS2Topic<ImageMessage> ZED2_DEPTH = BEST_EFFORT.withModule(ZED2_NAME).withType(ImageMessage.class).withSuffix("depth");
   public static final ROS2Topic<IterativeClosestPointRequest> ICP_REQUEST = IHMC_ROOT.withModule("iterative_closest_point")
                                                                                      .withSuffix("request")
                                                                                      .withType(IterativeClosestPointRequest.class);
   public static final ROS2Topic<DetectedObjectPacket> ICP_RESULT = IHMC_ROOT.withModule("iterative_closest_point")
                                                                             .withSuffix("result")
                                                                             .withType(DetectedObjectPacket.class);
   public static final ROS2Topic<Empty> REQUEST_PLANAR_REGIONS = IHMC_ROOT.withModule("planar_regions")
                                                                          .withSuffix("request")
                                                                          .withType(Empty.class);

   public static final ROS2Topic<LidarScanMessage> MULTISENSE_LIDAR_SCAN = IHMC_ROOT.withTypeName(LidarScanMessage.class);
   public static final ROS2Topic<FusedSensorHeadPointCloudMessage> FUSED_SENSOR_HEAD_POINT_CLOUD = BEST_EFFORT.withTypeName(FusedSensorHeadPointCloudMessage.class);
   public static final ROS2Topic<FusedSensorHeadPointCloudMessage> D435_COLORED_POINT_CLOUD = BEST_EFFORT.withType(FusedSensorHeadPointCloudMessage.class)
                                                                                                         .withSuffix("d435_color");
   public static final ROS2Topic<FusedSensorHeadPointCloudMessage> OUSTER_POINT_CLOUD = BEST_EFFORT.withType(FusedSensorHeadPointCloudMessage.class)
                                                                                                   .withSuffix("ouster");
   public static final ROS2Topic<LidarScanMessage> OUSTER_LIDAR_SCAN = BEST_EFFORT.withType(LidarScanMessage.class).withSuffix("ouster");
   public static final ROS2Topic<StereoVisionPointCloudMessage> MULTISENSE_STEREO_POINT_CLOUD = BEST_EFFORT.withTypeName(StereoVisionPointCloudMessage.class);

   public static final ROS2Topic<StereoVisionPointCloudMessage> D435_POINT_CLOUD = BEST_EFFORT.withSuffix(D435_NAME)
                                                                                              .withTypeName(StereoVisionPointCloudMessage.class);
   public static final ROS2Topic<StereoVisionPointCloudMessage> L515_POINT_CLOUD = BEST_EFFORT.withSuffix(L515_NAME)
                                                                                              .withTypeName(StereoVisionPointCloudMessage.class);
   public static final ROS2Topic<StampedPosePacket> T265_POSE = IHMC_ROOT.withSuffix(T265_NAME).withTypeName(StampedPosePacket.class);

   public static final ROS2IOTopicPair<SceneGraphMessage> SCENE_GRAPH = new ROS2IOTopicPair<>(SCENE_GRAPH_MODULE.withTypeName(SceneGraphMessage.class));

   /** MoCap Topics */
   public static final ROS2Topic<Pose3D> MOCAP_RIGID_BODY = IHMC_ROOT.withTypeName(Pose3D.class)
                                                                     .withModule("frame_update")
                                                                     .withSuffix("mocap");

   /** Output regions from Lidar (Multisense) from REA */
   public static final ROS2Topic<PlanarRegionsListMessage> LIDAR_REA_REGIONS = REA.withOutput().withTypeName(PlanarRegionsListMessage.class);
   public static final ROS2Topic<PlanarRegionsListMessage> REALSENSE_REA = PerceptionAPI.REA.withOutput()
                                                                                            .withPrefix("stereo")
                                                                                            .withTypeName(PlanarRegionsListMessage.class);
   public static final ROS2Topic<PlanarRegionsListMessage> BIPEDAL_SUPPORT_REGIONS = REA_SUPPORT_REGIONS.withTypeName(PlanarRegionsListMessage.class)
                                                                                                        .withInput();
   /** Output regions from Atlas Realsense SLAM module */
   public static final ROS2Topic<PlanarRegionsListMessage> REALSENSE_SLAM_REGIONS = REALSENSE_SLAM_MODULE.withOutput()
                                                                                                         .withTypeName(PlanarRegionsListMessage.class);
   public static final ROS2Topic<PlanarRegionsListMessage> MAPSENSE_REGIONS = MAPSENSE_MODULE.withOutput().withTypeName(PlanarRegionsListMessage.class);
   /** Rapid regions are generated in Java, come with epoch second and nano timestamp, are pre-filtered for body collisions, and in world frame.
    *  They are prre filtered using the polygonizer, segmentation, and concave hull filtering parameters. There is no need for delay compensation. */

   public static final ROS2Topic<FramePlanarRegionsListMessage> PERSPECTIVE_RAPID_REGIONS
         = PERCEPTION_MODULE.withOutput().withTypeName(FramePlanarRegionsListMessage.class).withSuffix("perspective").withQoS(ROS2QosProfile.BEST_EFFORT());

   public static final ROS2Topic<FramePlanarRegionsListMessage> SPHERICAL_RAPID_REGIONS_WITH_POSE = PERCEPTION_MODULE.withOutput().withTypeName(FramePlanarRegionsListMessage.class).withSuffix("spherical");
   public static final ROS2Topic<PlanarRegionsListMessage> SPHERICAL_RAPID_REGIONS = PERCEPTION_MODULE.withOutput().withTypeName(PlanarRegionsListMessage.class).withSuffix("spherical");

   public static final ROS2Topic<PlanarRegionsListMessage> SLAM_OUTPUT_RAPID_REGIONS = PERCEPTION_MODULE.withOutput().withTypeName(PlanarRegionsListMessage.class).withSuffix("slam_output");

   private static final ROS2Topic<BipedalSupportPlanarRegionParametersMessage> BIPEDAL_SUPPORT_REGION_PARAMETERS = BIPED_SUPPORT_REGION_PUBLISHER.withInput()
                                                                                                                                                 .withType(BipedalSupportPlanarRegionParametersMessage.class);


   /**
    * Rapid regions are generated in Java, come with epoch second and nano timestamp, are pre-filtered
    * for body collisions, and in world frame. They are prre filtered using the polygonizer,
    * segmentation, and concave hull filtering parameters. There is no need for delay compensation.
    */
   public static final ROS2Topic<PlanarRegionsListMessage> RAPID_REGIONS = PERCEPTION_MODULE.withOutput().withTypeName(PlanarRegionsListMessage.class);
   /** Output regions from experimental mapping module which assembles the above outputs */
   public static final ROS2Topic<PlanarRegionsListMessage> MAP_REGIONS = MAPPING_MODULE.withOutput().withTypeName(PlanarRegionsListMessage.class);
   public static final ROS2Topic<Float64> MAPSENSE_REGIONS_DELAY_OFFSET = MAPSENSE_MODULE.withType(Float64.class).withSuffix("delay_offset");
   public static final ROS2Topic<HeightMapMessage> HEIGHT_MAP_OUTPUT = HEIGHT_MAP_MODULE.withOutput().withTypeName(HeightMapMessage.class);
   public static final ROS2Topic<HeightMapStateRequestMessage> HEIGHT_MAP_STATE_REQUEST = HEIGHT_MAP_MODULE.withOutput().withTypeName(HeightMapStateRequestMessage.class);

   public static final ROS2Topic<ImageMessage> OCCUPANCY_GRID_MESSAGE = PERCEPTION_MODULE.withOutput().withTypeName(ImageMessage.class).withSuffix("occupancy_grid");
   public static final ROS2Topic<ImageMessage> HEIGHT_MAP_CROPPED = PERCEPTION_MODULE.withOutput().withTypeName(ImageMessage.class).withSuffix("height_map_cropped_global");
   public static final ROS2Topic<ImageMessage> HEIGHT_MAP_GLOBAL = PERCEPTION_MODULE.withOutput().withTypeName(ImageMessage.class).withSuffix("height_map_global");
   public static final ROS2Topic<ImageMessage> HEIGHT_MAP_LOCAL = PERCEPTION_MODULE.withOutput().withTypeName(ImageMessage.class).withSuffix("height_map_local");

   public static final ROS2Topic<Empty> REQUEST_LIDAR_SCAN = PERCEPTION_MODULE.withSuffix("request_lidar_scan").withType(Empty.class);
   public static final ROS2Topic<Empty> REQUEST_HEIGHT_MAP = PERCEPTION_MODULE.withSuffix("request_height_map").withType(Empty.class);

   public static final ROS2Topic<ArUcoMarkerPoses> ARUCO_MARKER_POSES = PERCEPTION_MODULE.withType(ArUcoMarkerPoses.class).withSuffix("aruco_marker_poses");
   public static final ROS2Topic<Empty> REQUEST_ARUCO = PERCEPTION_MODULE.withSuffix("request_aruco").withType(Empty.class);

   private static final ROS2Topic<RigidBodyTransformMessage> TRANSFORM_TUNING_BASE_TOPIC = IHMC_ROOT.withTypeName(RigidBodyTransformMessage.class)
                                                                                                    .withModule("transform_tuning");
   public static final ROS2IOTopicPair<RigidBodyTransformMessage> OBJECT_DETECTION_CAMERA_TO_PARENT_TUNING
         = new ROS2IOTopicPair<>(TRANSFORM_TUNING_BASE_TOPIC.withSuffix("object_detection_camera_to_parent"));

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

   public static final ROS2IOTopicPair<RigidBodyTransformMessage> STEPPING_CAMERA_TO_PARENT_TUNING
         = new ROS2IOTopicPair<>(TRANSFORM_TUNING_BASE_TOPIC.withSuffix("stepping_camera_to_parent"));
   public static final ROS2IOTopicPair<RigidBodyTransformMessage> EXPERIMENTAL_CAMERA_TO_PARENT_TUNING
         = new ROS2IOTopicPair<>(TRANSFORM_TUNING_BASE_TOPIC.withSuffix("experimental_camera_to_parent"));
   public static final ROS2IOTopicPair<RigidBodyTransformMessage> OUSTER_TO_CHEST_TUNING
         = new ROS2IOTopicPair<>(TRANSFORM_TUNING_BASE_TOPIC.withSuffix("ouster_to_chest"));

   public static ROS2Topic<DoorLocationPacket> getDoorLocationTopic(String robotName)
   {
      return OBJECT_DETECTOR_TOOLBOX_OUTPUT.withRobot(robotName).withTypeName(DoorLocationPacket.class);
   }

   public static ROS2Topic<BipedalSupportPlanarRegionParametersMessage> getBipedalSupportRegionParametersTopic(String robotName)
   {
      return BIPEDAL_SUPPORT_REGION_PARAMETERS.withRobot(robotName);
   }
}
