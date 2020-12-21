package us.ihmc.communication;

import java.io.IOException;
import java.net.InetAddress;
import java.util.function.Function;

import controller_msgs.msg.dds.*;
import sensor_msgs.msg.dds.CompressedImage;
import sensor_msgs.msg.dds.Image;
import us.ihmc.commons.exception.ExceptionHandler;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.*;
import us.ihmc.util.PeriodicNonRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicThreadSchedulerFactory;

public class ROS2Tools
{
   public static final String IHMC_TOPIC_PREFIX = "ihmc";

   public static final String HUMANOID_CONTROLLER_NODE_NAME = "ihmc_controller";
   public static final String HUMANOID_KINEMATICS_CONTROLLER_NODE_NAME = "kinematics_ihmc_controller";
   public static final String REA_NODE_NAME = "REA_module";
   public static final String MAPPING_MODULE_NODE_NAME = "mapping_module";
   public static final String STEREO_REA_NODE_NAME = "SREA_module";
   public static final String LLAMA_NODE_NAME = "llama_network";
   public static final String FOOTSTEP_PLANNER_NODE_NAME = "ihmc_multi_stage_footstep_planning_module";
   public static final String BEHAVIOR_MODULE_NODE_NAME = "behavior_module";

   public static final String HUMANOID_CONTROL_MODULE_NAME = "humanoid_control";
   public static final String QUADRUPED_CONTROL_MODULE_NAME = "quadruped_control";
   public static final String FOOTSTEP_PLANNER_MODULE_NAME = "toolbox/footstep_plan";
   public static final String CONTINUOUS_PLANNING_TOOLBOX_MODULE_NAME = "toolbox/continuous_planning";
   public static final String FOOTSTEP_POSTPROCESSING_TOOLBOX_MODULE_NAME = "toolbox/footstep_postprocessing";
   public static final String HEIGHT_QUADTREE_TOOLBOX_MODULE_NAME = "toolbox/height_quad_tree";
   public static final String FIDUCIAL_MODULE_NAME = "toolbox/fiducial_detector";
   public static final String OBJECT_DETECTOR_MODULE_NAME = "toolbox/object_detector";

   public static final String KINEMATICS_TOOLBOX_MODULE_NAME = "toolbox/ik";
   public static final String KINEMATICS_PLANNING_TOOLBOX_MODULE_NAME = "toolbox/ik_planning";
   public static final String KINEMATICS_STREAMING_TOOLBOX_MODULE_NAME = "toolbox/ik_streaming";
   public static final String STEP_CONSTRAINT_TOOLBOX_MODULE_NAME = "/toolbox/step_constraint";

   public static final String WHOLE_BODY_TRAJECTORY_TOOLBOX_MODULE_NAME = "toolbox/ik_trajectory";
   public static final String WALKING_PREVIEW_TOOLBOX_MODULE_NAME = "toolbox/walking_controller_preview";
   public static final String EXTERNAL_FORCE_ESTIMATION_TOOLBOX_MODULE_NAME = "toolbox/external_force_estimation";
   public static final String STEP_TELEOP_TOOLBOX_MODULE_NAME = "toolbox/teleop/step_teleop";
   public static final String QUADRUPED_SUPPORT_REGION_PUBLISHER_MODULE_NAME = "quadruped_support_region_publisher";
   public static final String BIPED_SUPPORT_REGION_PUBLISHER_MODULE_NAME = "bipedal_support_region_publisher";
   public static final String BEHAVIOR_MODULE_NAME = "behavior";
   public static final String REA_MODULE_NAME = "rea";
   public static final String MAPPING_MODULE_NAME = "map";
   public static final String REALSENSE_SLAM_MODULE_NAME = "slam";

   public static final String REA_CUSTOM_REGION_NAME = "custom_region";
   public static final String D435_NAME = "d435";
   public static final String T265_NAME = "t265";
   public static final String MULTISENSE_NAME = "multisense";
   public static final String INPUT = ROS2Topic.INPUT;
   public static final String OUTPUT = ROS2Topic.OUTPUT;

   public static final ROS2Topic<?> IHMC_ROOT = new ROS2Topic<>().withPrefix(IHMC_TOPIC_PREFIX);
   public static final ROS2Topic<?> HUMANOID_CONTROLLER = IHMC_ROOT.withModule(HUMANOID_CONTROL_MODULE_NAME);
   public static final ROS2Topic<?> QUADRUPED_CONTROLLER = IHMC_ROOT.withModule(QUADRUPED_CONTROL_MODULE_NAME);
   public static final ROS2Topic<?> FOOTSTEP_PLANNER = IHMC_ROOT.withModule(FOOTSTEP_PLANNER_MODULE_NAME);
   public static final ROS2Topic<?> CONTINUOUS_PLANNING_TOOLBOX = IHMC_ROOT.withModule(CONTINUOUS_PLANNING_TOOLBOX_MODULE_NAME);
   public static final ROS2Topic<?> FOOTSTEP_POSTPROCESSING_TOOLBOX = IHMC_ROOT.withModule(FOOTSTEP_POSTPROCESSING_TOOLBOX_MODULE_NAME);
   public static final ROS2Topic<?> HEIGHT_QUADTREE_TOOLBOX = IHMC_ROOT.withModule(HEIGHT_QUADTREE_TOOLBOX_MODULE_NAME);
   public static final ROS2Topic<?> FIDUCIAL_DETECTOR_TOOLBOX = IHMC_ROOT.withModule(FIDUCIAL_MODULE_NAME);
   
   public static final ROS2Topic<?> OBJECT_DETECTOR_TOOLBOX = IHMC_ROOT.withModule(OBJECT_DETECTOR_MODULE_NAME);

   public static final ROS2Topic<?> KINEMATICS_TOOLBOX = IHMC_ROOT.withModule(KINEMATICS_TOOLBOX_MODULE_NAME);
   public static final ROS2Topic<?> KINEMATICS_PLANNING_TOOLBOX = IHMC_ROOT.withModule(KINEMATICS_PLANNING_TOOLBOX_MODULE_NAME);
   public static final ROS2Topic<?> KINEMATICS_STREAMING_TOOLBOX = IHMC_ROOT.withModule(KINEMATICS_STREAMING_TOOLBOX_MODULE_NAME);
   public static final ROS2Topic<?> STEP_CONSTRAINT_TOOLBOX = IHMC_ROOT.withModule(STEP_CONSTRAINT_TOOLBOX_MODULE_NAME);
   public static final ROS2Topic<?> WHOLE_BODY_TRAJECTORY_TOOLBOX = IHMC_ROOT.withModule(WHOLE_BODY_TRAJECTORY_TOOLBOX_MODULE_NAME);
   public static final ROS2Topic<?> WALKING_PREVIEW_TOOLBOX = IHMC_ROOT.withModule(WALKING_PREVIEW_TOOLBOX_MODULE_NAME);
   public static final ROS2Topic<?> EXTERNAL_FORCE_ESTIMATION_TOOLBOX = IHMC_ROOT.withModule(EXTERNAL_FORCE_ESTIMATION_TOOLBOX_MODULE_NAME);
   public static final ROS2Topic<?> STEP_TELEOP_TOOLBOX = IHMC_ROOT.withModule(STEP_TELEOP_TOOLBOX_MODULE_NAME);
   public static final ROS2Topic<?> QUADRUPED_SUPPORT_REGION_PUBLISHER = IHMC_ROOT.withModule(QUADRUPED_SUPPORT_REGION_PUBLISHER_MODULE_NAME);
   public static final ROS2Topic<?> BIPED_SUPPORT_REGION_PUBLISHER = IHMC_ROOT.withModule(BIPED_SUPPORT_REGION_PUBLISHER_MODULE_NAME);
   public static final ROS2Topic<?> BEHAVIOR_MODULE = IHMC_ROOT.withModule(BEHAVIOR_MODULE_NAME);
   public static final ROS2Topic<?> REA = IHMC_ROOT.withModule(REA_MODULE_NAME);
   public static final ROS2Topic<?> MAPPING_MODULE = IHMC_ROOT.withModule(MAPPING_MODULE_NAME);
   public static final ROS2Topic<?> REALSENSE_SLAM_MODULE = IHMC_ROOT.withModule(REALSENSE_SLAM_MODULE_NAME);

   public static final ROS2Topic<TextToSpeechPacket> TEXT_STATUS = IHMC_ROOT.withTypeName(TextToSpeechPacket.class);

   public static final ROS2Topic<?> REA_SUPPORT_REGIONS = REA.withSuffix(REA_CUSTOM_REGION_NAME);
   public static final ROS2Topic<PlanarRegionsListMessage> REA_SUPPORT_REGIONS_INPUT
         = REA.withRobot(null).withInput().withType(PlanarRegionsListMessage.class).withSuffix(ROS2Tools.REA_CUSTOM_REGION_NAME);
   public static final ROS2Topic<REAStateRequestMessage> REA_STATE_REQUEST = REA.withInput().withTypeName(REAStateRequestMessage.class);

   public static final ROS2Topic<VideoPacket> VIDEO = IHMC_ROOT.withTypeName(VideoPacket.class);
   public static final ROS2Topic<VideoPacket> D435_VIDEO = IHMC_ROOT.withModule(D435_NAME).withType(VideoPacket.class).withSuffix("video");

   public static final ROS2Topic<LidarScanMessage> MULTISENSE_LIDAR_SCAN = IHMC_ROOT.withTypeName(LidarScanMessage.class);
   public static final ROS2Topic<StereoVisionPointCloudMessage> MULTISENSE_LIDAR_POINT_CLOUD
         = IHMC_ROOT.withModule(MULTISENSE_NAME).withType(StereoVisionPointCloudMessage.class).withSuffix("pointcloud");
   public static final ROS2Topic<StereoVisionPointCloudMessage> MULTISENSE_STEREO_POINT_CLOUD
         = IHMC_ROOT.withTypeName(StereoVisionPointCloudMessage.class);

   public static final ROS2Topic<StereoVisionPointCloudMessage> D435_POINT_CLOUD = IHMC_ROOT.withSuffix(D435_NAME)
                                                                                            .withTypeName(StereoVisionPointCloudMessage.class);
   public static final ROS2Topic<StampedPosePacket> T265_POSE = IHMC_ROOT.withSuffix(T265_NAME).withTypeName(StampedPosePacket.class);

   /** Output regions from Lidar (Multisense) from REA */
   public static final ROS2Topic<PlanarRegionsListMessage> LIDAR_REA_REGIONS = REA.withOutput().withTypeName(PlanarRegionsListMessage.class);
   public static final ROS2Topic<PlanarRegionsListMessage> REALSENSE_REA = ROS2Tools.REA.withOutput().withPrefix("stereo").withTypeName(PlanarRegionsListMessage.class);
   public static final ROS2Topic<PlanarRegionsListMessage> BIPEDAL_SUPPORT_REGIONS = REA_SUPPORT_REGIONS.withTypeName(PlanarRegionsListMessage.class);
   /** Output regions from Atlas Realsense SLAM module */
   public static final ROS2Topic<PlanarRegionsListMessage> REALSENSE_SLAM_REGIONS = REALSENSE_SLAM_MODULE.withOutput().withTypeName(PlanarRegionsListMessage.class);
   /** Output regions from experimental mapping module which assembles the above outputs */
   public static final ROS2Topic<PlanarRegionsListMessage> MAP_REGIONS = MAPPING_MODULE.withOutput().withTypeName(PlanarRegionsListMessage.class);

   public static final Function<String, String> NAMED_BY_TYPE = typeName -> typeName;

   public static ROS2Topic<?> getControllerOutputTopic(String robotName)
   {
      return HUMANOID_CONTROLLER.withRobot(robotName).withOutput();
   }

   public static ROS2Topic<?> getControllerInputTopic(String robotName)
   {
      return HUMANOID_CONTROLLER.withRobot(robotName).withInput();
   }

   public static ROS2Topic<?> getQuadrupedControllerOutputTopic(String robotName)
   {
      return QUADRUPED_CONTROLLER.withRobot(robotName).withOutput();
   }

   public static ROS2Topic<?> getQuadrupedControllerInputTopic(String robotName)
   {
      return QUADRUPED_CONTROLLER.withRobot(robotName).withInput();
   }

   public static <T> ROS2Topic<T> typeNamedTopic(Class<T> messageType)
   {
      return new ROS2Topic<>().withTypeName(messageType).withTypeName();
   }

   public static <T> ROS2Topic<T> typeNamedTopic(Class<T> messageType, ROS2Topic<?> topicName)
   {
      return typeNamedTopic(messageType).withTopic(topicName);
   }

   public static ROS2Topic<RobotConfigurationData> getRobotConfigurationDataTopic(String robotName)
   {
      return typeNamedTopic(RobotConfigurationData.class, getControllerOutputTopic(robotName));
   }

   public static ROS2Topic<StampedPosePacket> getPoseCorrectionTopic(String robotName)
   {
      return getControllerInputTopic(robotName).withTypeName(StampedPosePacket.class);
   }

   public static ROS2Topic<DoorParameterPacket> getDoorParameterTopic()
   {
      return typeNamedTopic(DoorParameterPacket.class, ROS2Tools.IHMC_ROOT);
   }

   public final static ExceptionHandler RUNTIME_EXCEPTION = e ->
   {
      throw new RuntimeException(e);
   };

   public final static String NAMESPACE = "/us/ihmc"; // ? no idea what this does

   private static final RTPSCommunicationFactory FACTORY = new RTPSCommunicationFactory();
   private static final int DOMAIN_ID = FACTORY.getDomainId();
   private static final InetAddress ADDRESS_RESTRICTION = FACTORY.getAddressRestriction();
   private static final ROS2Distro ROS2_DISTRO = ROS2Distro.fromEnvironment();

   /**
    * Creates a ROS2 node that shares the same implementation as a real-time node <b>but that should
    * not be run in a real-time environment</b>.
    *
    * @param pubSubImplementation the implementation to use.
    * @param nodeName the name of the new ROS node.
    * @return the ROS node.
    */
   public static RealtimeROS2Node createRealtimeROS2Node(PubSubImplementation pubSubImplementation, String nodeName)
   {
      return createRealtimeROS2Node(pubSubImplementation, nodeName, RUNTIME_EXCEPTION);
   }

   /**
    * Creates a ROS2 node that shares the same implementation as a real-time node <b>but that should
    * not be run in a real-time environment</b>.
    *
    * @param pubSubImplementation the implementation to use.
    * @param nodeName the name of the new ROS node.
    * @param exceptionHandler how to handle exceptions thrown during the instantiation.
    * @return the ROS node.
    */
   public static RealtimeROS2Node createRealtimeROS2Node(PubSubImplementation pubSubImplementation, String nodeName, ExceptionHandler exceptionHandler)
   {
      return createRealtimeROS2Node(pubSubImplementation, new PeriodicNonRealtimeThreadSchedulerFactory(), nodeName, exceptionHandler);
   }

   /**
    * Creates a ROS2 node that is meant to run in real-time environment if the given
    * {@code periodicThreadSchedulerFactory} is a {@link PeriodicRealtimeThreadSchedulerFactory}.
    *
    * @param pubSubImplementation the implementation to use.
    * @param periodicThreadSchedulerFactory the factory used to create a periodic thread.
    * @param nodeName the name of the new ROS node.
    * @return the ROS node.
    */
   public static RealtimeROS2Node createRealtimeROS2Node(PubSubImplementation pubSubImplementation,
                                                         PeriodicThreadSchedulerFactory periodicThreadSchedulerFactory,
                                                         String nodeName)
   {
      return createRealtimeROS2Node(pubSubImplementation, periodicThreadSchedulerFactory, nodeName, RUNTIME_EXCEPTION);
   }

   /**
    * Creates a ROS2 node that is meant to run in real-time environment if the given
    * {@code periodicThreadSchedulerFactory} is a {@link PeriodicRealtimeThreadSchedulerFactory}.
    *
    * @param pubSubImplementation the implementation to use.
    * @param periodicThreadSchedulerFactory the factory used to create a periodic thread.
    * @param nodeName the name of the new ROS node.
    * @param exceptionHandler how to handle exceptions thrown during the instantiation.
    * @return the ROS node.
    */
   public static RealtimeROS2Node createRealtimeROS2Node(PubSubImplementation pubSubImplementation,
                                                         PeriodicThreadSchedulerFactory periodicThreadSchedulerFactory,
                                                         String nodeName,
                                                         ExceptionHandler exceptionHandler)
   {
      try
      {
         return new RealtimeROS2Node(pubSubImplementation, ROS2_DISTRO, periodicThreadSchedulerFactory, nodeName, NAMESPACE, DOMAIN_ID, ADDRESS_RESTRICTION);
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
         return null;
      }
   }

   public static ROS2Node createInterprocessROS2Node(String nodeName)
   {
      return createROS2Node(PubSubImplementation.FAST_RTPS, nodeName, RUNTIME_EXCEPTION);
   }

   public static ROS2Node createIntraprocessROS2Node(String nodeName)
   {
      return createROS2Node(PubSubImplementation.INTRAPROCESS, nodeName, RUNTIME_EXCEPTION);
   }

   public static ROS2Node createROS2Node(PubSubImplementation pubSubImplementation, String nodeName)
   {
      return createROS2Node(pubSubImplementation, nodeName, RUNTIME_EXCEPTION);
   }

   public static ROS2Node createROS2Node(PubSubImplementation pubSubImplementation, String nodeName, ExceptionHandler exceptionHandler)
   {
      try
      {
         return new ROS2Node(pubSubImplementation, ROS2_DISTRO, nodeName, NAMESPACE, DOMAIN_ID, ADDRESS_RESTRICTION);
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
         return null;
      }
   }

   public static <T> ROS2Subscription<T> createCallbackSubscriptionTypeNamed(ROS2NodeInterface ros2Node,
                                                                             Class<T> messageType,
                                                                             ROS2Topic<?> topicName,
                                                                             NewMessageListener<T> newMessageListener)
   {
      return createCallbackSubscription(ros2Node, typeNamedTopic(messageType).withTopic(topicName), newMessageListener);
   }

   public static <T> ROS2Subscription<T> createCallbackSubscription(ROS2NodeInterface ros2Node, ROS2Topic<T> topic, NewMessageListener<T> newMessageListener)
   {
      return createCallbackSubscription(ros2Node, topic.getType(), topic.getName(), newMessageListener);
   }

   public static <T> ROS2Subscription<T> createCallbackSubscription(ROS2NodeInterface ros2Node,
                                                                    Class<T> messageType,
                                                                    ROS2Topic<?> topicName,
                                                                    NewMessageListener<T> newMessageListener)
   {
      return createCallbackSubscription(ros2Node, messageType, topicName.toString(), newMessageListener);
   }

   public static <T> ROS2Subscription<T> createCallbackSubscription(ROS2NodeInterface ros2Node,
                                                                    Class<T> messageType,
                                                                    String topicName,
                                                                    NewMessageListener<T> newMessageListener)
   {
      return createCallbackSubscription(ros2Node, messageType, topicName, newMessageListener, RUNTIME_EXCEPTION);
   }

   public static <T> ROS2Subscription<T> createCallbackSubscription(ROS2NodeInterface ros2Node,
                                                                    Class<T> messageType,
                                                                    String topicName,
                                                                    NewMessageListener<T> newMessageListener,
                                                                    ExceptionHandler exceptionHandler)
   {
      try
      {
         TopicDataType<T> topicDataType = ROS2TopicNameTools.newMessageTopicDataTypeInstance(messageType);
         return ros2Node.createSubscription(topicDataType, newMessageListener, topicName, ROS2QosProfile.DEFAULT());
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
         return null;
      }
   }

   public static <T> void createCallbackSubscriptionTypeNamed(RealtimeROS2Node realtimeROS2Node,
                                                              Class<T> messageType,
                                                              ROS2Topic<?> topicName,
                                                              NewMessageListener<T> newMessageListener)
   {
      createCallbackSubscription(realtimeROS2Node, typeNamedTopic(messageType).withTopic(topicName), newMessageListener);
   }

   public static <T> void createCallbackSubscription(RealtimeROS2Node realtimeROS2Node, ROS2Topic<T> topic, NewMessageListener<T> newMessageListener)
   {
      createCallbackSubscription(realtimeROS2Node, topic.getType(), topic.getName(), newMessageListener);
   }

   public static <T> void createCallbackSubscription(RealtimeROS2Node realtimeROS2Node,
                                                     Class<T> messageType,
                                                     ROS2Topic<?> topicName,
                                                     NewMessageListener<T> newMessageListener)
   {
      createCallbackSubscription(realtimeROS2Node, messageType, topicName.toString(), newMessageListener);
   }

   public static <T> void createCallbackSubscription(RealtimeROS2Node realtimeROS2Node,
                                                     Class<T> messageType,
                                                     String topicName,
                                                     NewMessageListener<T> newMessageListener)
   {
      createCallbackSubscription(realtimeROS2Node, messageType, topicName, newMessageListener, RUNTIME_EXCEPTION);
   }

   public static <T> void createCallbackSubscription(RealtimeROS2Node realtimeROS2Node,
                                                     Class<T> messageType,
                                                     String topicName,
                                                     NewMessageListener<T> newMessageListener,
                                                     ExceptionHandler exceptionHandler)
   {
      createCallbackSubscription(realtimeROS2Node, messageType, topicName, newMessageListener, ROS2QosProfile.DEFAULT(), exceptionHandler);
   }

   public static <T> void createCallbackSubscription(RealtimeROS2Node realtimeROS2Node,
                                                     Class<T> messageType,
                                                     String topicName,
                                                     NewMessageListener<T> newMessageListener,
                                                     ROS2QosProfile qosProfile,
                                                     ExceptionHandler exceptionHandler)
   {
      try
      {
         TopicDataType<T> topicDataType = ROS2TopicNameTools.newMessageTopicDataTypeInstance(messageType);
         realtimeROS2Node.createCallbackSubscription(topicDataType, topicName, newMessageListener, qosProfile);
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
      }
   }

   public static <T> RealtimeROS2Subscription<T> createQueuedSubscriptionTypeNamed(RealtimeROS2Node realtimeROS2Node,
                                                                                   Class<T> messageType,
                                                                                   ROS2Topic<?> topicName)
   {
      return createQueuedSubscription(realtimeROS2Node, typeNamedTopic(messageType).withTopic(topicName));
   }

   public static <T> RealtimeROS2Subscription<T> createQueuedSubscription(RealtimeROS2Node realtimeROS2Node, ROS2Topic<T> topic)
   {
      return createQueuedSubscription(realtimeROS2Node, topic.getType(), topic.getName());
   }

   public static <T> RealtimeROS2Subscription<T> createQueuedSubscription(RealtimeROS2Node realtimeROS2Node, Class<T> messageType, ROS2Topic<?> topicName)
   {
      return createQueuedSubscription(realtimeROS2Node, messageType, topicName.toString());
   }

   public static <T> RealtimeROS2Subscription<T> createQueuedSubscription(RealtimeROS2Node realtimeROS2Node, Class<T> messageType, String topicName)
   {
      return createQueuedSubscription(realtimeROS2Node, messageType, topicName, RUNTIME_EXCEPTION);
   }

   public static <T> RealtimeROS2Subscription<T> createQueuedSubscription(RealtimeROS2Node realtimeROS2Node,
                                                                          Class<T> messageType,
                                                                          String topicName,
                                                                          ExceptionHandler exceptionHandler)
   {
      try
      {
         TopicDataType<T> topicDataType = ROS2TopicNameTools.newMessageTopicDataTypeInstance(messageType);
         return realtimeROS2Node.createQueuedSubscription(topicDataType, topicName, ROS2QosProfile.DEFAULT(), 10);
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
         return null;
      }
   }

   public static <T> IHMCRealtimeROS2Publisher<T> createPublisherTypeNamed(RealtimeROS2Node realtimeROS2Node,
                                                                           Class<T> messageType,
                                                                           ROS2Topic<?> topicName)
   {
      return createPublisher(realtimeROS2Node, typeNamedTopic(messageType).withTopic(topicName));
   }

   public static <T> IHMCRealtimeROS2Publisher<T> createPublisher(RealtimeROS2Node realtimeROS2Node, ROS2Topic<T> topic)
   {
      return createPublisher(realtimeROS2Node, topic.getType(), topic.getName());
   }

   public static <T> IHMCRealtimeROS2Publisher<T> createPublisher(RealtimeROS2Node realtimeROS2Node, Class<T> messageType, ROS2Topic<?> topicName)
   {
      return createPublisher(realtimeROS2Node, messageType, topicName.toString());
   }

   public static <T> IHMCRealtimeROS2Publisher<T> createPublisher(RealtimeROS2Node realtimeROS2Node, Class<T> messageType, String topicName)
   {
      return createPublisher(realtimeROS2Node, messageType, topicName, RUNTIME_EXCEPTION);
   }

   public static <T> IHMCRealtimeROS2Publisher<T> createPublisher(RealtimeROS2Node realtimeROS2Node,
                                                                  Class<T> messageType,
                                                                  String topicName,
                                                                  ExceptionHandler exceptionHandler)
   {
      try
      {
         TopicDataType<T> topicDataType = ROS2TopicNameTools.newMessageTopicDataTypeInstance(messageType);
         return new IHMCRealtimeROS2Publisher<T>(realtimeROS2Node.createPublisher(topicDataType, topicName, ROS2QosProfile.DEFAULT(), 10));
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
         return null;
      }
   }

   public static <T> IHMCROS2Publisher<T> createPublisherTypeNamed(ROS2NodeInterface ros2Node, Class<T> messageType, ROS2Topic<?> topicName)
   {
      return createPublisher(ros2Node, typeNamedTopic(messageType).withTopic(topicName));
   }

   public static <T> IHMCROS2Publisher<T> createPublisher(ROS2NodeInterface ros2Node, ROS2Topic<T> topic)
   {
      return createPublisher(ros2Node, topic.getType(), topic.getName());
   }

   public static <T> IHMCROS2Publisher<T> createPublisher(ROS2NodeInterface ros2Node, ROS2Topic<T> topic, ROS2QosProfile qosProfile)
   {
      return createPublisher(ros2Node, topic.getType(), topic.getName(), qosProfile, RUNTIME_EXCEPTION);
   }

   public static <T> IHMCROS2Publisher<T> createPublisher(ROS2NodeInterface ros2Node, Class<T> messageType, ROS2Topic<?> topicName)
   {
      return createPublisher(ros2Node, messageType, topicName.toString());
   }

   public static <T> IHMCROS2Publisher<T> createPublisher(ROS2NodeInterface ros2Node, Class<T> messageType, String topicName)
   {
      return createPublisher(ros2Node, messageType, topicName, RUNTIME_EXCEPTION);
   }

   public static <T> IHMCROS2Publisher<T> createPublisher(ROS2NodeInterface ros2Node, Class<T> messageType, String topicName, ExceptionHandler exceptionHandler)
   {
      return createPublisher(ros2Node, messageType, topicName, ROS2QosProfile.DEFAULT(), exceptionHandler);
   }

   public static <T> IHMCROS2Publisher<T> createPublisher(ROS2NodeInterface ros2Node,
                                                          Class<T> messageType,
                                                          String topicName,
                                                          ROS2QosProfile qosProfile,
                                                          ExceptionHandler exceptionHandler)
   {
      try
      {
         TopicDataType<T> topicDataType = ROS2TopicNameTools.newMessageTopicDataTypeInstance(messageType);
         return new IHMCROS2Publisher<T>(ros2Node.createPublisher(topicDataType, topicName, qosProfile));
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
         return null;
      }
   }
}
