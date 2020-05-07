package us.ihmc.communication;

import java.io.IOException;
import java.net.InetAddress;

import us.ihmc.commons.exception.ExceptionHandler;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.TopicDataType;
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
   public static final String KINEMATICS_TOOLBOX_MODULE_NAME = "toolbox/ik";
   public static final String KINEMATICS_PLANNING_TOOLBOX_MODULE_NAME = "toolbox/ik_planning";
   public static final String KINEMATICS_STREAMING_TOOLBOX_MODULE_NAME = "toolbox/ik_streaming";
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
   public static final String INPUT = ROS2TopicName.INPUT;
   public static final String OUTPUT = ROS2TopicName.OUTPUT;

   public static final ROS2TopicName IHMC_ROOT = new ROS2TopicName().withPrefix(IHMC_TOPIC_PREFIX);
   public static final ROS2TopicName HUMANOID_CONTROLLER = IHMC_ROOT.withModule(HUMANOID_CONTROL_MODULE_NAME);
   public static final ROS2TopicName QUADRUPED_CONTROLLER = IHMC_ROOT.withModule(QUADRUPED_CONTROL_MODULE_NAME);
   public static final ROS2TopicName FOOTSTEP_PLANNER = IHMC_ROOT.withModule(FOOTSTEP_PLANNER_MODULE_NAME);
   public static final ROS2TopicName CONTINUOUS_PLANNING_TOOLBOX = IHMC_ROOT.withModule(CONTINUOUS_PLANNING_TOOLBOX_MODULE_NAME);
   public static final ROS2TopicName FOOTSTEP_POSTPROCESSING_TOOLBOX = IHMC_ROOT.withModule(FOOTSTEP_POSTPROCESSING_TOOLBOX_MODULE_NAME);
   public static final ROS2TopicName HEIGHT_QUADTREE_TOOLBOX = IHMC_ROOT.withModule(HEIGHT_QUADTREE_TOOLBOX_MODULE_NAME);
   public static final ROS2TopicName KINEMATICS_TOOLBOX = IHMC_ROOT.withModule(KINEMATICS_TOOLBOX_MODULE_NAME);
   public static final ROS2TopicName KINEMATICS_PLANNING_TOOLBOX = IHMC_ROOT.withModule(KINEMATICS_PLANNING_TOOLBOX_MODULE_NAME);
   public static final ROS2TopicName KINEMATICS_STREAMING_TOOLBOX = IHMC_ROOT.withModule(KINEMATICS_STREAMING_TOOLBOX_MODULE_NAME);
   public static final ROS2TopicName WHOLE_BODY_TRAJECTORY_TOOLBOX = IHMC_ROOT.withModule(WHOLE_BODY_TRAJECTORY_TOOLBOX_MODULE_NAME);
   public static final ROS2TopicName WALKING_PREVIEW_TOOLBOX = IHMC_ROOT.withModule(WALKING_PREVIEW_TOOLBOX_MODULE_NAME);
   public static final ROS2TopicName EXTERNAL_FORCE_ESTIMATION_TOOLBOX = IHMC_ROOT.withModule(EXTERNAL_FORCE_ESTIMATION_TOOLBOX_MODULE_NAME);
   public static final ROS2TopicName STEP_TELEOP_TOOLBOX = IHMC_ROOT.withModule(STEP_TELEOP_TOOLBOX_MODULE_NAME);
   public static final ROS2TopicName QUADRUPED_SUPPORT_REGION_PUBLISHER = IHMC_ROOT.withModule(QUADRUPED_SUPPORT_REGION_PUBLISHER_MODULE_NAME);
   public static final ROS2TopicName BIPED_SUPPORT_REGION_PUBLISHER = IHMC_ROOT.withModule(BIPED_SUPPORT_REGION_PUBLISHER_MODULE_NAME);
   public static final ROS2TopicName BEHAVIOR_MODULE = IHMC_ROOT.withModule(BEHAVIOR_MODULE_NAME);
   public static final ROS2TopicName REA = IHMC_ROOT.withModule(REA_MODULE_NAME);
   public static final ROS2TopicName MAPPING_MODULE = IHMC_ROOT.withModule(MAPPING_MODULE_NAME);
   public static final ROS2TopicName REALSENSE_SLAM_MAP = IHMC_ROOT.withModule(REALSENSE_SLAM_MODULE_NAME);

   public static final ROS2TopicName REA_SUPPORT_REGIONS = REA.withSuffix(REA_CUSTOM_REGION_NAME);

   public static ROS2TopicName getControllerOutputTopicName(String robotName)
   {
      return HUMANOID_CONTROLLER.withRobot(robotName).withOutput();
   }

   public static ROS2TopicName getControllerInputTopicName(String robotName)
   {
      return HUMANOID_CONTROLLER.withRobot(robotName).withInput();
   }

   public static ROS2TopicName getQuadrupedControllerOutputTopicName(String robotName)
   {
      return QUADRUPED_CONTROLLER.withRobot(robotName).withOutput();
   }

   public static ROS2TopicName getQuadrupedControllerInputTopicName(String robotName)
   {
      return QUADRUPED_CONTROLLER.withRobot(robotName).withInput();
   }

   public final static ExceptionHandler RUNTIME_EXCEPTION = e ->
   {
      throw new RuntimeException(e);
   };

   public final static String NAMESPACE = "/us/ihmc"; // ? no idea what this does

   private static final RTPSCommunicationFactory FACTORY = new RTPSCommunicationFactory();
   private static final int DOMAIN_ID = FACTORY.getDomainId();
   private static final InetAddress ADDRESS_RESTRICTION = FACTORY.getAddressRestriction();
   private static final Ros2Distro ROS2_DISTRO = Ros2Distro.fromEnvironment();

   /**
    * Creates a ROS2 node that shares the same implementation as a real-time node <b>but that should
    * not be run in a real-time environment</b>.
    *
    * @param pubSubImplementation the implementation to use.
    * @param nodeName the name of the new ROS node.
    * @return the ROS node.
    */
   public static RealtimeRos2Node createRealtimeRos2Node(PubSubImplementation pubSubImplementation, String nodeName)
   {
      return createRealtimeRos2Node(pubSubImplementation, nodeName, RUNTIME_EXCEPTION);
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
   public static RealtimeRos2Node createRealtimeRos2Node(PubSubImplementation pubSubImplementation, String nodeName, ExceptionHandler exceptionHandler)
   {
      return createRealtimeRos2Node(pubSubImplementation, new PeriodicNonRealtimeThreadSchedulerFactory(), nodeName, exceptionHandler);
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
   public static RealtimeRos2Node createRealtimeRos2Node(PubSubImplementation pubSubImplementation,
                                                         PeriodicThreadSchedulerFactory periodicThreadSchedulerFactory,
                                                         String nodeName)
   {
      return createRealtimeRos2Node(pubSubImplementation, periodicThreadSchedulerFactory, nodeName, RUNTIME_EXCEPTION);
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
   public static RealtimeRos2Node createRealtimeRos2Node(PubSubImplementation pubSubImplementation,
                                                         PeriodicThreadSchedulerFactory periodicThreadSchedulerFactory,
                                                         String nodeName,
                                                         ExceptionHandler exceptionHandler)
   {
      try
      {
         return new RealtimeRos2Node(pubSubImplementation, ROS2_DISTRO, periodicThreadSchedulerFactory, nodeName, NAMESPACE, DOMAIN_ID, ADDRESS_RESTRICTION);
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
         return null;
      }
   }

   public static Ros2Node createRos2Node(PubSubImplementation pubSubImplementation, String nodeName)
   {
      return createRos2Node(pubSubImplementation, nodeName, RUNTIME_EXCEPTION);
   }

   public static Ros2Node createRos2Node(PubSubImplementation pubSubImplementation, String nodeName, ExceptionHandler exceptionHandler)
   {
      try
      {
         return new Ros2Node(pubSubImplementation, ROS2_DISTRO, nodeName, NAMESPACE, DOMAIN_ID, ADDRESS_RESTRICTION);
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
         return null;
      }
   }

   public static <T> Ros2Subscription<T> createCallbackSubscription(Ros2NodeInterface ros2Node,
                                                                    Class<T> messageType,
                                                                    ROS2TopicName topicName,
                                                                    NewMessageListener<T> newMessageListener)
   {
      return createCallbackSubscription(ros2Node, messageType, topicName.withType(messageType).toString(), newMessageListener);
   }

   public static <T> Ros2Subscription<T> createCallbackSubscription(Ros2NodeInterface ros2Node,
                                                                    Class<T> messageType,
                                                                    String topicName,
                                                                    NewMessageListener<T> newMessageListener)
   {
      return createCallbackSubscription(ros2Node, messageType, topicName, newMessageListener, RUNTIME_EXCEPTION);
   }

   public static <T> Ros2Subscription<T> createCallbackSubscription(Ros2NodeInterface ros2Node,
                                                                    Class<T> messageType,
                                                                    String topicName,
                                                                    NewMessageListener<T> newMessageListener,
                                                                    ExceptionHandler exceptionHandler)
   {
      try
      {
         TopicDataType<T> topicDataType = ROS2TopicNameTools.newMessageTopicDataTypeInstance(messageType);
         return ros2Node.createSubscription(topicDataType, newMessageListener, topicName, Ros2QosProfile.DEFAULT());
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
         return null;
      }
   }

   public static <T> void createCallbackSubscription(RealtimeRos2Node realtimeRos2Node,
                                                     Class<T> messageType,
                                                     ROS2TopicName topicName,
                                                     NewMessageListener<T> newMessageListener)
   {
      createCallbackSubscription(realtimeRos2Node, messageType, topicName.withType(messageType).toString(), newMessageListener);
   }

   public static <T> void createCallbackSubscription(RealtimeRos2Node realtimeRos2Node,
                                                     Class<T> messageType,
                                                     String topicName,
                                                     NewMessageListener<T> newMessageListener)
   {
      createCallbackSubscription(realtimeRos2Node, messageType, topicName, newMessageListener, RUNTIME_EXCEPTION);
   }

   public static <T> void createCallbackSubscription(RealtimeRos2Node realtimeRos2Node,
                                                     Class<T> messageType,
                                                     String topicName,
                                                     NewMessageListener<T> newMessageListener,
                                                     ExceptionHandler exceptionHandler)
   {
      try
      {
         TopicDataType<T> topicDataType = ROS2TopicNameTools.newMessageTopicDataTypeInstance(messageType);
         realtimeRos2Node.createCallbackSubscription(topicDataType, topicName, newMessageListener, Ros2QosProfile.DEFAULT());
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
      }
   }

   public static <T> RealtimeRos2Subscription<T> createQueuedSubscription(RealtimeRos2Node realtimeRos2Node,
                                                                          Class<T> messageType,
                                                                          ROS2TopicName topicName)
   {
      return createQueuedSubscription(realtimeRos2Node, messageType, topicName.withType(messageType).toString(), RUNTIME_EXCEPTION);
   }

   public static <T> RealtimeRos2Subscription<T> createQueuedSubscription(RealtimeRos2Node realtimeRos2Node, Class<T> messageType, String topicName)
   {
      return createQueuedSubscription(realtimeRos2Node, messageType, topicName, RUNTIME_EXCEPTION);
   }

   public static <T> RealtimeRos2Subscription<T> createQueuedSubscription(RealtimeRos2Node realtimeRos2Node,
                                                                          Class<T> messageType,
                                                                          String topicName,
                                                                          ExceptionHandler exceptionHandler)
   {
      try
      {
         TopicDataType<T> topicDataType = ROS2TopicNameTools.newMessageTopicDataTypeInstance(messageType);
         return realtimeRos2Node.createQueuedSubscription(topicDataType, topicName, Ros2QosProfile.DEFAULT(), 10);
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
         return null;
      }
   }

   public static <T> IHMCRealtimeROS2Publisher<T> createPublisher(RealtimeRos2Node realtimeRos2Node,
                                                                  Class<T> messageType,
                                                                  ROS2TopicName topicName)
   {
      return createPublisher(realtimeRos2Node, messageType, topicName.withType(messageType).toString());
   }

   public static <T> IHMCRealtimeROS2Publisher<T> createPublisher(RealtimeRos2Node realtimeRos2Node, Class<T> messageType, String topicName)
   {
      return createPublisher(realtimeRos2Node, messageType, topicName, RUNTIME_EXCEPTION);
   }

   public static <T> IHMCRealtimeROS2Publisher<T> createPublisher(RealtimeRos2Node realtimeRos2Node,
                                                                  Class<T> messageType,
                                                                  String topicName,
                                                                  ExceptionHandler exceptionHandler)
   {
      try
      {
         TopicDataType<T> topicDataType = ROS2TopicNameTools.newMessageTopicDataTypeInstance(messageType);
         return new IHMCRealtimeROS2Publisher<T>(realtimeRos2Node.createPublisher(topicDataType, topicName, Ros2QosProfile.DEFAULT(), 10));
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
         return null;
      }
   }

   public static <T> IHMCROS2Publisher<T> createPublisher(Ros2NodeInterface ros2Node, Class<T> messageType, ROS2TopicName topicName)
   {
      return createPublisher(ros2Node, messageType, topicName.withType(messageType).toString());
   }

   public static <T> IHMCROS2Publisher<T> createPublisher(Ros2NodeInterface ros2Node, Class<T> messageType, String topicName)
   {
      return createPublisher(ros2Node, messageType, topicName, RUNTIME_EXCEPTION);
   }

   public static <T> IHMCROS2Publisher<T> createPublisher(Ros2NodeInterface ros2Node, Class<T> messageType, String topicName, ExceptionHandler exceptionHandler)
   {
      try
      {
         TopicDataType<T> topicDataType = ROS2TopicNameTools.newMessageTopicDataTypeInstance(messageType);
         return new IHMCROS2Publisher<T>(ros2Node.createPublisher(topicDataType, topicName, Ros2QosProfile.DEFAULT()));
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
         return null;
      }
   }
}
