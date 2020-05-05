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
   public static final String IHMC_TOPIC_PREFIX = "/ihmc";
   public static final String OUTPUT_TOPIC_QUALIFIER = "/output";
   public static final String INPUT_TOPIC_QUALIFIER = "/input";

   public static final String HUMANOID_CONTROLLER_NODE_NAME = "ihmc_controller";
   public static final String HUMANOID_KINEMATICS_CONTROLLER_NODE_NAME = "kinematics_ihmc_controller";
   public static final String REA_NODE_NAME = "REA_module";
   public static final String MAPPING_MODULE_NODE_NAME = "mapping_module";
   public static final String STEREO_REA_NODE_NAME = "SREA_module";
   public static final String LLAMA_NODE_NAME = "llama_network";
   public static final String FOOTSTEP_PLANNER_NODE_NAME = "ihmc_multi_stage_footstep_planning_module";
   public static final String BEHAVIOR_MODULE_NODE_NAME = "behavior_module";

   public static final String HUMANOID_CONTROL_MODULE_NAME = "/humanoid_control";
   public static final String QUADRUPED_CONTROL_MODULE_NAME = "/quadruped_control";
   public static final String FOOTSTEP_PLANNER_MODULE_NAME = "/toolbox/footstep_plan";
   public static final String CONTINUOUS_PLANNING_TOOLBOX_MODULE_NAME = "/toolbox/continuous_planning";
   public static final String FOOTSTEP_POSTPROCESSING_TOOLBOX_MODULE_NAME = "/toolbox/footstep_postprocessing";
   public static final String HEIGHT_QUADTREE_TOOLBOX_MODULE_NAME = "/toolbox/height_quad_tree";
   public static final String KINEMATICS_TOOLBOX_MODULE_NAME = "/toolbox/ik";
   public static final String KINEMATICS_PLANNING_TOOLBOX_MODULE_NAME = "/toolbox/ik_planning";
   public static final String KINEMATICS_STREAMING_TOOLBOX_MODULE_NAME = "/toolbox/ik_streaming";
   public static final String WHOLE_BODY_TRAJECTORY_TOOLBOX_MODULE_NAME = "/toolbox/ik_trajectory";
   public static final String WALKING_PREVIEW_TOOLBOX_MODULE_NAME = "/toolbox/walking_controller_preview";
   public static final String EXTERNAL_FORCE_ESTIMATION_TOOLBOX_MODULE_NAME = "/toolbox/external_force_estimation";
   public static final String STEP_TELEOP_TOOLBOX_MODULE_NAME = "/toolbox/teleop/step_teleop";
   public static final String QUADRUPED_SUPPORT_REGION_PUBLISHER_MODULE_NAME = "/quadruped_support_region_publisher";
   public static final String BIPED_SUPPORT_REGION_PUBLISHER_MODULE_NAME = "/bipedal_support_region_publisher";
   public static final String BEHAVIOR_MODULE_NAME = "/behavior";
   public static final String REA_MODULE_NAME = "/rea";
   public static final String STEREO_REA_MODULE_NAME = "/srea";
   public static final String MAPPING_MODULE_NAME = "/map";
   public static final String REALSENSE_SLAM_MODULE_NAME = "/slam";

   public static final String REA_CUSTOM_REGION_NAME = "/custom_region";

   public static final ROS2TopicName IHMC_ROOT = new ROS2TopicName().prefix(IHMC_TOPIC_PREFIX);
   public static final ROS2TopicName HUMANOID_CONTROLLER = IHMC_ROOT.module(HUMANOID_CONTROL_MODULE_NAME);
   public static final ROS2TopicName QUADRUPED_CONTROLLER = IHMC_ROOT.module(QUADRUPED_CONTROL_MODULE_NAME);
   public static final ROS2TopicName FOOTSTEP_PLANNER = IHMC_ROOT.module(FOOTSTEP_PLANNER_MODULE_NAME);
   public static final ROS2TopicName CONTINUOUS_PLANNING_TOOLBOX = IHMC_ROOT.module(CONTINUOUS_PLANNING_TOOLBOX_MODULE_NAME);
   public static final ROS2TopicName FOOTSTEP_POSTPROCESSING_TOOLBOX = IHMC_ROOT.module(FOOTSTEP_POSTPROCESSING_TOOLBOX_MODULE_NAME);
   public static final ROS2TopicName HEIGHT_QUADTREE_TOOLBOX = IHMC_ROOT.module(HEIGHT_QUADTREE_TOOLBOX_MODULE_NAME);
   public static final ROS2TopicName KINEMATICS_TOOLBOX = IHMC_ROOT.module(KINEMATICS_TOOLBOX_MODULE_NAME);
   public static final ROS2TopicName KINEMATICS_PLANNING_TOOLBOX = IHMC_ROOT.module(KINEMATICS_PLANNING_TOOLBOX_MODULE_NAME);
   public static final ROS2TopicName KINEMATICS_STREAMING_TOOLBOX = IHMC_ROOT.module(KINEMATICS_STREAMING_TOOLBOX_MODULE_NAME);
   public static final ROS2TopicName WHOLE_BODY_TRAJECTORY_TOOLBOX = IHMC_ROOT.module(WHOLE_BODY_TRAJECTORY_TOOLBOX_MODULE_NAME);
   public static final ROS2TopicName WALKING_PREVIEW_TOOLBOX = IHMC_ROOT.module(WALKING_PREVIEW_TOOLBOX_MODULE_NAME);
   public static final ROS2TopicName EXTERNAL_FORCE_ESTIMATION_TOOLBOX = IHMC_ROOT.module(EXTERNAL_FORCE_ESTIMATION_TOOLBOX_MODULE_NAME);
   public static final ROS2TopicName STEP_TELEOP_TOOLBOX = IHMC_ROOT.module(STEP_TELEOP_TOOLBOX_MODULE_NAME);
   public static final ROS2TopicName QUADRUPED_SUPPORT_REGION_PUBLISHER = IHMC_ROOT.module(QUADRUPED_SUPPORT_REGION_PUBLISHER_MODULE_NAME);
   public static final ROS2TopicName BIPED_SUPPORT_REGION_PUBLISHER = IHMC_ROOT.module(BIPED_SUPPORT_REGION_PUBLISHER_MODULE_NAME);
   public static final ROS2TopicName BEHAVIOR_MODULE = IHMC_ROOT.module(BEHAVIOR_MODULE_NAME);
   public static final ROS2TopicName REA = IHMC_ROOT.module(REA_MODULE_NAME);
   public static final ROS2TopicName STEREO_REA = IHMC_ROOT.module(STEREO_REA_MODULE_NAME);
   public static final ROS2TopicName MAPPING_MODULE = IHMC_ROOT.module(MAPPING_MODULE_NAME);
   public static final ROS2TopicName REALSENSE_SLAM_MAP = IHMC_ROOT.module(REALSENSE_SLAM_MODULE_NAME);

   public static final ROS2TopicName REA_SUPPORT_REGIONS = REA.name(REA_CUSTOM_REGION_NAME);

   public final static ExceptionHandler RUNTIME_EXCEPTION = e -> {
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
                                                         PeriodicThreadSchedulerFactory periodicThreadSchedulerFactory, String nodeName)
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
                                                         PeriodicThreadSchedulerFactory periodicThreadSchedulerFactory, String nodeName,
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

   @Deprecated
   public static <T> Ros2Subscription<T> createCallbackSubscription(Ros2NodeInterface ros2Node,
                                                                    Class<T> messageType,
                                                                    ROS2MessageTopicNameGenerator topicNameGenerator,
                                                                    NewMessageListener<T> newMessageListener)
   {
      String topicName = topicNameGenerator.generateTopicName(messageType);
      return createCallbackSubscription(ros2Node, messageType, topicName, newMessageListener);
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

   @Deprecated
   public static <T> Ros2QueuedSubscription<T> createQueuedSubscription(Ros2NodeInterface ros2Node,
                                                                        Class<T> messageType,
                                                                        ROS2MessageTopicNameGenerator topicNameGenerator)
   {
      String topicName = topicNameGenerator.generateTopicName(messageType);
      return createQueuedSubscription(ros2Node, messageType, topicName, RUNTIME_EXCEPTION);
   }

   public static <T> Ros2QueuedSubscription<T> createQueuedSubscription(Ros2NodeInterface ros2Node, Class<T> messageType, String topicName)
   {
      return createQueuedSubscription(ros2Node, messageType, topicName, RUNTIME_EXCEPTION);
   }

   public static <T> Ros2QueuedSubscription<T> createQueuedSubscription(Ros2NodeInterface ros2Node,
                                                                        Class<T> messageType,
                                                                        String topicName,
                                                                        ExceptionHandler exceptionHandler)
   {
      try
      {
         TopicDataType<T> topicDataType = ROS2TopicNameTools.newMessageTopicDataTypeInstance(messageType);
         Ros2QueuedSubscription<T> ros2QueuedSubscription = new Ros2QueuedSubscription<>(topicDataType, 10);
         ros2QueuedSubscription.setRos2Subscription(ros2Node.createSubscription(topicDataType, ros2QueuedSubscription, topicName, Ros2QosProfile.DEFAULT()));
         return ros2QueuedSubscription;
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
         return null;
      }
   }

   @Deprecated
   public static <T> void createCallbackSubscription(RealtimeRos2Node realtimeRos2Node,
                                                     Class<T> messageType,
                                                     ROS2MessageTopicNameGenerator topicNameGenerator,
                                                     NewMessageListener<T> newMessageListener)
   {
      String topicName = topicNameGenerator.generateTopicName(messageType);
      createCallbackSubscription(realtimeRos2Node, messageType, topicName, newMessageListener);
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

   @Deprecated
   public static <T> RealtimeRos2Subscription<T> createQueuedSubscription(RealtimeRos2Node realtimeRos2Node,
                                                                          Class<T> messageType,
                                                                          ROS2MessageTopicNameGenerator topicNameGenerator)
   {
      String topicName = topicNameGenerator.generateTopicName(messageType);
      return createQueuedSubscription(realtimeRos2Node, messageType, topicName, RUNTIME_EXCEPTION);
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

   @Deprecated
   public static <T> IHMCRealtimeROS2Publisher<T> createPublisher(RealtimeRos2Node realtimeRos2Node,
                                                                  Class<T> messageType,
                                                                  ROS2MessageTopicNameGenerator topicNameGenerator)
   {
      String topicName = topicNameGenerator.generateTopicName(messageType);
      return createPublisher(realtimeRos2Node, messageType, topicName);
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

   @Deprecated
   public static <T> IHMCROS2Publisher<T> createPublisher(Ros2NodeInterface ros2Node, Class<T> messageType, ROS2MessageTopicNameGenerator topicNameGenerator)
   {
      String topicName = topicNameGenerator.generateTopicName(messageType);
      return createPublisher(ros2Node, messageType, topicName);
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

   /**
    * Creates a default topic name generator that uses {@value #IHMC_TOPIC_PREFIX} as prefix.
    * <p>
    * This generator is not great at all as the topic name does not include the name of the robot,
    * the name of the module, nor info about whether the topic is an input or output of the module
    * declaring it.
    * </p>
    * <p>
    * Here is a couple examples for this generator:
    * <ul>
    * <li>For {@code TextToSpeechPacket} this generates the topic name:
    * {@code "/ihmc/text_to_speech"}.
    * <li>For {@code ArmTrajectoryMessage} this generates the topic name:
    * {@code "/ihmc/arm_trajectory"}.
    * </ul>
    * </p>
    *
    * @deprecated
    * @return the default generator.
    */
   public static ROS2MessageTopicNameGenerator getDefaultTopicNameGenerator()
   {
      return ROS2Tools::generateDefaultTopicName;
   }

   /**
    * Creates a default topic name generator that uses {@value #IHMC_TOPIC_PREFIX} plus the name
    * of the robot as prefix.
    * <p>
    * This generator is not great as the topic name does not include the name of the module, nor
    * info about whether the topic is an input or output of the module declaring it.
    * </p>
    * <p>
    * Here is a couple examples for this generator:
    * <ul>
    * <li>For {@code TextToSpeechPacket} when running Atlas this generates the topic name:
    * {@code "/ihmc/atlas/text_to_speech"}.
    * <li>For {@code ArmTrajectoryMessage} when running Valkyrie this generates the topic name:
    * {@code "/ihmc/valkyrie/arm_trajectory"}.
    * </ul>
    * </p>
    *
    * @deprecated
    * @return the default generator.
    */
   public static ROS2MessageTopicNameGenerator getDefaultTopicNameGenerator(String robotName)
   {
      return messageType -> generateDefaultTopicName(messageType, robotName);
   }

   /**
    * Creates a default topic name generator that uses {@value #IHMC_TOPIC_PREFIX} plus the name
    * of the robot as prefix.
    * <p>
    * This generator is not great as the topic name does not include the name of the module, nor
    * info about whether the topic is an input or output of the module declaring it.
    * </p>
    * <p>
    * Here is a couple examples for this generator:
    * <ul>
    * <li>For {@code TextToSpeechPacket} when running Atlas this generates the topic name:
    * {@code "/ihmc/atlas/text_to_speech"}.
    * <li>For {@code ArmTrajectoryMessage} when running Valkyrie this generates the topic name:
    * {@code "/ihmc/valkyrie/arm_trajectory"}.
    * </ul>
    * </p>
    *
    * @deprecated
    * @return the generator.
    */
   public static ROS2MessageTopicNameGenerator getTopicNameGenerator(String robotName, String moduleName, ROS2TopicQualifier qualifier)
   {
      return messageType -> generateDefaultTopicName(messageType, robotName, moduleName, qualifier);
   }

   /**
    * Generates a default topic name using the class name of the message, for instance:<br>
    * For {@code TextToSpeechPacket} this generates the topic name: {@code "/ihmc/text_to_speech"}.
    *
    * @deprecated
    * @param messageClass the class of the message to generate the topic name for.
    * @return the topic name.
    */
   public static String generateDefaultTopicName(Class<?> messageClass)
   {
      return generateDefaultTopicName(messageClass, null, null, null);
   }

   /**
    * Generates a default topic name using the class name of the message, for instance:<br>
    * For {@code TextToSpeechPacket} when running Valkyrie this generates the topic name:
    * {@code "/ihmc/valkyrie/text_to_speech"}.
    *
    * @deprecated
    * @param messageClass the class of the message to generate the topic name for.
    * @return the topic name.
    */
   public static String generateDefaultTopicName(Class<?> messageClass, String robotName)
   {
      return generateDefaultTopicName(messageClass, robotName, null, null);
   }

   /**
    * Generates a topic name in a similar way to
    * {@link #generateDefaultTopicName(Class, String)}:<br>
    * For {@code TextToSpeechPacket} when running Valkyrie this generates the topic name:<br>
    * {@code "/ihmc/valkyrie/" + moduleName.toLowerCase() + qualifier + "/text_to_speech"}.
    *
    *
    * @deprecated
    * @param messageClass the class of the message to generate the topic name for.
    * @return the topic name.
    */
   public static String generateDefaultTopicName(Class<?> messageClass, String robotName, String moduleName, ROS2TopicQualifier qualifier)
   {
      ROS2TopicName topicName = IHMC_ROOT.type(messageClass).robot(robotName).module(moduleName);

      if (qualifier != null)
      {
         if (qualifier.equals(ROS2TopicQualifier.INPUT)) // TODO fix this if ROS2TopicName gets a better API
         {
            topicName = topicName.input();
         }
         else
         {
            topicName = topicName.output();
         }
      }

      return topicName.toString();
   }
}
