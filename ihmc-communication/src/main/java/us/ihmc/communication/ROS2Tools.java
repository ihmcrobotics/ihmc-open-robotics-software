package us.ihmc.communication;

import com.eprosima.xmlschemas.fastrtps_profiles.ReliabilityQosKindType;
import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import controller_msgs.msg.dds.HandJointAnglePacket;
import controller_msgs.msg.dds.SakeHandDesiredCommandMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.SakeHandStatusMessage;
import ihmc_common_msgs.msg.dds.StampedPosePacket;
import ihmc_common_msgs.msg.dds.TextToSpeechPacket;
import mission_control_msgs.msg.dds.*;
import perception_msgs.msg.dds.DoorParameterPacket;
import std_msgs.msg.dds.Empty;
import std_msgs.msg.dds.Float64;
import toolbox_msgs.msg.dds.*;
import us.ihmc.commons.exception.ExceptionHandler;
import us.ihmc.commons.thread.Notification;
import us.ihmc.pubsub.Domain;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.*;
import us.ihmc.tools.thread.SwapReference;
import us.ihmc.util.PeriodicNonRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicThreadSchedulerFactory;

import java.io.IOException;
import java.net.InetAddress;
import java.util.UUID;
import java.util.function.Consumer;

public class ROS2Tools
{
   public static final String IHMC_TOPIC_PREFIX = "ihmc";

   public static final String HUMANOID_CONTROLLER_NODE_NAME = "ihmc_controller";
   public static final String HUMANOID_KINEMATICS_CONTROLLER_NODE_NAME = "kinematics_ihmc_controller";
   public static final String LLAMA_NODE_NAME = "llama_network";
   public static final String FOOTSTEP_PLANNER_NODE_NAME = "ihmc_multi_stage_footstep_planning_module";
   public static final String BEHAVIOR_MODULE_NODE_NAME = "behavior_module";

   public static final String HUMANOID_CONTROL_MODULE_NAME = "humanoid_control";
   public static final String QUADRUPED_CONTROL_MODULE_NAME = "quadruped_control";
   public static final String FOOTSTEP_PLANNER_MODULE_NAME = "toolbox/footstep_plan";
   public static final String CONTINUOUS_PLANNING_TOOLBOX_MODULE_NAME = "toolbox/continuous_planning";
   public static final String FOOTSTEP_POSTPROCESSING_TOOLBOX_MODULE_NAME = "toolbox/footstep_postprocessing";

   public static final String KINEMATICS_TOOLBOX_MODULE_NAME = "toolbox/ik";
   public static final String KINEMATICS_PLANNING_TOOLBOX_MODULE_NAME = "toolbox/ik_planning";
   public static final String KINEMATICS_STREAMING_TOOLBOX_MODULE_NAME = "toolbox/ik_streaming";
   public static final String STEP_CONSTRAINT_TOOLBOX_MODULE_NAME = "/toolbox/step_constraint";

   public static final String WHOLE_BODY_TRAJECTORY_TOOLBOX_MODULE_NAME = "toolbox/ik_trajectory";
   public static final String WALKING_PREVIEW_TOOLBOX_MODULE_NAME = "toolbox/walking_controller_preview";
   public static final String EXTERNAL_FORCE_ESTIMATION_TOOLBOX_MODULE_NAME = "toolbox/external_force_estimation";
   public static final String STEP_TELEOP_TOOLBOX_MODULE_NAME = "toolbox/teleop/step_teleop";
   public static final String DIRECTIONAL_CONTROL_TOOLBOX_MODULE_NAME = "/toolbox/directional_control";
   public static final String QUADRUPED_SUPPORT_REGION_PUBLISHER_MODULE_NAME = "quadruped_support_region_publisher";

   public static final String BEHAVIOR_MODULE_NAME = "behavior";

   public static final String INPUT = ROS2Topic.INPUT;
   public static final String OUTPUT = ROS2Topic.OUTPUT;

   public static final ROS2Topic<?> IHMC_ROOT = new ROS2Topic<>().withPrefix(IHMC_TOPIC_PREFIX);
   public static final ROS2Topic<?> HUMANOID_CONTROLLER = IHMC_ROOT.withModule(HUMANOID_CONTROL_MODULE_NAME);
   public static final ROS2Topic<?> QUADRUPED_CONTROLLER = IHMC_ROOT.withModule(QUADRUPED_CONTROL_MODULE_NAME);
   public static final ROS2Topic<?> FOOTSTEP_PLANNER = IHMC_ROOT.withModule(FOOTSTEP_PLANNER_MODULE_NAME);
   public static final ROS2Topic<?> CONTINUOUS_PLANNING_TOOLBOX = IHMC_ROOT.withModule(CONTINUOUS_PLANNING_TOOLBOX_MODULE_NAME);
   public static final ROS2Topic<?> FOOTSTEP_POSTPROCESSING_TOOLBOX = IHMC_ROOT.withModule(FOOTSTEP_POSTPROCESSING_TOOLBOX_MODULE_NAME);

   public static final ROS2Topic<?> KINEMATICS_TOOLBOX = IHMC_ROOT.withModule(KINEMATICS_TOOLBOX_MODULE_NAME);
   public static final ROS2Topic<?> KINEMATICS_PLANNING_TOOLBOX = IHMC_ROOT.withModule(KINEMATICS_PLANNING_TOOLBOX_MODULE_NAME);
   public static final ROS2Topic<?> KINEMATICS_STREAMING_TOOLBOX = IHMC_ROOT.withModule(KINEMATICS_STREAMING_TOOLBOX_MODULE_NAME);
   public static final ROS2Topic<?> STEP_CONSTRAINT_TOOLBOX = IHMC_ROOT.withModule(STEP_CONSTRAINT_TOOLBOX_MODULE_NAME);
   public static final ROS2Topic<?> WHOLE_BODY_TRAJECTORY_TOOLBOX = IHMC_ROOT.withModule(WHOLE_BODY_TRAJECTORY_TOOLBOX_MODULE_NAME);
   public static final ROS2Topic<?> WALKING_PREVIEW_TOOLBOX = IHMC_ROOT.withModule(WALKING_PREVIEW_TOOLBOX_MODULE_NAME);
   public static final ROS2Topic<?> EXTERNAL_FORCE_ESTIMATION_TOOLBOX = IHMC_ROOT.withModule(EXTERNAL_FORCE_ESTIMATION_TOOLBOX_MODULE_NAME);
   public static final ROS2Topic<?> STEP_TELEOP_TOOLBOX = IHMC_ROOT.withModule(STEP_TELEOP_TOOLBOX_MODULE_NAME);
   public static final ROS2Topic<?> DIRECTIONAL_CONTROL_TOOLBOX = IHMC_ROOT.withModule(DIRECTIONAL_CONTROL_TOOLBOX_MODULE_NAME);
   public static final ROS2Topic<?> QUADRUPED_SUPPORT_REGION_PUBLISHER = IHMC_ROOT.withModule(QUADRUPED_SUPPORT_REGION_PUBLISHER_MODULE_NAME);

   public static final ROS2Topic<?> BEHAVIOR_MODULE = IHMC_ROOT.withModule(BEHAVIOR_MODULE_NAME);

   public static final ROS2Topic<Empty> KINEMATICS_SIMULATION_HEARTBEAT
         = IHMC_ROOT.withModule("kinematics_simulation").withOutput().withSuffix("heartbeat").withType(Empty.class);

   public static final ROS2Topic<TextToSpeechPacket> TEXT_STATUS = IHMC_ROOT.withTypeName(TextToSpeechPacket.class);

   public static final ROS2Topic<?> BEHAVIOR_MODULE_INPUT = ROS2Tools.BEHAVIOR_MODULE.withInput();
   public static final ROS2Topic<?> BEHAVIOR_MODULE_OUTPUT = ROS2Tools.BEHAVIOR_MODULE.withOutput();
   private static final ROS2Topic<BehaviorControlModePacket> BEHAVIOR_CONTROL_MODE = BEHAVIOR_MODULE_INPUT.withTypeName(BehaviorControlModePacket.class);
   private static final ROS2Topic<HumanoidBehaviorTypePacket> BEHAVIOR_TYPE = BEHAVIOR_MODULE_INPUT.withTypeName(HumanoidBehaviorTypePacket.class);
   private static final ROS2Topic<BehaviorStatusPacket> BEHAVIOR_STATUS = BEHAVIOR_MODULE_OUTPUT.withTypeName(BehaviorStatusPacket.class);
   private static final ROS2Topic<HandDesiredConfigurationMessage> HAND_CONFIGURATION = HUMANOID_CONTROLLER.withInput()
                                                                                                           .withTypeName(HandDesiredConfigurationMessage.class);
   private static final ROS2Topic<SakeHandDesiredCommandMessage> HAND_SAKE_DESIRED_COMMAND = HUMANOID_CONTROLLER.withInput()
                                                                                                                .withTypeName(SakeHandDesiredCommandMessage.class);
   private static final ROS2Topic<SakeHandStatusMessage> HAND_SAKE_DESIRED_STATUS = HUMANOID_CONTROLLER.withOutput()
                                                                                                       .withTypeName(SakeHandStatusMessage.class);
   private static final ROS2Topic<HandJointAnglePacket> HAND_JOINT_ANGLES = HUMANOID_CONTROLLER.withOutput().withTypeName(HandJointAnglePacket.class);

   public static final ROS2Topic<Float64> BOX_MASS = IHMC_ROOT.withSuffix("box_mass").withType(Float64.class);

   public static final ROS2Topic<SystemAvailableMessage> SYSTEM_AVAILABLE = IHMC_ROOT.withModule("mission_control").withType(SystemAvailableMessage.class);

   public final static ExceptionHandler RUNTIME_EXCEPTION = e ->
   {
      throw new RuntimeException(e);
   };

   public static ROS2Topic<HandDesiredConfigurationMessage> getHandConfigurationTopic(String robotName)
   {
      return HAND_CONFIGURATION.withRobot(robotName);
   }

   public static ROS2Topic<SakeHandDesiredCommandMessage> getHandSakeCommandTopic(String robotName, RobotSide side)
   {
      return HAND_SAKE_DESIRED_COMMAND.withRobot(robotName).withSuffix(side.getLowerCaseName());
   }

   public static ROS2Topic<SakeHandStatusMessage> getHandSakeStatusTopic(String robotName, RobotSide side)
   {
      return HAND_SAKE_DESIRED_STATUS.withRobot(robotName).withSuffix(side.getLowerCaseName());
   }

   public static ROS2Topic<HandJointAnglePacket> getHandJointAnglesTopic(String robotName)
   {
      return HAND_JOINT_ANGLES.withRobot(robotName);
   }

   public static ROS2Topic<BehaviorControlModePacket> getBehaviorControlModeTopic(String robotName)
   {
      return BEHAVIOR_CONTROL_MODE.withRobot(robotName);
   }

   public static ROS2Topic<HumanoidBehaviorTypePacket> getBehaviorTypeTopic(String robotName)
   {
      return BEHAVIOR_TYPE.withRobot(robotName);
   }

   public static ROS2Topic<BehaviorStatusPacket> getBehaviorStatusTopic(String robotName)
   {
      return BEHAVIOR_STATUS.withRobot(robotName);
   }

   public static ROS2Topic<?> getControllerOutputTopic(String robotName)
   {
      return HUMANOID_CONTROLLER.withRobot(robotName).withOutput();
   }

   public static ROS2Topic<?> getControllerInputTopic(String robotName)
   {
      return HUMANOID_CONTROLLER.withRobot(robotName).withInput();
   }

   public static ROS2Topic<WalkingControllerPreviewInputMessage> getControllerPreviewInputTopic(String robotName)
   {
      return WALKING_PREVIEW_TOOLBOX.withRobot(robotName).withInput().withTypeName(WalkingControllerPreviewInputMessage.class);
   }

   public static ROS2Topic<WalkingControllerPreviewOutputMessage> getControllerPreviewOutputTopic(String robotName)
   {
      return WALKING_PREVIEW_TOOLBOX.withRobot(robotName).withOutput().withTypeName(WalkingControllerPreviewOutputMessage.class);
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
      return new ROS2Topic<>().withTypeName(messageType);
   }

   public static <T> ROS2Topic<T> typeNamedTopic(Class<T> messageType, ROS2Topic<?> topicName)
   {
      return typeNamedTopic(messageType).withTopic(topicName);
   }

   public static ROS2Topic<RobotConfigurationData> getRobotConfigurationDataTopic(String robotName)
   {
      return typeNamedTopic(RobotConfigurationData.class, getControllerOutputTopic(robotName));
   }

   public static ROS2Topic<HandJointAnglePacket> getHandJointAnglePacketTopic(String robotName)
   {
      return typeNamedTopic(HandJointAnglePacket.class, getControllerOutputTopic(robotName));
   }

   public static ROS2Topic<StampedPosePacket> getPoseCorrectionTopic(String robotName)
   {
      return getControllerInputTopic(robotName).withTypeName(StampedPosePacket.class);
   }

   public static ROS2Topic<DoorParameterPacket> getDoorParameterTopic()
   {
      return typeNamedTopic(DoorParameterPacket.class, ROS2Tools.IHMC_ROOT);
   }

   /**
    * Get system resource usage topic for Mission Control
    * @param instanceId of the Mission Control Daemon
    * @return the ROS2Topic the daemon will use for system resource usage messages
    */
   public static ROS2Topic<SystemResourceUsageMessage> getSystemResourceUsageTopic(UUID instanceId)
   {
      String topicId = instanceId.toString().replace("-", ""); // ROS2 topic names cannot have dashes
      return typeNamedTopic(SystemResourceUsageMessage.class, IHMC_ROOT.withModule("mission_control").withSuffix(topicId));
   }

   /**
    * Get system service status topic for Mission Control
    * @param instanceId of the Mission Control Daemon
    * @return the ROS2Topic the daemon will use for system service status messages
    */
   public static ROS2Topic<SystemServiceStatusMessage> getSystemServiceStatusTopic(UUID instanceId)
   {
      String topicId = instanceId.toString().replace("-", ""); // ROS2 topic names cannot have dashes
      return typeNamedTopic(SystemServiceStatusMessage.class, IHMC_ROOT.withModule("mission_control").withSuffix(topicId));
   }

   public static ROS2Topic<SystemServiceActionMessage> getSystemServiceActionTopic(UUID instanceId)
   {
      String topicId = instanceId.toString().replace("-", ""); // ROS2 topic names cannot have dashes
      return typeNamedTopic(SystemServiceActionMessage.class, IHMC_ROOT.withModule("mission_control").withSuffix(topicId));
   }

   public static ROS2Topic<Empty> getSystemRebootTopic(UUID instanceId)
   {
      String topicId = instanceId.toString().replace("-", ""); // ROS2 topic names cannot have dashes
      return typeNamedTopic(Empty.class, IHMC_ROOT.withModule("mission_control").withSuffix(topicId));
   }

   public static ROS2Topic<SystemServiceLogRefreshMessage> getSystemServiceLogRefreshTopic(UUID instanceId)
   {
      String topicId = instanceId.toString().replace("-", ""); // ROS2 topic names cannot have dashes
      return typeNamedTopic(SystemServiceLogRefreshMessage.class, IHMC_ROOT.withModule("mission_control").withSuffix(topicId));
   }

   /**
    * Get the system service status QOS profile for Mission Control
    * @return the ROS2QosProfile with history
    */
   public static ROS2QosProfile getSystemServiceStatusQosProfile()
   {
      ROS2QosProfile profile = new ROS2QosProfile();
      profile.setReliability(ReliabilityQosKindType.RELIABLE);
      return profile;
   }

   public final static String NAMESPACE = "/us/ihmc"; // ? no idea what this does

   private static final RTPSCommunicationFactory FACTORY = new RTPSCommunicationFactory();
   private static final int DOMAIN_ID = FACTORY.getDomainId();
   private static final InetAddress ADDRESS_RESTRICTION = FACTORY.getAddressRestriction();

   /**
    * Creates a ROS2 node that shares the same implementation as a real-time node <b>but that should
    * not be run in a real-time environment</b>.
    *
    * @param pubSubImplementation the implementation to use.
    * @param nodeName             the name of the new ROS node.
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
    * @param nodeName             the name of the new ROS node.
    * @param exceptionHandler     how to handle exceptions thrown during the instantiation.
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
    * @param pubSubImplementation           the implementation to use.
    * @param periodicThreadSchedulerFactory the factory used to create a periodic thread.
    * @param nodeName                       the name of the new ROS node.
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
    * @param pubSubImplementation           the implementation to use.
    * @param periodicThreadSchedulerFactory the factory used to create a periodic thread.
    * @param nodeName                       the name of the new ROS node.
    * @param exceptionHandler               how to handle exceptions thrown during the instantiation.
    * @return the ROS node.
    */
   public static RealtimeROS2Node createRealtimeROS2Node(PubSubImplementation pubSubImplementation,
                                                         PeriodicThreadSchedulerFactory periodicThreadSchedulerFactory,
                                                         String nodeName,
                                                         ExceptionHandler exceptionHandler)
   {
      Domain domain = DomainFactory.getDomain(pubSubImplementation);
      try
      {
         return new RealtimeROS2Node(domain, periodicThreadSchedulerFactory, nodeName, NAMESPACE, DOMAIN_ID, ADDRESS_RESTRICTION);
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
      Domain domain = DomainFactory.getDomain(pubSubImplementation);
      try
      {
         return new ROS2Node(domain, nodeName, NAMESPACE, DOMAIN_ID, ADDRESS_RESTRICTION);
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
      return createCallbackSubscriptionTypeNamed(ros2Node, messageType, topicName, newMessageListener, ROS2QosProfile.DEFAULT());
   }

   public static <T> ROS2Subscription<T> createCallbackSubscriptionTypeNamed(ROS2NodeInterface ros2Node,
                                                                             Class<T> messageType,
                                                                             ROS2Topic<?> topicName,
                                                                             NewMessageListener<T> newMessageListener,
                                                                             ROS2QosProfile qosProfile)
   {
      return createCallbackSubscription(ros2Node, typeNamedTopic(messageType).withTopic(topicName), newMessageListener, qosProfile);
   }

   public static <T> ROS2Subscription<T> createCallbackSubscription(ROS2NodeInterface ros2Node, ROS2Topic<T> topic, NewMessageListener<T> newMessageListener)
   {
      return createCallbackSubscription(ros2Node, topic, newMessageListener, ROS2QosProfile.DEFAULT());
   }

   public static <T> ROS2Subscription<T> createCallbackSubscription(ROS2NodeInterface ros2Node,
                                                                    ROS2Topic<T> topic,
                                                                    NewMessageListener<T> newMessageListener,
                                                                    ROS2QosProfile qosProfile)
   {
      return createCallbackSubscription(ros2Node, topic.getType(), topic.getName(), newMessageListener, qosProfile);
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
      return createCallbackSubscription(ros2Node, messageType, topicName, newMessageListener, ROS2QosProfile.DEFAULT());
   }

   public static <T> ROS2Subscription<T> createCallbackSubscription(ROS2NodeInterface ros2Node,
                                                                    Class<T> messageType,
                                                                    String topicName,
                                                                    NewMessageListener<T> newMessageListener,
                                                                    ROS2QosProfile qosProfile)
   {
      return createCallbackSubscription(ros2Node, messageType, topicName, newMessageListener, qosProfile, RUNTIME_EXCEPTION);
   }

   public static <T> ROS2Subscription<T> createCallbackSubscription(ROS2NodeInterface ros2Node,
                                                                    Class<T> messageType,
                                                                    String topicName,
                                                                    NewMessageListener<T> newMessageListener,
                                                                    ExceptionHandler exceptionHandler)
   {
      return createCallbackSubscription(ros2Node, messageType, topicName, newMessageListener, ROS2QosProfile.DEFAULT(), exceptionHandler);
   }

   public static <T> ROS2Subscription<T> createCallbackSubscription(ROS2NodeInterface ros2Node,
                                                                    Class<T> messageType,
                                                                    String topicName,
                                                                    NewMessageListener<T> newMessageListener,
                                                                    ROS2QosProfile qosProfile,
                                                                    ExceptionHandler exceptionHandler)
   {
      try
      {
         TopicDataType<T> topicDataType = ROS2TopicNameTools.newMessageTopicDataTypeInstance(messageType);
         return ros2Node.createSubscription(topicDataType, newMessageListener, topicName, qosProfile);
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
                                                     ROS2Topic<T> topic,
                                                     ROS2QosProfile qosProfile,
                                                     NewMessageListener<T> newMessageListener)
   {
      createCallbackSubscription(realtimeROS2Node, topic.getType(), topic.getName(), newMessageListener, qosProfile, RUNTIME_EXCEPTION);
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
         realtimeROS2Node.createSubscription(topicDataType, newMessageListener, topicName, qosProfile);
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
      }
   }

   public static <T> IHMCROS2Callback createCallbackSubscription2(ROS2NodeInterface ros2Node, ROS2Topic<T> topic, Consumer<T> callback)
   {
      return new IHMCROS2Callback<>(ros2Node, topic, callback);
   }

   public static <T> IHMCROS2Callback createCallbackSubscription2(ROS2NodeInterface ros2Node, ROS2Topic<Empty> topic, Runnable callback)
   {
      return new IHMCROS2Callback<>(ros2Node, topic, message -> callback.run());
   }

   /**
    * Allocation free callback where the user only has access to the message in the callback.
    * The user should not take up any significant time in the callback to not slow down the ROS 2
    * thread.
    */
   public static <T> void createVolatileCallbackSubscription(ROS2NodeInterface ros2Node, ROS2Topic<T> topic, Consumer<T> callback)
   {
      try
      {
         TopicDataType<T> topicDataType = IHMCROS2Callback.newMessageTopicDataTypeInstance(topic.getType());
         T data = topicDataType.createData();
         ros2Node.createSubscription(topicDataType, subscriber ->
         {
            if (subscriber.takeNextData(data, null))
            {
               callback.accept(data);
            }
         }, topic.getName(), ROS2QosProfile.DEFAULT());
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   /** Use when you only need the latest message and need allocation free. */
   public static <T> SwapReference<T> createSwapReferenceSubscription(ROS2NodeInterface ros2Node, ROS2Topic<T> topic, Notification callback)
   {
      try
      {
         TopicDataType<T> topicDataType = IHMCROS2Callback.newMessageTopicDataTypeInstance(topic.getType());
         SwapReference<T> swapReference = new SwapReference<>(topicDataType::createData);
         ros2Node.createSubscription(topicDataType, subscriber ->
         {
            if (subscriber.takeNextData(swapReference.getForThreadOne(), null))
            {
               swapReference.swap();
               callback.set();
            }
         }, topic.getName(), ROS2QosProfile.DEFAULT());
         return swapReference;
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
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

   public static <T> IHMCRealtimeROS2Publisher<T> createPublisherTypeNamed(RealtimeROS2Node realtimeROS2Node, Class<T> messageType, ROS2Topic<?> topicName)
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

   public static <T> IHMCRealtimeROS2Publisher<T> createPublisher(RealtimeROS2Node realtimeROS2Node, ROS2Topic<T> topic, ROS2QosProfile qosProfile)
   {
      return createPublisher(realtimeROS2Node, topic.getType(), topic.getName(), qosProfile, RUNTIME_EXCEPTION);
   }

   public static <T> IHMCRealtimeROS2Publisher<T> createPublisher(RealtimeROS2Node realtimeROS2Node,
                                                                  Class<T> messageType,
                                                                  String topicName,
                                                                  ExceptionHandler exceptionHandler)
   {
      return createPublisher(realtimeROS2Node, messageType, topicName, ROS2QosProfile.DEFAULT(), exceptionHandler);
   }

   public static <T> IHMCRealtimeROS2Publisher<T> createPublisher(RealtimeROS2Node realtimeROS2Node,
                                                                  Class<T> messageType,
                                                                  String topicName,
                                                                  ROS2QosProfile qosProfile,
                                                                  ExceptionHandler exceptionHandler)
   {
      try
      {
         TopicDataType<T> topicDataType = ROS2TopicNameTools.newMessageTopicDataTypeInstance(messageType);
         return new IHMCRealtimeROS2Publisher<T>(realtimeROS2Node.createPublisher(topicDataType, topicName, qosProfile, 10));
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
