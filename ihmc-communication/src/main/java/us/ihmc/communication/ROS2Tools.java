package us.ihmc.communication;

import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import controller_msgs.msg.dds.SakeHandDesiredCommandMessage;
import controller_msgs.msg.dds.SakeHandStatusMessage;
import ihmc_common_msgs.msg.dds.StampedPosePacket;
import ihmc_common_msgs.msg.dds.TextToSpeechPacket;
import mission_control_msgs.msg.dds.*;
import perception_msgs.msg.dds.DoorParameterPacket;
import std_msgs.msg.dds.Empty;
import std_msgs.msg.dds.Float64;
import toolbox_msgs.msg.dds.*;
import us.ihmc.commons.thread.Notification;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.*;
import us.ihmc.tools.thread.SwapReference;
import us.ihmc.util.PeriodicRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicThreadSchedulerFactory;

import java.util.function.Consumer;

public class ROS2Tools
{
   public static final String IHMC_TOPIC_PREFIX = "ihmc";

   public static final String FOOTSTEP_PLANNER_NODE_NAME = "ihmc_multi_stage_footstep_planning_module";

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

   public static final ROS2Topic<?> IHMC_ROOT = new ROS2Topic<>().withPrefix(IHMC_TOPIC_PREFIX);
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

   public static final ROS2Topic<Empty> KINEMATICS_SIMULATION_HEARTBEAT
         = IHMC_ROOT.withModule("kinematics_simulation").withOutput().withSuffix("heartbeat").withType(Empty.class);

   public static final ROS2Topic<TextToSpeechPacket> TEXT_STATUS = IHMC_ROOT.withTypeName(TextToSpeechPacket.class);

   private static final ROS2Topic<HandDesiredConfigurationMessage> HAND_CONFIGURATION = HumanoidControllerAPI.HUMANOID_CONTROLLER.withInput()
                                                                                                                                 .withTypeName(HandDesiredConfigurationMessage.class);
   private static final ROS2Topic<SakeHandDesiredCommandMessage> HAND_SAKE_DESIRED_COMMAND = HumanoidControllerAPI.HUMANOID_CONTROLLER.withInput()
                                                                                                                                      .withTypeName(SakeHandDesiredCommandMessage.class);
   private static final ROS2Topic<SakeHandStatusMessage> HAND_SAKE_DESIRED_STATUS = HumanoidControllerAPI.HUMANOID_CONTROLLER.withOutput()
                                                                                                                             .withTypeName(SakeHandStatusMessage.class);

   public static final ROS2Topic<Float64> BOX_MASS = IHMC_ROOT.withSuffix("box_mass").withType(Float64.class);

   public static final ROS2Topic<SystemAvailableMessage> SYSTEM_AVAILABLE = IHMC_ROOT.withModule("mission_control").withType(SystemAvailableMessage.class);

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

   public static ROS2Topic<WalkingControllerPreviewInputMessage> getControllerPreviewInputTopic(String robotName)
   {
      return WALKING_PREVIEW_TOOLBOX.withRobot(robotName).withInput().withTypeName(WalkingControllerPreviewInputMessage.class);
   }

   public static ROS2Topic<WalkingControllerPreviewOutputMessage> getControllerPreviewOutputTopic(String robotName)
   {
      return WALKING_PREVIEW_TOOLBOX.withRobot(robotName).withOutput().withTypeName(WalkingControllerPreviewOutputMessage.class);
   }

   /** @deprecated Use {@link ROS2Topic#withTypeName} instead. */
   public static <T> ROS2Topic<T> typeNamedTopic(Class<T> messageType)
   {
      return new ROS2Topic<>().withTypeName(messageType);
   }

   public static ROS2Topic<StampedPosePacket> getPoseCorrectionTopic(String robotName)
   {
      return HumanoidControllerAPI.getInputTopic(robotName).withTypeName(StampedPosePacket.class);
   }

   public static ROS2Topic<DoorParameterPacket> getDoorParameterTopic()
   {
      return ROS2Tools.IHMC_ROOT.withTypeName(DoorParameterPacket.class);
   }

   private static final RTPSCommunicationFactory FACTORY = new RTPSCommunicationFactory();

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
      return new RealtimeROS2Node(pubSubImplementation, nodeName, FACTORY.getDomainId(), FACTORY.getAddressRestriction());
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
      return new RealtimeROS2Node(pubSubImplementation, periodicThreadSchedulerFactory, nodeName, FACTORY.getDomainId(), FACTORY.getAddressRestriction());
   }

   public static ROS2Node createROS2Node(PubSubImplementation pubSubImplementation, String nodeName)
   {
      return new ROS2Node(pubSubImplementation, nodeName, FACTORY.getDomainId(), FACTORY.getAddressRestriction());
   }

   /** @deprecated Use {@link ROS2Topic#withTypeName} or look at other examples how to retrieve the topic in a safer way. */
   public static <T> ROS2Subscription<T> createCallbackSubscriptionTypeNamed(ROS2NodeInterface ros2Node,
                                                                             Class<T> messageType,
                                                                             ROS2Topic<?> topicName,
                                                                             NewMessageListener<T> newMessageListener)
   {
      return createCallbackSubscription(ros2Node, typeNamedTopic(messageType).withTopic(topicName), newMessageListener);
   }

   public static <T> ROS2Subscription<T> createCallbackSubscription(ROS2NodeInterface ros2Node,
                                                                    ROS2Topic<T> topic,
                                                                    NewMessageListener<T> newMessageListener)
   {
      return ros2Node.createSubscription(topic, newMessageListener, newMessageListener::onSubscriptionMatched);
   }

   /**
    * Allocation free callback where the user only has access to the message in the callback.
    * The user should not take up any significant time in the callback to not slow down the ROS 2
    * thread.
    */
   public static <T> void createVolatileCallbackSubscription(ROS2NodeInterface ros2Node, ROS2Topic<T> topic, Consumer<T> callback)
   {
      TopicDataType<T> topicDataType = ROS2TopicNameTools.newMessageTopicDataTypeInstance(topic.getType());
      T data = topicDataType.createData();
      ros2Node.createSubscription(topicDataType, subscriber ->
      {
         if (subscriber.takeNextData(data, null))
         {
            callback.accept(data);
         }
      }, topic.getName(), topic.getQoS());
   }

   /** Use when you only need the latest message and need allocation free. */
   public static <T> SwapReference<T> createSwapReferenceSubscription(ROS2NodeInterface ros2Node, ROS2Topic<T> topic, Notification callback)
   {
      TopicDataType<T> topicDataType = ROS2TopicNameTools.newMessageTopicDataTypeInstance(topic.getType());
      SwapReference<T> swapReference = new SwapReference<>(topicDataType::createData);
      ros2Node.createSubscription(topicDataType, subscriber ->
      {
         if (subscriber.takeNextData(swapReference.getForThreadOne(), null))
         {
            swapReference.swap();
            callback.set();
         }
      }, topic.getName(), topic.getQoS());
      return swapReference;
   }
}
