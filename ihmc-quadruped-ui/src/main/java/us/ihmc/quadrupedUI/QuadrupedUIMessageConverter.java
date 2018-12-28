package us.ihmc.quadrupedUI;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.quadrupedCommunication.QuadrupedControllerAPIDefinition;
import us.ihmc.quadrupedCommunication.QuadrupedMessageTools;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.ros2.RealtimeRos2Node;

import us.ihmc.messager.Messager;

import static us.ihmc.communication.ROS2Tools.getTopicNameGenerator;

public class QuadrupedUIMessageConverter
{
   private final RealtimeRos2Node ros2Node;

   private final Messager messager;
   private final String robotName;

   private IHMCRealtimeROS2Publisher<HighLevelStateMessage> desiredHighLevelStatePublisher;
   private IHMCRealtimeROS2Publisher<QuadrupedBodyHeightMessage> bodyHeightPublisher;
   private IHMCRealtimeROS2Publisher<ToolboxStateMessage> enableStepTeleopPublisher;
   private IHMCRealtimeROS2Publisher<ToolboxStateMessage> enableBodyTeleopPublisher;
   private IHMCRealtimeROS2Publisher<ToolboxStateMessage> enableHeightTeleopPublisher;
   private IHMCRealtimeROS2Publisher<ToolboxStateMessage> enableJoystickPublisher;
   private IHMCRealtimeROS2Publisher<QuadrupedXGaitSettingsPacket> stepTeleopXGaitSettingsPublisher;
   private IHMCRealtimeROS2Publisher<QuadrupedXGaitSettingsPacket> xboxXGaitSettingsPublisher;

   public QuadrupedUIMessageConverter(RealtimeRos2Node ros2Node, Messager messager, String robotName)
   {
      this.messager = messager;
      this.robotName = robotName;
      this.ros2Node = ros2Node;

      registerPubSubs();

      ros2Node.spin();
   }

   public void destroy()
   {
      ros2Node.destroy();
   }

   private void registerPubSubs()
   {
      MessageTopicNameGenerator controllerPubGenerator = QuadrupedControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      ROS2Tools
            .createCallbackSubscription(ros2Node, RobotConfigurationData.class, controllerPubGenerator, s -> processRobotConfigurationData(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(ros2Node, HighLevelStateChangeStatusMessage.class, controllerPubGenerator,
                                           s -> processControllerStateChangeMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(ros2Node, QuadrupedFootstepStatusMessage.class, controllerPubGenerator,
                                           s -> messager.submitMessage(QuadrupedUIMessagerAPI.FootstepStatusMessageTopic, s.takeNextData()));

      MessageTopicNameGenerator controllerSubGenerator = QuadrupedControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);
      desiredHighLevelStatePublisher = ROS2Tools.createPublisher(ros2Node, HighLevelStateMessage.class, controllerSubGenerator);
      bodyHeightPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedBodyHeightMessage.class, controllerSubGenerator);

      enableBodyTeleopPublisher = ROS2Tools.createPublisher(ros2Node, ToolboxStateMessage.class, getTopicNameGenerator(robotName, ROS2Tools.BODY_TELEOP_TOOLBOX,
                                                                                                                       ROS2Tools.ROS2TopicQualifier.INPUT));
      enableStepTeleopPublisher = ROS2Tools.createPublisher(ros2Node, ToolboxStateMessage.class, getTopicNameGenerator(robotName, ROS2Tools.STEP_TELEOP_TOOLBOX,
                                                                                                                       ROS2Tools.ROS2TopicQualifier.INPUT));
      enableHeightTeleopPublisher = ROS2Tools.createPublisher(ros2Node, ToolboxStateMessage.class,
                                                              getTopicNameGenerator(robotName, ROS2Tools.HEIGHT_TELEOP_TOOLBOX,
                                                                                    ROS2Tools.ROS2TopicQualifier.INPUT));
      enableJoystickPublisher = ROS2Tools.createPublisher(ros2Node, ToolboxStateMessage.class,
                                                          getTopicNameGenerator(robotName, ROS2Tools.XBOX_TELEOP_TOOLBOX, ROS2Tools.ROS2TopicQualifier.INPUT));
      stepTeleopXGaitSettingsPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedXGaitSettingsPacket.class, getTopicNameGenerator(robotName, ROS2Tools.STEP_TELEOP_TOOLBOX,
                                                                                                                                       ROS2Tools.ROS2TopicQualifier.INPUT));
      xboxXGaitSettingsPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedXGaitSettingsPacket.class, getTopicNameGenerator(robotName, ROS2Tools.XBOX_TELEOP_TOOLBOX,
                                                                                                                                       ROS2Tools.ROS2TopicQualifier.INPUT));

      messager.registerTopicListener(QuadrupedUIMessagerAPI.DesiredControllerNameTopic, this::publishDesiredHighLevelControllerState);
      messager.registerTopicListener(QuadrupedUIMessagerAPI.DesiredBodyHeightTopic, this::publishDesiredBodyHeight);

      messager.registerTopicListener(QuadrupedUIMessagerAPI.EnableStepTeleopTopic, this::publishEnableStepTeleop);
      messager.registerTopicListener(QuadrupedUIMessagerAPI.EnableBodyTeleopTopic, this::publishEnableBodyTeleop);
      messager.registerTopicListener(QuadrupedUIMessagerAPI.EnableHeightTeleopTopic, this::publishEnableHeightTeleop);
      messager.registerTopicListener(QuadrupedUIMessagerAPI.EnableJoystickTopic, this::publishEnableJoystick);
      messager.registerTopicListener(QuadrupedUIMessagerAPI.XGaitSettingsTopic, this::publishQuadrupedXGaitSettings);
   }

   private void processRobotConfigurationData(RobotConfigurationData robotConfigurationData)
   {
      messager.submitMessage(QuadrupedUIMessagerAPI.RobotConfigurationDataTopic, robotConfigurationData);
   }

   private void processControllerStateChangeMessage(HighLevelStateChangeStatusMessage stateChangeStatusMessage)
   {
      HighLevelControllerName currentState = HighLevelControllerName.fromByte(stateChangeStatusMessage.getEndHighLevelControllerName());
      messager.submitMessage(QuadrupedUIMessagerAPI.CurrentControllerNameTopic, currentState);
   }

   private void publishDesiredHighLevelControllerState(HighLevelControllerName controllerName)
   {
      desiredHighLevelStatePublisher.publish(HumanoidMessageTools.createHighLevelStateMessage(controllerName));
   }

   public void publishDesiredBodyHeight(double desiredBodyHeight)
   {
      QuadrupedBodyHeightMessage bodyHeightMessage = QuadrupedMessageTools.createQuadrupedBodyHeightMessage(0.0, desiredBodyHeight);
      bodyHeightMessage.setControlBodyHeight(true);
      bodyHeightMessage.setIsExpressedInAbsoluteTime(false);
      bodyHeightPublisher.publish(bodyHeightMessage);
   }

   public void publishEnableStepTeleop(boolean enable)
   {
      if (enable)
         enableStepTeleopPublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));
      else
         enableStepTeleopPublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.SLEEP));
   }

   public void publishEnableBodyTeleop(boolean enable)
   {
      if (enable)
         enableBodyTeleopPublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));
      else
         enableBodyTeleopPublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.SLEEP));
   }

   public void publishEnableHeightTeleop(boolean enable)
   {
      if (enable)
         enableHeightTeleopPublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));
      else
         enableHeightTeleopPublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.SLEEP));
   }

   public void publishEnableJoystick(boolean enable)
   {
      if (enable)
         enableJoystickPublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));
      else
         enableJoystickPublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.SLEEP));
   }

   public void publishQuadrupedXGaitSettings(QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      QuadrupedXGaitSettingsPacket packet = new QuadrupedXGaitSettingsPacket();

      packet.setStanceLength(xGaitSettings.getStanceLength());
      packet.setStanceWidth(xGaitSettings.getStanceWidth());
      packet.setEndPhaseShift(xGaitSettings.getEndPhaseShift());
      packet.setEndDoubleSupportDuration(xGaitSettings.getEndDoubleSupportDuration());
      packet.setStepDuration(xGaitSettings.getStepDuration());
      packet.setStepGroundClearance(xGaitSettings.getStepGroundClearance());

      stepTeleopXGaitSettingsPublisher.publish(packet);
      xboxXGaitSettingsPublisher.publish(packet);
   }

   public static QuadrupedUIMessageConverter createConverter(Messager messager, String robotName, DomainFactory.PubSubImplementation implementation)
   {
      RealtimeRos2Node ros2Node = ROS2Tools.createRealtimeRos2Node(implementation, "ihmc_quadruped_ui");
      return createConverter(ros2Node, messager, robotName);
   }

   public static QuadrupedUIMessageConverter createConverter(RealtimeRos2Node ros2Node, Messager messager, String robotName)
   {
      return new QuadrupedUIMessageConverter(ros2Node, messager, robotName);
   }
}
