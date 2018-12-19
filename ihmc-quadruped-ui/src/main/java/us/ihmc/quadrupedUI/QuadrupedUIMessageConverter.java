package us.ihmc.quadrupedUI;

import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import controller_msgs.msg.dds.HighLevelStateMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.quadrupedCommunication.QuadrupedControllerAPIDefinition;
import us.ihmc.ros2.RealtimeRos2Node;

import us.ihmc.messager.Messager;

public class QuadrupedUIMessageConverter
{
   private final RealtimeRos2Node ros2Node;

   private final Messager messager;
   private final String robotName;

   private IHMCRealtimeROS2Publisher<HighLevelStateMessage> desiredHighLevelStatePublisher;

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

      MessageTopicNameGenerator controllerSubGenerator = QuadrupedControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);
      desiredHighLevelStatePublisher = ROS2Tools.createPublisher(ros2Node, HighLevelStateMessage.class, controllerSubGenerator);

      messager.registerTopicListener(QuadrupedUIMessagerAPI.DesiredControllerNameTopic, this::publishDesiredHighLevelControllerState);
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
