package us.ihmc.humanoidBehaviors.ui.tools;

import controller_msgs.msg.dds.HighLevelStateMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.ros2.Ros2Node;

public class ValkyrieDirectRobotInterface implements RobotLowLevelMessenger
{
   private final IHMCROS2Publisher<HighLevelStateMessage> highLevelStatePublisher;

   public ValkyrieDirectRobotInterface(Ros2Node ros2Node, DRCRobotModel robotModel)
   {
      MessageTopicNameGenerator subscriberTopicNameGenerator = ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotModel.getSimpleRobotName());
      highLevelStatePublisher = ROS2Tools.createPublisher(ros2Node, HighLevelStateMessage.class, subscriberTopicNameGenerator);
   }

   @Override
   public void sendFreezeRequest()
   {
      HighLevelStateMessage message = new HighLevelStateMessage();
      message.setHighLevelControllerName(HighLevelControllerName.EXIT_WALKING.toByte());
      highLevelStatePublisher.publish(message);
   }

   @Override
   public void sendStandRequest()
   {
      HighLevelStateMessage message = new HighLevelStateMessage();
      message.setHighLevelControllerName(HighLevelControllerName.STAND_TRANSITION_STATE.toByte());
      highLevelStatePublisher.publish(message);
   }
}
