package us.ihmc.humanoidBehaviors.ui.tools;

import controller_msgs.msg.dds.AbortWalkingMessage;
import controller_msgs.msg.dds.HighLevelStateMessage;
import controller_msgs.msg.dds.PauseWalkingMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.Ros2Node;

public class ValkyrieDirectRobotInterface implements RobotLowLevelMessenger
{
   private final IHMCROS2Publisher<HighLevelStateMessage> highLevelStatePublisher;
   private final IHMCROS2Publisher<AbortWalkingMessage> abortWalkingPublisher;
   private final IHMCROS2Publisher<PauseWalkingMessage> pauseWalkingPublisher;

   public ValkyrieDirectRobotInterface(Ros2Node ros2Node, DRCRobotModel robotModel)
   {
      ROS2Topic inputTopic = ROS2Tools.getControllerInputTopic(robotModel.getSimpleRobotName());
      highLevelStatePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, HighLevelStateMessage.class, inputTopic);
      abortWalkingPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, AbortWalkingMessage.class, inputTopic);
      pauseWalkingPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, PauseWalkingMessage.class, inputTopic);
   }

   @Override
   public void sendAbortWalkingRequest()
   {
      abortWalkingPublisher.publish(new AbortWalkingMessage());
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

   @Override
   public void sendPauseWalkingRequest()
   {
      PauseWalkingMessage message = new PauseWalkingMessage();
      message.setPause(true);
      pauseWalkingPublisher.publish(message);
   }

   @Override
   public void sendContinueWalkingRequest()
   {
      PauseWalkingMessage message = new PauseWalkingMessage();
      message.setPause(false);
      pauseWalkingPublisher.publish(message);
   }
}
