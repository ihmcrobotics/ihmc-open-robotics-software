package us.ihmc.zulu;

import controller_msgs.AbortWalkingMessage;
import controller_msgs.HighLevelStateMessage;
import controller_msgs.PauseWalkingMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;

public class ZuluDirectRobotInterface implements RobotLowLevelMessenger
{
   private final ROS2Publisher<AbortWalkingMessage> abortWalkingPublisher;
   private final ROS2Publisher<PauseWalkingMessage> pauseWalkingPublisher;
   private final ROS2Publisher<HighLevelStateMessage> highLevelStatePublisher;

   public ZuluDirectRobotInterface(ROS2Node ros2Node, DRCRobotModel robotModel)
   {
      this.abortWalkingPublisher = ros2Node.createPublisher(HumanoidControllerAPI.getTopic(AbortWalkingMessage.class, robotModel.getSimpleRobotName()));
      this.pauseWalkingPublisher = ros2Node.createPublisher(HumanoidControllerAPI.getTopic(PauseWalkingMessage.class, robotModel.getSimpleRobotName()));
      highLevelStatePublisher = ros2Node.createPublisher(HumanoidControllerAPI.getTopic(HighLevelStateMessage.class, robotModel.getSimpleRobotName()));
   }

   @Override
   public void sendFreezeRequest()
   {
      highLevelStatePublisher.publish(HumanoidMessageTools.createHighLevelStateMessage(HighLevelControllerName.FREEZE_STATE));
   }

   @Override
   public void sendStandRequest()
   {
      highLevelStatePublisher.publish(HumanoidMessageTools.createHighLevelStateMessage(HighLevelControllerName.STAND_PREP_STATE));
   }

   @Override
   public void sendAbortWalkingRequest()
   {
      abortWalkingPublisher.publish(new AbortWalkingMessage());
   }

   @Override
   public void sendPauseWalkingRequest()
   {
      pauseWalkingPublisher.publish(HumanoidMessageTools.createPauseWalkingMessage(true));
   }

   @Override
   public void sendContinueWalkingRequest()
   {
      pauseWalkingPublisher.publish(HumanoidMessageTools.createPauseWalkingMessage(false));
   }
}
