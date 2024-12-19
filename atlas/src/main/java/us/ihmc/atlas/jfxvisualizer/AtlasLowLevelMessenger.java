package us.ihmc.atlas.jfxvisualizer;

import atlas_msgs.msg.dds.AtlasLowLevelControlModeMessage;
import atlas_msgs.msg.dds.BDIBehaviorCommandPacket;
import controller_msgs.msg.dds.AbortWalkingMessage;
import controller_msgs.msg.dds.PauseWalkingMessage;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.atlas.AtlasLowLevelControlMode;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;

public class AtlasLowLevelMessenger implements RobotLowLevelMessenger
{
   private final ROS2Publisher<AtlasLowLevelControlModeMessage> lowLevelModePublisher;
   private final ROS2Publisher<BDIBehaviorCommandPacket> bdiBehaviorPublisher;
   private final ROS2Publisher<AbortWalkingMessage> abortWalkingPublisher;
   private final ROS2Publisher<PauseWalkingMessage> pauseWalkingPublisher;

   public AtlasLowLevelMessenger(RealtimeROS2Node ros2Node, String robotName)
   {
      ROS2Topic inputTopic = HumanoidControllerAPI.getInputTopic(robotName);
      lowLevelModePublisher = ros2Node.createPublisher(inputTopic.withTypeName(AtlasLowLevelControlModeMessage.class));
      bdiBehaviorPublisher = ros2Node.createPublisher(inputTopic.withTypeName(BDIBehaviorCommandPacket.class));
      abortWalkingPublisher = ros2Node.createPublisher(inputTopic.withTypeName(AbortWalkingMessage.class));
      pauseWalkingPublisher = ros2Node.createPublisher(inputTopic.withTypeName(PauseWalkingMessage.class));
   }

   @Override
   public void sendFreezeRequest()
   {
      AtlasLowLevelControlModeMessage message = new AtlasLowLevelControlModeMessage();
      message.setRequestedAtlasLowLevelControlMode(AtlasLowLevelControlMode.FREEZE.toByte());
      lowLevelModePublisher.publish(message);
   }

   @Override
   public void sendStandRequest()
   {
      AtlasLowLevelControlModeMessage message = new AtlasLowLevelControlModeMessage();
      message.setRequestedAtlasLowLevelControlMode(AtlasLowLevelControlMode.STAND_PREP.toByte());
      lowLevelModePublisher.publish(message);
   }
   
   @Override
   public void sendShutdownRequest()
   {
      BDIBehaviorCommandPacket message = HumanoidMessageTools.createBDIBehaviorCommandPacket(true);
      bdiBehaviorPublisher.publish(message);
   }

   @Override
   public void sendAbortWalkingRequest()
   {
      abortWalkingPublisher.publish(new AbortWalkingMessage());
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
