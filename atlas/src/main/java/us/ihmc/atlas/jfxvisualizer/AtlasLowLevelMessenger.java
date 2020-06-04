package us.ihmc.atlas.jfxvisualizer;

import controller_msgs.msg.dds.AbortWalkingMessage;
import controller_msgs.msg.dds.AtlasLowLevelControlModeMessage;
import controller_msgs.msg.dds.BDIBehaviorCommandPacket;
import controller_msgs.msg.dds.PauseWalkingMessage;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.atlas.AtlasLowLevelControlMode;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeRos2Node;

public class AtlasLowLevelMessenger implements RobotLowLevelMessenger
{
   private final IHMCRealtimeROS2Publisher<AtlasLowLevelControlModeMessage> lowLevelModePublisher;
   private final IHMCRealtimeROS2Publisher<BDIBehaviorCommandPacket> bdiBehaviorPublisher;
   private final IHMCRealtimeROS2Publisher<AbortWalkingMessage> abortWalkingPublisher;
   private final IHMCRealtimeROS2Publisher<PauseWalkingMessage> pauseWalkingPublisher;

   public AtlasLowLevelMessenger(RealtimeRos2Node ros2Node, String robotName)
   {
      ROS2Topic inputTopic = ROS2Tools.getControllerInputTopic(robotName);
      lowLevelModePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, AtlasLowLevelControlModeMessage.class, inputTopic);
      bdiBehaviorPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, BDIBehaviorCommandPacket.class, inputTopic);
      abortWalkingPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, AbortWalkingMessage.class, inputTopic);
      pauseWalkingPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, PauseWalkingMessage.class, inputTopic);
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
