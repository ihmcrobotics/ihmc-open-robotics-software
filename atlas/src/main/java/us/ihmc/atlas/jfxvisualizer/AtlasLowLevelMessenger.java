package us.ihmc.atlas.jfxvisualizer;

import controller_msgs.msg.dds.AbortWalkingMessage;
import controller_msgs.msg.dds.AtlasLowLevelControlModeMessage;
import controller_msgs.msg.dds.BDIBehaviorCommandPacket;
import controller_msgs.msg.dds.PauseWalkingMessage;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.atlas.AtlasLowLevelControlMode;
import us.ihmc.ros2.RealtimeRos2Node;

public class AtlasLowLevelMessenger implements RobotLowLevelMessenger
{
   private final IHMCRealtimeROS2Publisher<AtlasLowLevelControlModeMessage> lowLevelModePublisher;
   private final IHMCRealtimeROS2Publisher<BDIBehaviorCommandPacket> bdiBehaviorPublisher;
   private final IHMCRealtimeROS2Publisher<AbortWalkingMessage> abortWalkingPublisher;
   private final IHMCRealtimeROS2Publisher<PauseWalkingMessage> pauseWalkingPublisher;

   public AtlasLowLevelMessenger(RealtimeRos2Node ros2Node, String robotName)
   {
      MessageTopicNameGenerator subscriberTopicNameGenerator = ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);
      lowLevelModePublisher = ROS2Tools.createPublisher(ros2Node, AtlasLowLevelControlModeMessage.class, subscriberTopicNameGenerator);
      bdiBehaviorPublisher = ROS2Tools.createPublisher(ros2Node, BDIBehaviorCommandPacket.class, subscriberTopicNameGenerator);
      abortWalkingPublisher = ROS2Tools.createPublisher(ros2Node, AbortWalkingMessage.class, subscriberTopicNameGenerator);
      pauseWalkingPublisher = ROS2Tools.createPublisher(ros2Node, PauseWalkingMessage.class, subscriberTopicNameGenerator);
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
