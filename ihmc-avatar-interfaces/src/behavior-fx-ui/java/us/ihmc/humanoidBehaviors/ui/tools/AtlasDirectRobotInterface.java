package us.ihmc.humanoidBehaviors.ui.tools;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.atlas.AtlasLowLevelControlMode;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.Ros2Node;

public class AtlasDirectRobotInterface implements RobotLowLevelMessenger
{
   private final IHMCROS2Publisher<AtlasLowLevelControlModeMessage> lowLevelModePublisher;
   private final IHMCROS2Publisher<BDIBehaviorCommandPacket> bdiBehaviorPublisher;
   private final IHMCROS2Publisher<AtlasDesiredPumpPSIPacket> desiredPumpPSIPublisher;
   private final IHMCROS2Publisher<AbortWalkingMessage> abortWalkingPublisher;
   private final IHMCROS2Publisher<PauseWalkingMessage> pauseWalkingPublisher;

   public AtlasDirectRobotInterface(Ros2Node ros2Node, DRCRobotModel robotModel)
   {
      ROS2Topic inputTopic = ROS2Tools.getControllerInputTopic(robotModel.getSimpleRobotName());
      lowLevelModePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, AtlasLowLevelControlModeMessage.class, inputTopic);
      bdiBehaviorPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, BDIBehaviorCommandPacket.class, inputTopic);
      desiredPumpPSIPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, AtlasDesiredPumpPSIPacket.class, inputTopic);
      abortWalkingPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, AbortWalkingMessage.class, inputTopic);
      pauseWalkingPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, PauseWalkingMessage.class, inputTopic);
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
   public void setHydraulicPumpPSI(int psi)
   {
      AtlasDesiredPumpPSIPacket message = new AtlasDesiredPumpPSIPacket();
      message.setDesiredPumpPsi(psi);
      desiredPumpPSIPublisher.publish(message);
   }
}
