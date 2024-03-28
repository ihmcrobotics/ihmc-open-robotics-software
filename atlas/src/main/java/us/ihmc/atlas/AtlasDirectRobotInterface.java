package us.ihmc.atlas;

import atlas_msgs.msg.dds.AtlasDesiredPumpPSIPacket;
import atlas_msgs.msg.dds.AtlasLowLevelControlModeMessage;
import atlas_msgs.msg.dds.BDIBehaviorCommandPacket;
import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.atlas.AtlasLowLevelControlMode;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;

public class AtlasDirectRobotInterface implements RobotLowLevelMessenger
{
   private final ROS2PublisherBasics<AtlasLowLevelControlModeMessage> lowLevelModePublisher;
   private final ROS2PublisherBasics<BDIBehaviorCommandPacket> bdiBehaviorPublisher;
   private final ROS2PublisherBasics<AtlasDesiredPumpPSIPacket> desiredPumpPSIPublisher;
   private final ROS2PublisherBasics<AbortWalkingMessage> abortWalkingPublisher;
   private final ROS2PublisherBasics<PauseWalkingMessage> pauseWalkingPublisher;

   public AtlasDirectRobotInterface(ROS2NodeInterface ros2Node, DRCRobotModel robotModel)
   {
      ROS2Topic inputTopic = HumanoidControllerAPI.getInputTopic(robotModel.getSimpleRobotName());
      lowLevelModePublisher = ros2Node.createPublisher(ROS2Tools.typeNamedTopic(AtlasLowLevelControlModeMessage.class).withTopic(inputTopic));
      bdiBehaviorPublisher = ros2Node.createPublisher(ROS2Tools.typeNamedTopic(BDIBehaviorCommandPacket.class).withTopic(inputTopic));
      desiredPumpPSIPublisher = ros2Node.createPublisher(ROS2Tools.typeNamedTopic(AtlasDesiredPumpPSIPacket.class).withTopic(inputTopic));
      abortWalkingPublisher = ros2Node.createPublisher(ROS2Tools.typeNamedTopic(AbortWalkingMessage.class).withTopic(inputTopic));
      pauseWalkingPublisher = ros2Node.createPublisher(ROS2Tools.typeNamedTopic(PauseWalkingMessage.class).withTopic(inputTopic));
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
