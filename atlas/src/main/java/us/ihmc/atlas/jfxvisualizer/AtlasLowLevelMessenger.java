package us.ihmc.atlas.jfxvisualizer;

import controller_msgs.msg.dds.AtlasLowLevelControlModeMessage;
import controller_msgs.msg.dds.BDIBehaviorCommandPacket;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.atlas.AtlasLowLevelControlMode;
import us.ihmc.ros2.RealtimeRos2Node;

public class AtlasLowLevelMessenger implements RobotLowLevelMessenger
{
   private final IHMCRealtimeROS2Publisher<AtlasLowLevelControlModeMessage> lowLevelModePublisher;
   private final IHMCRealtimeROS2Publisher<BDIBehaviorCommandPacket> bdiBehaviorPublisher;


   public AtlasLowLevelMessenger(RealtimeRos2Node ros2Node, String robotName)
   {
      lowLevelModePublisher = ROS2Tools.createPublisher(ros2Node, AtlasLowLevelControlModeMessage.class,
                                                        ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName));
      bdiBehaviorPublisher = ROS2Tools.createPublisher(ros2Node, BDIBehaviorCommandPacket.class,
                                                       ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName));
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
}
