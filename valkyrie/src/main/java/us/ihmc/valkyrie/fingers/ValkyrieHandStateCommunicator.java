package us.ihmc.valkyrie.fingers;

import java.util.EnumMap;

import controller_msgs.msg.dds.HandJointAnglePacket;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ValkyrieHandStateCommunicator implements RobotController
{
   private final SideDependentList<EnumMap<ValkyrieHandJointName, OneDoFJoint>> handJoints = SideDependentList.createListOfEnumMaps(ValkyrieHandJointName.class);

   private final HandJointAnglePacket packet;
   private final IHMCRealtimeROS2Publisher<HandJointAnglePacket> publisher;

   public ValkyrieHandStateCommunicator(String robotName, FullHumanoidRobotModel fullRobotModel, ValkyrieHandModel handModel, RealtimeRos2Node realtimeRos2Node)
   {
      publisher = ROS2Tools.createPublisher(realtimeRos2Node, HandJointAnglePacket.class, ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName));

      for (RobotSide robotside : RobotSide.values)
      {
         for (ValkyrieHandJointName jointEnum : ValkyrieHandJointName.values)
         {
            OneDoFJoint joint = fullRobotModel.getOneDoFJointByName(jointEnum.getJointName(robotside));
            handJoints.get(robotside).put(jointEnum, joint);
         }
      }

      packet = HumanoidMessageTools.createHandJointAnglePacket(null, false, false, new double[ValkyrieHandJointName.values.length]);
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void doControl()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         packet.setRobotSide(robotSide.toByte());
         packet.getJointAngles().reset();

         for (ValkyrieHandJointName jointEnum : ValkyrieHandJointName.values)
         {
            double q = handJoints.get(robotSide).get(jointEnum).getQ();
            packet.getJointAngles().add(q);
         }
         publisher.publish(packet);
      }
   }

   @Override
   public String getDescription()
   {
      return null;
   }

   @Override
   public String getName()
   {
      return null;
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return null;
   }
}
