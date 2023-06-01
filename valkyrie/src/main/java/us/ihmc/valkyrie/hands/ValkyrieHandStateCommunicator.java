package us.ihmc.valkyrie.hands;

import controller_msgs.msg.dds.HandJointAnglePacket;
import us.ihmc.avatar.handControl.packetsAndConsumers.HandModel;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandJointName;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ValkyrieHandStateCommunicator implements RobotController
{
   private final SideDependentList<OneDoFJointBasics[]> handJointsInOrder = new SideDependentList<>();

   private final SideDependentList<HandJointAnglePacket> packets = new SideDependentList<>();
   private final IHMCRealtimeROS2Publisher<HandJointAnglePacket> publisher;

   public ValkyrieHandStateCommunicator(FullHumanoidRobotModel fullRobotModel,
                                        SideDependentList<? extends HandModel> handModels,
                                        RealtimeROS2Node realtimeROS2Node,
                                        ROS2Topic<?> outputTopic)
   {
      publisher = ROS2Tools.createPublisherTypeNamed(realtimeROS2Node, HandJointAnglePacket.class, outputTopic);

      for (RobotSide robotSide : RobotSide.values)
      {
         HandModel handModel = handModels.get(robotSide);

         if (handModel == null)
         {
            LogTools.error("No {} hand model provided.", robotSide);
            handJointsInOrder.put(robotSide, new OneDoFJointBasics[0]);
            continue;
         }

         OneDoFJointBasics[] joints = new OneDoFJointBasics[handModel.getHandJointNames().length];
         HandJointName[] handJointNames = handModel.getHandJointNames();

         for (int i = 0; i < handJointNames.length; i++)
         {
            joints[i] = fullRobotModel.getOneDoFJointByName(handJointNames[i].getJointName(robotSide));
         }
         handJointsInOrder.put(robotSide, joints);

         packets.put(robotSide, HumanoidMessageTools.createHandJointAnglePacket(null, false, false, new double[joints.length]));
      }
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
         HandJointAnglePacket packet = packets.get(robotSide);
         packet.setRobotSide(robotSide.toByte());
         packet.getJointAngles().reset();

         OneDoFJointBasics[] joints = handJointsInOrder.get(robotSide);

         for (OneDoFJointBasics joint : joints)
         {
            packet.getJointAngles().add(joint.getQ());
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
   public YoRegistry getYoRegistry()
   {
      return null;
   }
}
