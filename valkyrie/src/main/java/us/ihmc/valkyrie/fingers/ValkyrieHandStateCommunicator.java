package us.ihmc.valkyrie.fingers;

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
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.valkyrie.fingers.valkyrieHand.ValkyrieHandModel.ValkyrieHandJointName;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ValkyrieHandStateCommunicator implements RobotController
{
   private final SideDependentList<OneDoFJointBasics[]> handJointsInOrder = new SideDependentList<>();

   private final HandJointAnglePacket packet;
   private final IHMCRealtimeROS2Publisher<HandJointAnglePacket> publisher;

   public ValkyrieHandStateCommunicator(String robotName,
                                        FullHumanoidRobotModel fullRobotModel,
                                        SideDependentList<HandModel> handModels,
                                        RealtimeROS2Node realtimeROS2Node)
   {
      publisher = ROS2Tools.createPublisherTypeNamed(realtimeROS2Node, HandJointAnglePacket.class, ROS2Tools.getControllerOutputTopic(robotName));

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
