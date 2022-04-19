package us.ihmc.avatar.drcRobot;

import controller_msgs.msg.dds.HandJointAnglePacket;
import us.ihmc.avatar.handControl.packetsAndConsumers.HandModel;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandJointName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.HashMap;

public class HandModelUtils
{
   public static void getHandJoints(HandModel handModel,
                                    FullHumanoidRobotModel fullRobotModel,
                                    SideDependentList<HashMap<HandJointName, OneDoFJointBasics>> handJoints)
   {
      for (RobotSide side : RobotSide.values)
      {
         final HashMap<HandJointName, OneDoFJointBasics> joints = new HashMap<>();

         for (HandJointName jointName : handModel.getHandJointNames())
         {
            joints.put(jointName, fullRobotModel.getOneDoFJointByName(jointName.getJointName(side)));
         }

         handJoints.put(side, joints);
      }
   }

   public static void copyHandJointAnglesFromMessagesToOneDoFJoints(HandModel handModel,
                                                                    SideDependentList<HashMap<HandJointName, OneDoFJointBasics>> handJoints,
                                                                    SideDependentList<HandJointAnglePacket> handJointAnglePackets)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         HandJointAnglePacket handJointAnglePacket = handJointAnglePackets.get(robotSide);
         if (handJointAnglePacket != null)
         {
            for (HandJointName jointName : handModel.getHandJointNames())
            {
               double jointAngle = HumanoidMessageTools.unpackJointAngle(handJointAnglePacket, jointName);
               OneDoFJointBasics oneDoFJoint = handJoints.get(robotSide).get(jointName);
               oneDoFJoint.setQ(jointAngle);
            }
         }
      }
   }
}
