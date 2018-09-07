package us.ihmc.valkyrie.network;

import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import controller_msgs.msg.dds.ValkyrieHandFingerTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.robotics.robotSide.RobotSide;

public class ValkyrieMessageTools
{
   public static ValkyrieHandFingerTrajectoryMessage createValkyrieHandFingerTrajectoryMessage(RobotSide robotSide, byte[] valkyrieFingerMotorNames,
                                                                                               double trajectoryTime, double[] desiredJointPositions)
   {
      ValkyrieHandFingerTrajectoryMessage message = new ValkyrieHandFingerTrajectoryMessage();

      message.setRobotSide(robotSide.toByte());

      int dimension = valkyrieFingerMotorNames.length;

      for (int i = 0; i < dimension; i++)
         message.getFingerMotorNames().add(valkyrieFingerMotorNames[i]);

      message.getJointspaceTrajectory().set(HumanoidMessageTools.createJointspaceTrajectoryMessage(trajectoryTime, desiredJointPositions));

      return message;
   }

   public static void appendDesiredFingerConfiguration(byte motorNameByteToAppend, double time, double desiredConfiguration,
                                                       ValkyrieHandFingerTrajectoryMessage messageToAppend)
   {
      messageToAppend.getFingerMotorNames().add(motorNameByteToAppend);
      Object<OneDoFJointTrajectoryMessage> jointTrajectoryMessages = messageToAppend.getJointspaceTrajectory().getJointTrajectoryMessages();
      jointTrajectoryMessages.add().set(HumanoidMessageTools.createOneDoFJointTrajectoryMessage(time, desiredConfiguration));
   }
}
