package us.ihmc.valkyrie.network;

import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import controller_msgs.msg.dds.ValkyrieHandFingerTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.valkyrie.fingers.ValkyrieFingerMotorName;

public class ValkyrieMessageTools
{
   public static ValkyrieHandFingerTrajectoryMessage createValkyrieHandFingerTrajectoryMessage(RobotSide robotSide,
                                                                                               ValkyrieFingerMotorName[] valkyrieFingerMotors,
                                                                                               double trajectoryTime, double[] desiredJointPositions)
   {
      int dimension = valkyrieFingerMotors.length;
      byte[] valkyrieFingerMotorNames = new byte[dimension];
      for (int i = 0; i < dimension; i++)
         valkyrieFingerMotorNames[i] = valkyrieFingerMotors[i].toByte();
      return createValkyrieHandFingerTrajectoryMessage(robotSide, valkyrieFingerMotorNames, trajectoryTime, desiredJointPositions);
   }

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

   public static void appendDesiredFingerConfiguration(ValkyrieFingerMotorName motorNameToAppend, double time, double desiredConfiguration,
                                                       ValkyrieHandFingerTrajectoryMessage messageToAppend)
   {
      appendDesiredFingerConfiguration(motorNameToAppend.toByte(), time, desiredConfiguration, messageToAppend);
   }

   public static void appendDesiredFingerConfiguration(byte motorNameByteToAppend, double time, double desiredConfiguration,
                                                       ValkyrieHandFingerTrajectoryMessage messageToAppend)
   {
      messageToAppend.getFingerMotorNames().add(motorNameByteToAppend);
      Object<OneDoFJointTrajectoryMessage> jointTrajectoryMessages = messageToAppend.getJointspaceTrajectory().getJointTrajectoryMessages();
      jointTrajectoryMessages.add().set(HumanoidMessageTools.createOneDoFJointTrajectoryMessage(time, desiredConfiguration));
   }
}
