package us.ihmc.valkyrie.network;

import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.OneDoFTrajectoryPointList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.valkyrie.hands.athena.AthenaHandModel.AthenaFingerMotorName;
import valkyrie_msgs.msg.dds.AthenaTrajectoryMessage;

public class ValkyrieMessageTools
{
   public static AthenaTrajectoryMessage createAthenaTrajectoryMessage(RobotSide robotSide,
                                                                       AthenaFingerMotorName[] athenaFingerMotors,
                                                                       double trajectoryTime,
                                                                       double[] desiredJointPositions)
   {
      int dimension = athenaFingerMotors.length;
      byte[] fingerMotorNames = new byte[dimension];
      for (int i = 0; i < dimension; i++)
         fingerMotorNames[i] = athenaFingerMotors[i].toByte();
      return createAthenaTrajectoryMessage(robotSide, fingerMotorNames, trajectoryTime, desiredJointPositions);
   }

   public static AthenaTrajectoryMessage createAthenaTrajectoryMessage(RobotSide robotSide,
                                                                       byte[] fingerMotorNames,
                                                                       double trajectoryTime,
                                                                       double[] desiredJointPositions)
   {
      if (fingerMotorNames.length != desiredJointPositions.length)
         throw new RuntimeException("number of fingers should be same with number of trajectory data " + fingerMotorNames.length);

      AthenaTrajectoryMessage message = new AthenaTrajectoryMessage();

      message.setRobotSide(robotSide.toByte());

      int dimension = fingerMotorNames.length;

      for (int i = 0; i < dimension; i++)
         message.getFingerMotorNames().add(fingerMotorNames[i]);

      message.getJointspaceTrajectory().set(HumanoidMessageTools.createJointspaceTrajectoryMessage(trajectoryTime, desiredJointPositions));

      return message;
   }

   public static AthenaTrajectoryMessage createAthenaTrajectoryMessage(RobotSide robotSide,
                                                                       AthenaFingerMotorName[] athenaFingerMotors,
                                                                       OneDoFTrajectoryPointList[] trajectoryData)
   {
      int dimension = athenaFingerMotors.length;
      byte[] valkyrieFingerMotorNames = new byte[dimension];
      for (int i = 0; i < dimension; i++)
         valkyrieFingerMotorNames[i] = athenaFingerMotors[i].toByte();
      return createAthenaTrajectoryMessage(robotSide, valkyrieFingerMotorNames, trajectoryData);
   }

   public static AthenaTrajectoryMessage createAthenaTrajectoryMessage(RobotSide robotSide, byte[] fingerMotorNames, OneDoFTrajectoryPointList[] trajectoryData)
   {
      if (fingerMotorNames.length != trajectoryData.length)
         throw new RuntimeException("number of fingers should be same with number of trajectory data " + fingerMotorNames.length);

      AthenaTrajectoryMessage message = new AthenaTrajectoryMessage();

      message.setRobotSide(robotSide.toByte());

      int dimension = fingerMotorNames.length;

      for (int i = 0; i < dimension; i++)
      {
         message.getFingerMotorNames().add(fingerMotorNames[i]);
         message.getJointspaceTrajectory().getJointTrajectoryMessages().add(HumanoidMessageTools.createOneDoFJointTrajectoryMessage(trajectoryData[i]));
      }

      return message;
   }

   public static void appendDesiredFingerConfiguration(AthenaFingerMotorName motorNameToAppend,
                                                       double time,
                                                       double desiredConfiguration,
                                                       AthenaTrajectoryMessage messageToAppend)
   {
      appendDesiredFingerConfiguration(motorNameToAppend.toByte(), time, desiredConfiguration, messageToAppend);
   }

   public static void appendDesiredFingerConfiguration(byte motorNameByteToAppend,
                                                       double time,
                                                       double desiredConfiguration,
                                                       AthenaTrajectoryMessage messageToAppend)
   {
      messageToAppend.getFingerMotorNames().add(motorNameByteToAppend);
      Object<OneDoFJointTrajectoryMessage> jointTrajectoryMessages = messageToAppend.getJointspaceTrajectory().getJointTrajectoryMessages();
      jointTrajectoryMessages.add().set(HumanoidMessageTools.createOneDoFJointTrajectoryMessage(time, desiredConfiguration));
   }

   public static void appendDesiredFingerConfiguration(AthenaFingerMotorName motorNameToAppend,
                                                       OneDoFTrajectoryPointList trajectoryData,
                                                       AthenaTrajectoryMessage messageToAppend)
   {
      appendDesiredFingerConfiguration(motorNameToAppend.toByte(), trajectoryData, messageToAppend);
   }

   public static void appendDesiredFingerConfiguration(byte motorNameByteToAppend,
                                                       OneDoFTrajectoryPointList trajectoryData,
                                                       AthenaTrajectoryMessage messageToAppend)
   {
      messageToAppend.getFingerMotorNames().add(motorNameByteToAppend);
      Object<OneDoFJointTrajectoryMessage> jointTrajectoryMessages = messageToAppend.getJointspaceTrajectory().getJointTrajectoryMessages();
      jointTrajectoryMessages.add().set(HumanoidMessageTools.createOneDoFJointTrajectoryMessage(trajectoryData));
   }
}
