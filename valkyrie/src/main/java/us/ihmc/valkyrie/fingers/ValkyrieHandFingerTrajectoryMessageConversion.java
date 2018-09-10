package us.ihmc.valkyrie.fingers;

import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import controller_msgs.msg.dds.ValkyrieHandFingerTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * This class converts desired HandConfiguration into a ValkyrieHandFingerTrajectoryMessage.
 * Finger controller will stop finger motors which of trajectory is not specified.
 */

public class ValkyrieHandFingerTrajectoryMessageConversion
{
   public static double trajectoryTime = 3.0;
   public static double extendedTimeRatioForThumb = 1.5;
   public static double basicGripThumbRollClosingTime = 1.0; // TODO.

   public static double trajectoryTimeForSim = 0.5;

   public static final void convertHandDesiredConfigurationMessage(HandDesiredConfigurationMessage handDesiredConfigurationMessage,
                                                                   ValkyrieHandFingerTrajectoryMessage messageToPack)
   {
      messageToPack.setRobotSide(handDesiredConfigurationMessage.getRobotSide());
      HandConfiguration desiredHandConfiguration = HandConfiguration.fromByte(handDesiredConfigurationMessage.getDesiredHandConfiguration());
      convertHandConfiguration(desiredHandConfiguration, messageToPack);
   }

   public static final void convertHandConfiguration(RobotSide robotSide, HandConfiguration desiredHandConfiguration,
                                                     ValkyrieHandFingerTrajectoryMessage messageToPack)
   {
      messageToPack.setRobotSide(robotSide.toByte());
      convertHandConfiguration(desiredHandConfiguration, messageToPack);
   }

   public static final void convertHandConfiguration(HandConfiguration desiredHandConfiguration, ValkyrieHandFingerTrajectoryMessage messageToPack)
   {
      ValkyrieFingerMotorName[] valkyrieFingerMotorNames = null;
      double[] trajectoryTimes = null;
      double[] desiredJointPositions = null;

      switch (desiredHandConfiguration)
      {
      case STOP:
         break;

      case OPEN:
         valkyrieFingerMotorNames = new ValkyrieFingerMotorName[ValkyrieFingerMotorName.values.length];
         valkyrieFingerMotorNames[0] = ValkyrieFingerMotorName.ThumbMotorRoll;
         valkyrieFingerMotorNames[1] = ValkyrieFingerMotorName.ThumbMotorPitch1;
         valkyrieFingerMotorNames[2] = ValkyrieFingerMotorName.ThumbMotorPitch2;
         valkyrieFingerMotorNames[3] = ValkyrieFingerMotorName.IndexFingerMotorPitch1;
         valkyrieFingerMotorNames[4] = ValkyrieFingerMotorName.MiddleFingerMotorPitch1;
         valkyrieFingerMotorNames[5] = ValkyrieFingerMotorName.PinkyMotorPitch1;

         trajectoryTimes = new double[] {trajectoryTime, trajectoryTime, trajectoryTime, trajectoryTime * extendedTimeRatioForThumb,
               trajectoryTime * extendedTimeRatioForThumb, trajectoryTime * extendedTimeRatioForThumb};
         desiredJointPositions = new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
         break;

      case CLOSE:
         valkyrieFingerMotorNames = new ValkyrieFingerMotorName[ValkyrieFingerMotorName.values.length];
         valkyrieFingerMotorNames[0] = ValkyrieFingerMotorName.ThumbMotorRoll;
         valkyrieFingerMotorNames[1] = ValkyrieFingerMotorName.ThumbMotorPitch1;
         valkyrieFingerMotorNames[2] = ValkyrieFingerMotorName.ThumbMotorPitch2;
         valkyrieFingerMotorNames[3] = ValkyrieFingerMotorName.IndexFingerMotorPitch1;
         valkyrieFingerMotorNames[4] = ValkyrieFingerMotorName.MiddleFingerMotorPitch1;
         valkyrieFingerMotorNames[5] = ValkyrieFingerMotorName.PinkyMotorPitch1;

         trajectoryTimes = new double[] {trajectoryTime * extendedTimeRatioForThumb, trajectoryTime * extendedTimeRatioForThumb,
               trajectoryTime * extendedTimeRatioForThumb, trajectoryTime, trajectoryTime, trajectoryTime};
         desiredJointPositions = new double[] {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
         break;

      case OPEN_FINGERS:
         valkyrieFingerMotorNames = new ValkyrieFingerMotorName[3];
         valkyrieFingerMotorNames[0] = ValkyrieFingerMotorName.IndexFingerMotorPitch1;
         valkyrieFingerMotorNames[1] = ValkyrieFingerMotorName.MiddleFingerMotorPitch1;
         valkyrieFingerMotorNames[2] = ValkyrieFingerMotorName.PinkyMotorPitch1;

         trajectoryTimes = new double[] {trajectoryTime, trajectoryTime, trajectoryTime};
         desiredJointPositions = new double[] {0.0, 0.0, 0.0};
         break;

      case OPEN_THUMB:
         valkyrieFingerMotorNames = new ValkyrieFingerMotorName[3];
         valkyrieFingerMotorNames[0] = ValkyrieFingerMotorName.ThumbMotorRoll;
         valkyrieFingerMotorNames[1] = ValkyrieFingerMotorName.ThumbMotorPitch1;
         valkyrieFingerMotorNames[2] = ValkyrieFingerMotorName.ThumbMotorPitch2;

         trajectoryTimes = new double[] {trajectoryTime, trajectoryTime, trajectoryTime};
         desiredJointPositions = new double[] {0.0, 0.0, 0.0};
         break;

      case CLOSE_FINGERS:
         valkyrieFingerMotorNames = new ValkyrieFingerMotorName[3];
         valkyrieFingerMotorNames[0] = ValkyrieFingerMotorName.IndexFingerMotorPitch1;
         valkyrieFingerMotorNames[1] = ValkyrieFingerMotorName.MiddleFingerMotorPitch1;
         valkyrieFingerMotorNames[2] = ValkyrieFingerMotorName.PinkyMotorPitch1;

         trajectoryTimes = new double[] {trajectoryTime, trajectoryTime, trajectoryTime};
         desiredJointPositions = new double[] {1.0, 1.0, 1.0};
         break;

      case CLOSE_THUMB:
         valkyrieFingerMotorNames = new ValkyrieFingerMotorName[3];
         valkyrieFingerMotorNames[0] = ValkyrieFingerMotorName.ThumbMotorRoll;
         valkyrieFingerMotorNames[1] = ValkyrieFingerMotorName.ThumbMotorPitch1;
         valkyrieFingerMotorNames[2] = ValkyrieFingerMotorName.ThumbMotorPitch2;

         trajectoryTimes = new double[] {trajectoryTime, trajectoryTime, trajectoryTime};
         desiredJointPositions = new double[] {1.0, 1.0, 1.0};
         break;

      default:
         throw new IllegalArgumentException("Message conversion for the desired HandConfiguration is not implemented");
      }

      if (valkyrieFingerMotorNames != null)
      {
         int dimension = valkyrieFingerMotorNames.length;
         messageToPack.getJointspaceTrajectory().set(HumanoidMessageTools.createJointspaceTrajectoryMessage(trajectoryTimes, desiredJointPositions));
         for (int i = 0; i < dimension; i++)
         {
            messageToPack.getFingerMotorNames().add(valkyrieFingerMotorNames[i].toByte());
         }
      }
   }
}
