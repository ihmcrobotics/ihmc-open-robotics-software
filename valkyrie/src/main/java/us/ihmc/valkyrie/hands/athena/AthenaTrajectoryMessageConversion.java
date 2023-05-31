package us.ihmc.valkyrie.hands.athena;

import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.valkyrie.hands.athena.AthenaHandModel.AthenaFingerMotorName;
import valkyrie_msgs.msg.dds.AthenaTrajectoryMessage;

/**
 * This class converts desired HandConfiguration into a ValkyrieHandFingerTrajectoryMessage. Finger
 * controller will stop finger motors which of trajectory is not specified.
 */

public class AthenaTrajectoryMessageConversion
{
   /*
    * TODO Somehow make it YoDoubles again. The fingers seem to be pretty slow, so kinda pointless
    * reducing this one. In addition, the comms to the fingers seems flaky, going faster increases the
    * chances to trigger a limit fault.
    */
   public static double trajectoryTime = 5.0;
   public static double extendedTimeRatioForThumb = 1.25; // Making sure the thumb does not crush the finger when closing.
   public static double basicGripThumbRollClosingTime = 1.0; // TODO.

   public static double trajectoryTimeForSim = 0.5;

   public static final void convertHandDesiredConfigurationMessage(HandDesiredConfigurationMessage handDesiredConfigurationMessage,
                                                                   AthenaTrajectoryMessage messageToPack)
   {
      messageToPack.setRobotSide(handDesiredConfigurationMessage.getRobotSide());
      HandConfiguration desiredHandConfiguration = HandConfiguration.fromByte(handDesiredConfigurationMessage.getDesiredHandConfiguration());
      convertHandConfiguration(desiredHandConfiguration, messageToPack);
   }

   public static final void convertHandConfiguration(RobotSide robotSide,
                                                     HandConfiguration desiredHandConfiguration,
                                                     AthenaTrajectoryMessage messageToPack)
   {
      messageToPack.setRobotSide(robotSide.toByte());
      convertHandConfiguration(desiredHandConfiguration, messageToPack);
   }

   public static final void convertHandConfiguration(HandConfiguration desiredHandConfiguration, AthenaTrajectoryMessage messageToPack)
   {
      AthenaFingerMotorName[] valkyrieFingerMotorNames = null;
      double[] trajectoryTimes = null;
      double[] desiredJointPositions = null;

      switch (desiredHandConfiguration)
      {
         case STOP:
            break;

         case OPEN:
            valkyrieFingerMotorNames = new AthenaFingerMotorName[AthenaFingerMotorName.values.length];
            valkyrieFingerMotorNames[0] = AthenaFingerMotorName.ThumbMotorRoll;
            valkyrieFingerMotorNames[1] = AthenaFingerMotorName.ThumbMotorPitch1;
            valkyrieFingerMotorNames[2] = AthenaFingerMotorName.ThumbMotorPitch2;
            valkyrieFingerMotorNames[3] = AthenaFingerMotorName.IndexFingerMotorPitch1;
            valkyrieFingerMotorNames[4] = AthenaFingerMotorName.MiddleFingerMotorPitch1;
            valkyrieFingerMotorNames[5] = AthenaFingerMotorName.PinkyMotorPitch1;

            trajectoryTimes = new double[] {trajectoryTime,
                                            trajectoryTime,
                                            trajectoryTime,
                                            trajectoryTime * extendedTimeRatioForThumb,
                                            trajectoryTime * extendedTimeRatioForThumb,
                                            trajectoryTime * extendedTimeRatioForThumb};
            desiredJointPositions = new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            break;

         case CLOSE:
            valkyrieFingerMotorNames = new AthenaFingerMotorName[AthenaFingerMotorName.values.length];
            valkyrieFingerMotorNames[0] = AthenaFingerMotorName.ThumbMotorRoll;
            valkyrieFingerMotorNames[1] = AthenaFingerMotorName.ThumbMotorPitch1;
            valkyrieFingerMotorNames[2] = AthenaFingerMotorName.ThumbMotorPitch2;
            valkyrieFingerMotorNames[3] = AthenaFingerMotorName.IndexFingerMotorPitch1;
            valkyrieFingerMotorNames[4] = AthenaFingerMotorName.MiddleFingerMotorPitch1;
            valkyrieFingerMotorNames[5] = AthenaFingerMotorName.PinkyMotorPitch1;

            trajectoryTimes = new double[] {trajectoryTime * extendedTimeRatioForThumb,
                                            trajectoryTime * extendedTimeRatioForThumb,
                                            trajectoryTime * extendedTimeRatioForThumb,
                                            trajectoryTime,
                                            trajectoryTime,
                                            trajectoryTime};
            desiredJointPositions = new double[] {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
            break;

         case OPEN_FINGERS:
            valkyrieFingerMotorNames = new AthenaFingerMotorName[3];
            valkyrieFingerMotorNames[0] = AthenaFingerMotorName.IndexFingerMotorPitch1;
            valkyrieFingerMotorNames[1] = AthenaFingerMotorName.MiddleFingerMotorPitch1;
            valkyrieFingerMotorNames[2] = AthenaFingerMotorName.PinkyMotorPitch1;

            trajectoryTimes = new double[] {trajectoryTime, trajectoryTime, trajectoryTime};
            desiredJointPositions = new double[] {0.0, 0.0, 0.0};
            break;

         case OPEN_THUMB:
            valkyrieFingerMotorNames = new AthenaFingerMotorName[3];
            valkyrieFingerMotorNames[0] = AthenaFingerMotorName.ThumbMotorRoll;
            valkyrieFingerMotorNames[1] = AthenaFingerMotorName.ThumbMotorPitch1;
            valkyrieFingerMotorNames[2] = AthenaFingerMotorName.ThumbMotorPitch2;

            trajectoryTimes = new double[] {trajectoryTime, trajectoryTime, trajectoryTime};
            desiredJointPositions = new double[] {0.0, 0.0, 0.0};
            break;

         case CLOSE_FINGERS:
            valkyrieFingerMotorNames = new AthenaFingerMotorName[3];
            valkyrieFingerMotorNames[0] = AthenaFingerMotorName.IndexFingerMotorPitch1;
            valkyrieFingerMotorNames[1] = AthenaFingerMotorName.MiddleFingerMotorPitch1;
            valkyrieFingerMotorNames[2] = AthenaFingerMotorName.PinkyMotorPitch1;

            trajectoryTimes = new double[] {trajectoryTime, trajectoryTime, trajectoryTime};
            desiredJointPositions = new double[] {1.0, 1.0, 1.0};
            break;

         case CLOSE_THUMB:
            valkyrieFingerMotorNames = new AthenaFingerMotorName[3];
            valkyrieFingerMotorNames[0] = AthenaFingerMotorName.ThumbMotorRoll;
            valkyrieFingerMotorNames[1] = AthenaFingerMotorName.ThumbMotorPitch1;
            valkyrieFingerMotorNames[2] = AthenaFingerMotorName.ThumbMotorPitch2;

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
