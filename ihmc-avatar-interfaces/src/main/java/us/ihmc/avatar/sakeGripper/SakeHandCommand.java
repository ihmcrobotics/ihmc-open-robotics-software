package us.ihmc.avatar.sakeGripper;

import controller_msgs.msg.dds.SakeHandDesiredCommandMessage;

public class SakeHandCommand
{
   private double desiredTorqueRatio;
   private double desiredPositionRatio;
   private SakeHandConfiguration handConfiguration;

   public SakeHandCommand()
   {

   }

   // FIXME: This seems like we could find a better design here.
   public void setFromLegacyHandConfiguration(us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration legacyHandConfiguration)
   {
      desiredTorqueRatio = 0.0;
      desiredPositionRatio = 0.0;
      switch (legacyHandConfiguration)
      {
         case CLOSE:
            handConfiguration = SakeHandConfiguration.CLOSE;
            desiredPositionRatio = 0.1;
            desiredTorqueRatio = 0.3;
            break;
         case OPEN:
            handConfiguration = SakeHandConfiguration.OPEN;
            desiredPositionRatio = 1.0;
            desiredTorqueRatio = 0.3;
            break;
         case STOP:
            handConfiguration = SakeHandConfiguration.RELEASE;
            desiredPositionRatio = 1.0;
            desiredTorqueRatio = 0.3;
            break;
         case CRUSH:
            handConfiguration = SakeHandConfiguration.GRIP_HARD;
            break;
         case BASIC_GRIP:
            handConfiguration = SakeHandConfiguration.GRIP_WITH_TORQUE;
            desiredTorqueRatio = 0.3;
            break;
         case RESET:
            handConfiguration = SakeHandConfiguration.RESET;
            desiredPositionRatio = 1.0;
            desiredTorqueRatio = 0.3;
            break;
         case CALIBRATE:
            handConfiguration = SakeHandConfiguration.CALIBRATE;
            desiredPositionRatio = 0.0;
            desiredTorqueRatio = 0.3;
            break;
         case HALF_CLOSE:
            handConfiguration = SakeHandConfiguration.GOTO_POSITION_WITH_TORQUE;
            desiredPositionRatio = 0.5;
            desiredTorqueRatio = 0.3;
            break;
         default:
            handConfiguration = null;
            break;
      }
   }

   public void setFromMessage(SakeHandDesiredCommandMessage desiredCommandMessage)
   {
      if (desiredCommandMessage.getTorqueRatio() >= 0)
         desiredTorqueRatio = desiredCommandMessage.getTorqueRatio();
      if (desiredCommandMessage.getPostionRatio() >= 0)
         desiredPositionRatio = desiredCommandMessage.getPostionRatio();
      handConfiguration = SakeHandConfiguration.fromByte(desiredCommandMessage.getDesiredHandConfiguration());
   }

   public double getDesiredTorqueRatio()
   {
      return desiredTorqueRatio;
   }

   public double getDesiredPositionRatio()
   {
      return desiredPositionRatio;
   }

   public SakeHandConfiguration getDesiredHandConfiguration()
   {
      return handConfiguration;
   }
}
