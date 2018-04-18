package us.ihmc.valkyrie.fingers;

import us.ihmc.robotics.partNames.FingerName;
import us.ihmc.robotics.robotSide.RobotSide;

public enum ValkyrieFingerMotorName
{
   ThumbMotorRoll,
   ThumbMotorPitch1,
   ThumbMotorPitch2,
   IndexFingerMotorPitch1,
   MiddleFingerMotorPitch1,
   PinkyMotorPitch1;
   
   public static final ValkyrieFingerMotorName[] values = values();

   public String getJointName(RobotSide robotSide)
   {
      return getCamelCaseJointName(robotSide);
   }

   public String getCamelCaseJointName(RobotSide side)
   {
      return side.getCamelCaseName() + name();
   }

   public String getPascalCaseJointName(RobotSide side)
   {
      return side.getPascalCaseName() + name();
   }

   public FingerName getFingerName()
   {
      switch (this)
      {
      case ThumbMotorRoll:
      case ThumbMotorPitch1:
      case ThumbMotorPitch2:
         return FingerName.THUMB;
      case IndexFingerMotorPitch1:
         return FingerName.INDEX;
      case MiddleFingerMotorPitch1:
         return FingerName.MIDDLE;
      case PinkyMotorPitch1:
         return FingerName.PINKY;
      default:
         throw new RuntimeException("Unexpected " + getClass().getSimpleName() + " value: " + this);
      }
   }
}
