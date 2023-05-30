package us.ihmc.valkyrie.fingers.valkyrieHand;

import us.ihmc.avatar.handControl.packetsAndConsumers.HandModel;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandJointName;
import us.ihmc.robotics.partNames.FingerName;
import us.ihmc.robotics.robotSide.RobotSide;

public class ValkyrieHandModel implements HandModel
{
   @Override
   public ValkyrieHandJointName[] getHandJointNames()
   {
      return ValkyrieHandJointName.values;
   }

   public static enum ValkyrieHandJointName implements HandJointName
   {
      ThumbRoll,
      ThumbPitch1,
      ThumbPitch2,
      ThumbPitch3,
      IndexFingerPitch1,
      IndexFingerPitch2,
      IndexFingerPitch3,
      MiddleFingerPitch1,
      MiddleFingerPitch2,
      MiddleFingerPitch3,
      PinkyPitch1,
      PinkyPitch2,
      PinkyPitch3;

      public static final ValkyrieHandJointName[] values = values();

      @Override
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
            case ThumbRoll:
            case ThumbPitch1:
            case ThumbPitch2:
            case ThumbPitch3:
               return FingerName.THUMB;
            case IndexFingerPitch1:
            case IndexFingerPitch2:
            case IndexFingerPitch3:
               return FingerName.INDEX;
            case MiddleFingerPitch1:
            case MiddleFingerPitch2:
            case MiddleFingerPitch3:
               return FingerName.MIDDLE;
            case PinkyPitch1:
            case PinkyPitch2:
            case PinkyPitch3:
               return FingerName.PINKY;
            default:
               throw new RuntimeException("Unexpected " + getClass().getSimpleName() + " value: " + this);
         }
      }

      @Override
      public int getIndex(RobotSide robotSide)
      {
         return ordinal();
      }
   }

   public static enum ValkyrieFingerMotorName
   {
      ThumbMotorRoll, ThumbMotorPitch1, ThumbMotorPitch2, IndexFingerMotorPitch1, MiddleFingerMotorPitch1, PinkyMotorPitch1;

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

      public ValkyrieHandJointName getCorrespondingJointName(int depth)
      {
         switch (this)
         {
            case ThumbMotorRoll:
               return ValkyrieHandJointName.ThumbRoll;
            case ThumbMotorPitch1:
            case ThumbMotorPitch2:
               if (depth == 1)
                  return ValkyrieHandJointName.ThumbPitch1;
               else if (depth == 2)
                  return ValkyrieHandJointName.ThumbPitch2;
               else
                  return ValkyrieHandJointName.ThumbPitch3;

            case IndexFingerMotorPitch1:
               if (depth == 1)
                  return ValkyrieHandJointName.IndexFingerPitch1;
               else if (depth == 2)
                  return ValkyrieHandJointName.IndexFingerPitch2;
               else
                  return ValkyrieHandJointName.IndexFingerPitch3;

            case MiddleFingerMotorPitch1:
               if (depth == 1)
                  return ValkyrieHandJointName.MiddleFingerPitch1;
               else if (depth == 2)
                  return ValkyrieHandJointName.MiddleFingerPitch2;
               else
                  return ValkyrieHandJointName.MiddleFingerPitch3;

            case PinkyMotorPitch1:
               if (depth == 1)
                  return ValkyrieHandJointName.PinkyPitch1;
               else if (depth == 2)
                  return ValkyrieHandJointName.PinkyPitch2;
               else
                  return ValkyrieHandJointName.PinkyPitch3;

            default:
               return null;
         }
      }

      public byte toByte()
      {
         return (byte) ordinal();
      }

      public static ValkyrieFingerMotorName fromByte(byte enumAsByte)
      {
         if (enumAsByte == -1)
            return null;
         return values[enumAsByte];
      }
   }
}
