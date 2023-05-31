package us.ihmc.valkyrie.hands.athena;

import us.ihmc.avatar.handControl.packetsAndConsumers.HandModel;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandJointName;
import us.ihmc.robotics.partNames.FingerName;
import us.ihmc.robotics.robotSide.RobotSide;

public class AthenaHandModel implements HandModel
{
   @Override
   public AthenaJointName[] getHandJointNames()
   {
      return AthenaJointName.values;
   }

   public static enum AthenaJointName implements HandJointName
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

      public static final AthenaJointName[] values = values();

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

   public static enum AthenaFingerMotorName
   {
      ThumbMotorRoll, ThumbMotorPitch1, ThumbMotorPitch2, IndexFingerMotorPitch1, MiddleFingerMotorPitch1, PinkyMotorPitch1;

      public static final AthenaFingerMotorName[] values = values();

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

      public AthenaJointName getCorrespondingJointName(int depth)
      {
         switch (this)
         {
            case ThumbMotorRoll:
               return AthenaJointName.ThumbRoll;
            case ThumbMotorPitch1:
            case ThumbMotorPitch2:
               if (depth == 1)
                  return AthenaJointName.ThumbPitch1;
               else if (depth == 2)
                  return AthenaJointName.ThumbPitch2;
               else
                  return AthenaJointName.ThumbPitch3;

            case IndexFingerMotorPitch1:
               if (depth == 1)
                  return AthenaJointName.IndexFingerPitch1;
               else if (depth == 2)
                  return AthenaJointName.IndexFingerPitch2;
               else
                  return AthenaJointName.IndexFingerPitch3;

            case MiddleFingerMotorPitch1:
               if (depth == 1)
                  return AthenaJointName.MiddleFingerPitch1;
               else if (depth == 2)
                  return AthenaJointName.MiddleFingerPitch2;
               else
                  return AthenaJointName.MiddleFingerPitch3;

            case PinkyMotorPitch1:
               if (depth == 1)
                  return AthenaJointName.PinkyPitch1;
               else if (depth == 2)
                  return AthenaJointName.PinkyPitch2;
               else
                  return AthenaJointName.PinkyPitch3;

            default:
               return null;
         }
      }

      public byte toByte()
      {
         return (byte) ordinal();
      }

      public static AthenaFingerMotorName fromByte(byte enumAsByte)
      {
         if (enumAsByte == -1)
            return null;
         return values[enumAsByte];
      }
   }
}
