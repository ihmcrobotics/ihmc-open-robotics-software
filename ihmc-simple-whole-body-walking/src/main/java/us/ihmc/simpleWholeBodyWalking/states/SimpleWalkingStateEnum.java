package us.ihmc.simpleWholeBodyWalking.states;

import us.ihmc.robotics.robotSide.RobotSide;

public enum SimpleWalkingStateEnum
{
   STANDING,
   TO_STANDING,

   TO_WALKING_LEFT_SUPPORT,
   TO_WALKING_RIGHT_SUPPORT,
   WALKING_LEFT_SUPPORT,
   WALKING_RIGHT_SUPPORT;


   public static SimpleWalkingStateEnum[] values = values();

   public static SimpleWalkingStateEnum getWalkingSingleSupportState(RobotSide supportLeg)
   {
      return supportLeg == RobotSide.LEFT ? WALKING_LEFT_SUPPORT : WALKING_RIGHT_SUPPORT;
   }

   public static SimpleWalkingStateEnum getWalkingTransferState(RobotSide transferToSide)
   {
      return transferToSide == RobotSide.LEFT ? TO_WALKING_LEFT_SUPPORT : TO_WALKING_RIGHT_SUPPORT;
   }


   public RobotSide getTransferToSide()
   {
      switch (this)
      {
      case TO_WALKING_LEFT_SUPPORT:
         return RobotSide.LEFT;

      case TO_WALKING_RIGHT_SUPPORT:
         return RobotSide.RIGHT;

      default:
         return null;
      }
   }

   public RobotSide getSupportSide()
   {
      switch (this)
      {
      case WALKING_LEFT_SUPPORT:
         return RobotSide.LEFT;

      case WALKING_RIGHT_SUPPORT:
         return RobotSide.RIGHT;

      default:
         return null;
      }
   }

   public boolean isDoubleSupport()
   {
      switch (this)
      {
      case STANDING:
      case TO_STANDING:
      case TO_WALKING_LEFT_SUPPORT:
      case TO_WALKING_RIGHT_SUPPORT:
         return true;

      case WALKING_LEFT_SUPPORT:
      case WALKING_RIGHT_SUPPORT:
         return false;

      default:
         throw new RuntimeException("Unknown " + getClass().getSimpleName() + " value: " + this);
      }
   }

   public boolean isSingleSupport()
   {
      return !isDoubleSupport();
   }
}
