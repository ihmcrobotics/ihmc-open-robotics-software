package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.states;

import us.ihmc.robotics.robotSide.RobotSide;

public enum PushRecoveryStateEnum
{
   TO_STANDING,

   TO_PUSH_RECOVERY_LEFT_SUPPORT,
   TO_PUSH_RECOVERY_RIGHT_SUPPORT,
   PUSH_RECOVERY_LEFT_SUPPORT,
   PUSH_RECOVERY_RIGHT_SUPPORT;
   
   public static PushRecoveryStateEnum[] values = values();

   public static PushRecoveryStateEnum getPushRecoverySingleSupportState(RobotSide supportLeg)
   {
      return supportLeg == RobotSide.LEFT ? PUSH_RECOVERY_LEFT_SUPPORT : PUSH_RECOVERY_RIGHT_SUPPORT;
   }

   public static PushRecoveryStateEnum getPushRecoveryTransferState(RobotSide transferToSide)
   {
      return transferToSide == RobotSide.LEFT ? TO_PUSH_RECOVERY_LEFT_SUPPORT : TO_PUSH_RECOVERY_RIGHT_SUPPORT;
   }

   public RobotSide getTransferToSide()
   {
      switch (this)
      {
      case TO_PUSH_RECOVERY_LEFT_SUPPORT:
         return RobotSide.LEFT;

      case TO_PUSH_RECOVERY_RIGHT_SUPPORT:
         return RobotSide.RIGHT;

      default:
         return null;
      }
   }


   public RobotSide getSupportSide()
   {
      switch (this)
      {
      case PUSH_RECOVERY_LEFT_SUPPORT:
         return RobotSide.LEFT;

      case PUSH_RECOVERY_RIGHT_SUPPORT:
         return RobotSide.RIGHT;

      default:
         return null;
      }
   }

   public boolean isDoubleSupport()
   {
      switch (this)
      {
      case TO_STANDING:
      case TO_PUSH_RECOVERY_LEFT_SUPPORT:
      case TO_PUSH_RECOVERY_RIGHT_SUPPORT:
         return true;

      case PUSH_RECOVERY_LEFT_SUPPORT:
      case PUSH_RECOVERY_RIGHT_SUPPORT:
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
