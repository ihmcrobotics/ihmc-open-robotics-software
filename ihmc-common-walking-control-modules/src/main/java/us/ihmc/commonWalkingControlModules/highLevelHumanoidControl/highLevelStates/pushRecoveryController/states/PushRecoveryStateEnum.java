package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.states;

import us.ihmc.robotics.robotSide.RobotSide;

public enum PushRecoveryStateEnum
{
   STANDING,
   TO_STANDING,

   TO_FLAMINGO_LEFT_SUPPORT,
   TO_FLAMINGO_RIGHT_SUPPORT,
   FLAMINGO_LEFT_SUPPORT,
   FLAMINGO_RIGHT_SUPPORT;
   
   public static PushRecoveryStateEnum[] values = values();

   public static PushRecoveryStateEnum getFlamingoSingleSupportState(RobotSide supportLeg)
   {
      return supportLeg == RobotSide.LEFT ? FLAMINGO_LEFT_SUPPORT : FLAMINGO_RIGHT_SUPPORT;
   }

   public static PushRecoveryStateEnum getFlamingoTransferState(RobotSide transferToSide)
   {
      return transferToSide == RobotSide.LEFT ? TO_FLAMINGO_LEFT_SUPPORT : TO_FLAMINGO_RIGHT_SUPPORT;
   }

   public RobotSide getTransferToSide()
   {
      switch (this)
      {
      case TO_FLAMINGO_LEFT_SUPPORT:
         return RobotSide.LEFT;

      case TO_FLAMINGO_RIGHT_SUPPORT:
         return RobotSide.RIGHT;

      default:
         return null;
      }
   }


   public RobotSide getSupportSide()
   {
      switch (this)
      {
      case FLAMINGO_LEFT_SUPPORT:
         return RobotSide.LEFT;

      case FLAMINGO_RIGHT_SUPPORT:
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
      case TO_FLAMINGO_LEFT_SUPPORT:
      case TO_FLAMINGO_RIGHT_SUPPORT:
         return true;

      case FLAMINGO_LEFT_SUPPORT:
      case FLAMINGO_RIGHT_SUPPORT:
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
