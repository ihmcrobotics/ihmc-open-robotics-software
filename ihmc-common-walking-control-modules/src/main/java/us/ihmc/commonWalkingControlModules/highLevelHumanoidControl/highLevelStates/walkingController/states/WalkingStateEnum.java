package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import us.ihmc.robotics.robotSide.RobotSide;

public enum WalkingStateEnum
{
   STANDING,
   TO_STANDING,

   TO_WALKING_LEFT_SUPPORT,
   TO_WALKING_RIGHT_SUPPORT,
   WALKING_LEFT_SUPPORT,
   WALKING_RIGHT_SUPPORT,

   TO_FLAMINGO_LEFT_SUPPORT,
   TO_FLAMINGO_RIGHT_SUPPORT,
   FLAMINGO_LEFT_SUPPORT,
   FLAMINGO_RIGHT_SUPPORT;
   
   public static WalkingStateEnum[] values = values();

   public static WalkingStateEnum getWalkingSingleSupportState(RobotSide supportLeg)
   {
      return supportLeg == RobotSide.LEFT ? WALKING_LEFT_SUPPORT : WALKING_RIGHT_SUPPORT;
   }

   public static WalkingStateEnum getWalkingTransferState(RobotSide transferToSide)
   {
      return transferToSide == RobotSide.LEFT ? TO_WALKING_LEFT_SUPPORT : TO_WALKING_RIGHT_SUPPORT;
   }

   public static WalkingStateEnum getFlamingoSingleSupportState(RobotSide supportLeg)
   {
      return supportLeg == RobotSide.LEFT ? FLAMINGO_LEFT_SUPPORT : FLAMINGO_RIGHT_SUPPORT;
   }

   public static WalkingStateEnum getFlamingoTransferState(RobotSide transferToSide)
   {
      return transferToSide == RobotSide.LEFT ? TO_FLAMINGO_LEFT_SUPPORT : TO_FLAMINGO_RIGHT_SUPPORT;
   }

   public RobotSide getTransferToSide()
   {
      switch (this)
      {
      case TO_WALKING_LEFT_SUPPORT:
      case TO_FLAMINGO_LEFT_SUPPORT:
         return RobotSide.LEFT;

      case TO_WALKING_RIGHT_SUPPORT:
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
      case WALKING_LEFT_SUPPORT:
      case FLAMINGO_LEFT_SUPPORT:
         return RobotSide.LEFT;

      case WALKING_RIGHT_SUPPORT:
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
      case TO_WALKING_LEFT_SUPPORT:
      case TO_FLAMINGO_LEFT_SUPPORT:
      case TO_WALKING_RIGHT_SUPPORT:
      case TO_FLAMINGO_RIGHT_SUPPORT:
         return true;

      case WALKING_LEFT_SUPPORT:
      case FLAMINGO_LEFT_SUPPORT:
      case WALKING_RIGHT_SUPPORT:
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
