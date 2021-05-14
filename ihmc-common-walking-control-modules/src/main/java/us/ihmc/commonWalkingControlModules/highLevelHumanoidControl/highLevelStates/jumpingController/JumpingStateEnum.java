package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import us.ihmc.robotics.robotSide.RobotSide;

public enum JumpingStateEnum
{
   STANDING,
   SUPPORT,
   FLIGHT,
   TO_STANDING;


   public boolean isDoubleSupport()
   {
      switch (this)
      {
      case STANDING:
      case TO_STANDING:
         return true;


      default:
         throw new RuntimeException("Unknown " + getClass().getSimpleName() + " value: " + this);
      }
   }

}
