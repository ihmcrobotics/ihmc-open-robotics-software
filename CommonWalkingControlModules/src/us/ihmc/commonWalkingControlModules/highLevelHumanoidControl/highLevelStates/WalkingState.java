package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.robotics.robotSide.RobotSide;

public enum WalkingState
{
   LEFT_SUPPORT, RIGHT_SUPPORT, TRANSFER_TO_LEFT_SUPPORT, TRANSFER_TO_RIGHT_SUPPORT, DOUBLE_SUPPORT;

   public static WalkingState getSingleSupportState(RobotSide supportLeg)
   {
      return supportLeg == RobotSide.LEFT ? LEFT_SUPPORT : RIGHT_SUPPORT;
   }

   public static WalkingState getTransferState(RobotSide transferToSide)
   {
      return transferToSide == RobotSide.LEFT ? TRANSFER_TO_LEFT_SUPPORT : TRANSFER_TO_RIGHT_SUPPORT;
   }
}

