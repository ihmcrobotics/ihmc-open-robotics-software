package us.ihmc.quadrupedRobotics.controller.force;

public enum QuadrupedSteppingStateEnum
{
   STAND,
   STEP,
   XGAIT,
   SOLE_WAYPOINT;

   public static QuadrupedSteppingStateEnum[] values = values();
}
