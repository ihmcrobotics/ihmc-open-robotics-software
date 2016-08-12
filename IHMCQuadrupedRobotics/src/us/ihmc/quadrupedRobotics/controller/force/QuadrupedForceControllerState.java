package us.ihmc.quadrupedRobotics.controller.force;

public enum QuadrupedForceControllerState
{
   JOINT_INITIALIZATION,
   DO_NOTHING,
   STAND_PREP,
   STAND_READY,
   FREEZE,
   STAND,
   STEP,
   XGAIT,
   FALL,
   SOLE_WAYPOINT;
   
   public static QuadrupedForceControllerState[] values = values();
}