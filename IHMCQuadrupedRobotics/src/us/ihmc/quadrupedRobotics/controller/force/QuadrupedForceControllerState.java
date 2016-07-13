package us.ihmc.quadrupedRobotics.controller.force;

public enum QuadrupedForceControllerState
{
   JOINT_INITIALIZATION,
   DO_NOTHING,
   STAND_PREP,
   STAND_READY,
   STAND,
   STEP,
   XGAIT,
   FALL,
   SOLE_WAYPOINT
}