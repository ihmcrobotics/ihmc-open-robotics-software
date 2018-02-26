package us.ihmc.quadrupedRobotics.controller.force;

public enum QuadrupedForceControllerEnum
{
   JOINT_INITIALIZATION,
   DO_NOTHING,
   STAND_PREP,
   STAND_READY,
   FREEZE,
   STEPPING,
   FALL;

   public static QuadrupedForceControllerEnum[] values = values();
}