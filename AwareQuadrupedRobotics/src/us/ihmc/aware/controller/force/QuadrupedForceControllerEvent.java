package us.ihmc.aware.controller.force;

public enum QuadrupedForceControllerEvent
{
   JOINTS_INITIALIZED,
   STARTING_POSE_REACHED,
   FINAL_STEP_COMPLETED,

   REQUEST_STAND_PREP,
   REQUEST_STAND,
   REQUEST_STEP,
   REQUEST_PACE,
   REQUEST_AMBLE,
   REQUEST_TROT
}
