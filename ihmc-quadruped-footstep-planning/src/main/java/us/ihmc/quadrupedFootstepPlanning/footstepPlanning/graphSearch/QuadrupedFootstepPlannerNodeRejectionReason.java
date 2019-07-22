package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch;

public enum QuadrupedFootstepPlannerNodeRejectionReason
{
   STEP_TOO_HIGH_OR_LOW,
   STEP_YAWING_TOO_MUCH,
//   STEP_TOO_FORWARD_AND_DOWN,
   STEP_TOO_FAR,
   STEP_TOO_FAR_FORWARD,
   STEP_TOO_FAR_BACKWARD,
   STEP_TOO_FAR_RIGHT,
   STEP_TOO_FAR_LEFT,
   STEP_TOO_FAR_FORWARD_AND_UP,
   STEP_TOO_FAR_BACKWARD_AND_UP,
   STEP_TOO_FAR_FORWARD_AND_DOWN,
   STEP_TOO_FAR_BACKWARD_AND_DOWN,
//   STEP_NOT_WIDE_ENOUGH,
   STEP_IN_PLACE,
   STEP_ON_OTHER_FOOT,
   COULD_NOT_SNAP,
   SURFACE_NORMAL_TOO_STEEP_TO_SNAP,
//   COULD_NOT_WIGGLE_INSIDE,
//   SURFACE_NORMAL_TOO_STEEP_TO_SNAP,
//   TOO_MUCH_PENETRATION_AFTER_WIGGLE,
   OBSTACLE_BLOCKING_STEP,
   OBSTACLE_BLOCKING_BODY,
   AT_CLIFF_BOTTOM,
   AT_CLIFF_TOP;
//   OBSTACLE_HITTING_BODY;

   public static final QuadrupedFootstepPlannerNodeRejectionReason[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static QuadrupedFootstepPlannerNodeRejectionReason fromByte(byte enumAsByte)
   {
      if (enumAsByte == -1)
         return null;
      return values[enumAsByte];
   }
}
