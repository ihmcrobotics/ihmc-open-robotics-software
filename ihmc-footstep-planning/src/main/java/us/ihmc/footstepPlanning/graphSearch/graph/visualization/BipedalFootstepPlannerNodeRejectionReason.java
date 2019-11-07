package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

public enum BipedalFootstepPlannerNodeRejectionReason
{
   STEP_TOO_HIGH_OR_LOW, STEP_TOO_FORWARD_AND_DOWN, STEP_TOO_WIDE_AND_DOWN,  STEP_TOO_FAR, STEP_TOO_FAR_AND_HIGH, STEP_TOO_WIDE_AND_HIGH, STEP_NOT_WIDE_ENOUGH,
   STEP_IN_PLACE, NOT_ENOUGH_AREA, COULD_NOT_SNAP, COULD_NOT_WIGGLE_INSIDE, SURFACE_NORMAL_TOO_STEEP_TO_SNAP, TOO_MUCH_PENETRATION_AFTER_WIGGLE, STEP_NOT_LONG_ENOUGH, STEP_TOO_WIDE,
   OBSTACLE_BLOCKING_BODY, OBSTACLE_HITTING_BODY, AT_CLIFF_BOTTOM;

   public static final BipedalFootstepPlannerNodeRejectionReason[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static BipedalFootstepPlannerNodeRejectionReason fromByte(byte enumAsByte)
   {
      if (enumAsByte == -1)
         return null;
      return values[enumAsByte];
   }
}
