package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

public enum RejectionReasonToVisualize
{
   ALL,
   NONE,
   STEP_TOO_HIGH_OR_LOW,
   STEP_TOO_FORWARD_AND_DOWN,
   STEP_TOO_WIDE_AND_DOWN,
   STEP_TOO_FAR, 
   STEP_TOO_FAR_AND_HIGH, 
   STEP_TOO_WIDE_AND_HIGH, 
   STEP_NOT_WIDE_ENOUGH,
   STEP_IN_PLACE, 
   NOT_ENOUGH_AREA, 
   COULD_NOT_SNAP, 
   SURFACE_NORMAL_TOO_STEEP_TO_SNAP,
   TOO_MUCH_PENETRATION_AFTER_WIGGLE, 
   STEP_NOT_LONG_ENOUGH, 
   STEP_TOO_WIDE,
   OBSTACLE_BLOCKING_BODY,
   OBSTACLE_HITTING_BODY, 
   AT_CLIFF_BOTTOM;

   public static final RejectionReasonToVisualize[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public boolean equals(BipedalFootstepPlannerNodeRejectionReason rejectionReason)
   {
      if (this == NONE)
         return false;

      switch (this)
      {
      case ALL:
         return true;
      case STEP_TOO_HIGH_OR_LOW:
         return rejectionReason == BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_HIGH_OR_LOW;
      case STEP_TOO_FORWARD_AND_DOWN:
         return rejectionReason == BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_FORWARD_AND_DOWN;
      case STEP_TOO_WIDE_AND_DOWN:
         return rejectionReason == BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_WIDE_AND_DOWN;
      case STEP_TOO_FAR:
         return rejectionReason == BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_FAR;
      case STEP_TOO_FAR_AND_HIGH:
         return rejectionReason == BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_FAR_AND_HIGH;
      case STEP_TOO_WIDE_AND_HIGH:
         return rejectionReason == BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_WIDE_AND_HIGH;
      case STEP_NOT_WIDE_ENOUGH:
         return rejectionReason == BipedalFootstepPlannerNodeRejectionReason.STEP_NOT_WIDE_ENOUGH;
      case STEP_IN_PLACE:
         return rejectionReason == BipedalFootstepPlannerNodeRejectionReason.STEP_IN_PLACE;
      case NOT_ENOUGH_AREA:
         return rejectionReason == BipedalFootstepPlannerNodeRejectionReason.NOT_ENOUGH_AREA;
      case COULD_NOT_SNAP:
         return rejectionReason == BipedalFootstepPlannerNodeRejectionReason.COULD_NOT_SNAP;
      case SURFACE_NORMAL_TOO_STEEP_TO_SNAP:
         return rejectionReason == BipedalFootstepPlannerNodeRejectionReason.SURFACE_NORMAL_TOO_STEEP_TO_SNAP;
      case TOO_MUCH_PENETRATION_AFTER_WIGGLE:
         return rejectionReason == BipedalFootstepPlannerNodeRejectionReason.TOO_MUCH_PENETRATION_AFTER_WIGGLE;
      case STEP_NOT_LONG_ENOUGH:
         return rejectionReason == BipedalFootstepPlannerNodeRejectionReason.STEP_NOT_LONG_ENOUGH;
      case STEP_TOO_WIDE:
         return rejectionReason == BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_WIDE;
      case OBSTACLE_BLOCKING_BODY:
         return rejectionReason == BipedalFootstepPlannerNodeRejectionReason.OBSTACLE_BLOCKING_BODY;
      case OBSTACLE_HITTING_BODY:
         return rejectionReason == BipedalFootstepPlannerNodeRejectionReason.OBSTACLE_HITTING_BODY;
      case AT_CLIFF_BOTTOM:
         return rejectionReason == BipedalFootstepPlannerNodeRejectionReason.AT_CLIFF_BOTTOM;
      default:
         throw new RuntimeException("Case is new and can't be rendered.");
      }
   }
}
