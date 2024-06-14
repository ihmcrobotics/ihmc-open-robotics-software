package us.ihmc.behaviors.roomExploration.old;

public enum BuildingExplorationStateName
{
   /** An "idle" behavior with no automated routines */
   TELEOP,

   /** Robot is executing the WalkThroughDoorBehavior */
   WALK_THROUGH_DOOR,

   /** Robot is executing the LookAndStepBehavior */
   LOOK_AND_STEP,

   /** Robot is going either up or down stairs */
   TRAVERSE_STAIRS;

   public static final BuildingExplorationStateName[] values = values();
}
