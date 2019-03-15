package us.ihmc.humanoidBehaviors.patrol;

/**
 * Simple, unsafe, but package private state holder.
 */
public enum PatrolState
{
   STOP,
   GATHER_PERCEPTION_DATA,
   RETRIEVE_FOOTSTEP_PLAN,
   WALK_TO_GOAL_WAYPOINT
}
