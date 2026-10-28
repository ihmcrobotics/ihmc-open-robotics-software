package us.ihmc.behaviors.behaviorTree.action.actions;

/** How the robot heads while the RL walking goal reacher walks it to the goal */
public enum RLWalkHeadingMode
{
   /** Face the goal while walking, then turn to the goal yaw */
   FACE_GOAL,
   /** Keep the current heading, walking sideways or backward, then turn to the goal yaw at the end */
   KEEP_HEADING,
   /** Turn to face along the line to the goal, walk the line, then turn to the goal yaw */
   TURN_WALK_TURN;

   public static final RLWalkHeadingMode[] values = values();
}
