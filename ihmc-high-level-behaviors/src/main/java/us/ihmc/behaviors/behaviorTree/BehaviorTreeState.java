package us.ihmc.behaviors.behaviorTree;

import org.apache.commons.lang3.mutable.MutableLong;

/**
 * This is the state related functionality of a behavior tree,
 * which would live on the UI side and the robot side.
 *
 * The root node is going to be a single basic root node with no functionality
 * and it will never be replaced.
 */
public class BehaviorTreeState
{
   private final MutableLong nextID = new MutableLong(0);

   private BehaviorTreeNodeState behaviorTreeNodeState;

   public void update()
   {

   }

   public MutableLong getNextID()
   {
      return nextID;
   }
}
