package us.ihmc.behaviors.behaviorTree;

/**
 * This is the state related functionality of a behavior tree,
 * which would live on the UI side and the robot side.
 *
 * The root node is going to be a single basic root node with no functionality
 * and it will never be replaced.
 */
public class BehaviorTreeState
{
   private final BehaviorTreeRootNode rootNode = new BehaviorTreeRootNode();

   public BehaviorTreeState()
   {

   }
}
