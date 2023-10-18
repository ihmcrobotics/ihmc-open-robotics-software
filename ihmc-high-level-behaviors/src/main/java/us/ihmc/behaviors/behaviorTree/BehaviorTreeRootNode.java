package us.ihmc.behaviors.behaviorTree;

/**
 * Used for the root node and nothing else.
 */
public class BehaviorTreeRootNode extends BehaviorTreeNodeState
{
   public BehaviorTreeRootNode()
   {
      super(new BehaviorTreeNodeDefinition());
   }
}
