package us.ihmc.behaviors.behaviorTree;

/**
 * Used for the root node and nothing else.
 */
public class BehaviorTreeRootNode extends BehaviorTreeNodeState
{
   private final BehaviorTreeNodeDefinition definition = new BehaviorTreeNodeDefinition();

   @Override
   public BehaviorTreeNodeDefinition getDefinition()
   {
      return definition;
   }
}
