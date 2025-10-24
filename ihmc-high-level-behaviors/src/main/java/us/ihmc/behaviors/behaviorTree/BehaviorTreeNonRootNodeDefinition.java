package us.ihmc.behaviors.behaviorTree;

/**
 * The base definition of a non-root behavior tree node.
 * This exists to provide a protected reference to the root node.
 */
public class BehaviorTreeNonRootNodeDefinition extends BehaviorTreeNodeDefinition
{
   protected final BehaviorTreeRootNodeDefinition rootNode;

   public BehaviorTreeNonRootNodeDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode.getCRDTInfo(), rootNode.getSaveFileDirectory());

      this.rootNode = rootNode;
   }
}
