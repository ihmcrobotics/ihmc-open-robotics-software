package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExtension;

public class BehaviorTreeNodeExtensionRemoval<T extends BehaviorTreeNodeExtension<T, ?, ?, ?>>
      extends BehaviorTreeNodeRemoval<T>
      implements BehaviorTreeModification
{
   private final BehaviorTreeNodeRemoval extendedNodeRemoval;

   public BehaviorTreeNodeExtensionRemoval(T nodeToRemove, T rootNode)
   {
      super(nodeToRemove, rootNode);

      if (nodeToRemove.getExtendedNode() instanceof BehaviorTreeNodeExtension extendedNodeToRemove
       && rootNode.getExtendedNode() instanceof BehaviorTreeNodeExtension extendedRootNode)
      {
         // This will result in recuresively performing the modification on all extended types
         extendedNodeRemoval = new BehaviorTreeNodeExtensionRemoval<>(extendedNodeToRemove, extendedRootNode);
      }
      else
      {
         extendedNodeRemoval = new BehaviorTreeNodeRemoval<>(nodeToRemove.getExtendedNode(), rootNode.getExtendedNode());
      }
   }

   @Override
   public void performOperation()
   {
      super.performOperation();

      extendedNodeRemoval.performOperation();
   }
}
