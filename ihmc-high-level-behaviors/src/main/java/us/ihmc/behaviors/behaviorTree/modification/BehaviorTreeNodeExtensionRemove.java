package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExtension;

public class BehaviorTreeNodeExtensionRemove<T extends BehaviorTreeNodeExtension<T, ?, ?, ?>>
      extends BehaviorTreeNodeRemove<T>
      implements BehaviorTreeModification
{
   private final BehaviorTreeNodeRemove extendedNodeRemoval;

   public BehaviorTreeNodeExtensionRemove(T nodeToRemove, T rootNode)
   {
      super(nodeToRemove, rootNode);

      if (nodeToRemove.getExtendedNode() instanceof BehaviorTreeNodeExtension extendedNodeToRemove
       && rootNode.getExtendedNode() instanceof BehaviorTreeNodeExtension extendedRootNode)
      {
         // This will result in recuresively performing the modification on all extended types
         extendedNodeRemoval = new BehaviorTreeNodeExtensionRemove(extendedNodeToRemove, extendedRootNode);
      }
      else
      {
         extendedNodeRemoval = new BehaviorTreeNodeRemove(nodeToRemove.getExtendedNode(), rootNode.getExtendedNode());
      }
   }

   @Override
   public void performOperation()
   {
      super.performOperation();

      extendedNodeRemoval.performOperation();
   }
}
