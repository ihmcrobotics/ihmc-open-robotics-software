package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExtension;

public class BehaviorTreeNodeExtensionReplacement<T extends BehaviorTreeNodeExtension<T, E, ?, ?>,
                                                  E extends BehaviorTreeNode<E>>
      extends BehaviorTreeNodeReplacement<T>
      implements BehaviorTreeModification<T>
{
   private final BehaviorTreeNodeReplacement<E> extendedTypeAddition;

   public BehaviorTreeNodeExtensionReplacement(T nodeToAdd, T parent)
   {
      super(nodeToAdd, parent);

      if (nodeToAdd.getExtendedNode() instanceof BehaviorTreeNodeExtension extendedNodeToAdd
       && parent.getExtendedNode() instanceof BehaviorTreeNodeExtension extendedParent)
      {
         // This will result in recuresively performing the modification on all extended types
         extendedTypeAddition = new BehaviorTreeNodeExtensionReplacement<>(extendedNodeToAdd, extendedParent);
      }
      else
      {
         extendedTypeAddition = new BehaviorTreeNodeReplacement<>(nodeToAdd.getExtendedNode(), parent.getExtendedNode());
      }
   }

   @Override
   public void performOperation()
   {
      extendedTypeAddition.performOperation();

      super.performOperation();
   }
}
