package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExtension;

public class BehaviorTreeNodeExtensionInsert<T extends BehaviorTreeNodeExtension<T, ?, ?, ?>>
      extends BehaviorTreeNodeInsert<T>
      implements BehaviorTreeModification
{
   private final BehaviorTreeNodeInsert extendedTypeInsertion;

   public BehaviorTreeNodeExtensionInsert(T nodeToInsert, T existingNode, BehaviorTreeNodeInsertionType insertionType)
   {
      super(nodeToInsert, existingNode, insertionType);

      if (nodeToInsert.getExtendedNode() instanceof BehaviorTreeNodeExtension extendedNodeToInsert
       && existingNode.getExtendedNode() instanceof BehaviorTreeNodeExtension extendedExistingNode)
      {
         // This will result in recursively performing the modification on all extended types
         extendedTypeInsertion = new BehaviorTreeNodeExtensionInsert(extendedNodeToInsert, extendedExistingNode, insertionType);
      }
      else
      {
         extendedTypeInsertion = new BehaviorTreeNodeInsert(nodeToInsert.getExtendedNode(), existingNode.getExtendedNode(), insertionType);
      }
   }

   @Override
   public void performOperation()
   {
      extendedTypeInsertion.performOperation();

      super.performOperation();
   }
}
