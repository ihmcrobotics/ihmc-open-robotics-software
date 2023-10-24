package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExtension;

public class BehaviorTreeNodeExtensionMove<T extends BehaviorTreeNodeExtension<T, ?, ?, ?>>
      extends BehaviorTreeNodeMove<T>
      implements BehaviorTreeModification
{
   private final BehaviorTreeNodeMove extendedTypeMove;

   public BehaviorTreeNodeExtensionMove(T nodeToMove, T previousParent, T newParent)
   {
      super(nodeToMove, previousParent, newParent);

      if (nodeToMove.getExtendedNode() instanceof BehaviorTreeNodeExtension extendedNodeToMove
       && previousParent.getExtendedNode() instanceof BehaviorTreeNodeExtension extendedPreviousParent
       && newParent.getExtendedNode() instanceof BehaviorTreeNodeExtension extendedNewParent)
      {
         // This will result in recuresively performing the modification on all extended types
         extendedTypeMove = new BehaviorTreeNodeExtensionMove<>(extendedNodeToMove, extendedPreviousParent, extendedNewParent);
      }
      else
      {
         extendedTypeMove = new BehaviorTreeNodeMove<>(nodeToMove.getExtendedNode(), previousParent.getExtendedNode(), newParent.getExtendedNode());
      }
   }

   @Override
   public void performOperation()
   {
      super.doRemovePart();

      extendedTypeMove.performOperation();

      super.doAddPart();
   }
}
