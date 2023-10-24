package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExtension;

public class BehaviorTreeExtensionSubtreeDestruction<T extends BehaviorTreeNodeExtension<T, E, ?, ?>,
                                                     E extends BehaviorTreeNode<E>>
      extends BehaviorTreeSubtreeDestruction<T>
      implements BehaviorTreeModification<T>
{
   private final BehaviorTreeSubtreeDestruction<E> extensionSubtreeDestruction;

   public BehaviorTreeExtensionSubtreeDestruction(T subtreeToDestroy)
   {
      super(subtreeToDestroy);

      if (subtreeToDestroy.getExtendedNode() instanceof BehaviorTreeNodeExtension extendedSubtreeToDestroy)
      {
         // This will result in recuresively performing the modification on all extended types
         extensionSubtreeDestruction = new BehaviorTreeExtensionSubtreeDestruction<>(extendedSubtreeToDestroy);
      }
      else
      {
         extensionSubtreeDestruction = new BehaviorTreeSubtreeDestruction<>(subtreeToDestroy.getExtendedNode());
      }
   }

   @Override
   public void performOperation()
   {
      super.performOperation();

      extensionSubtreeDestruction.performOperation();
   }
}

