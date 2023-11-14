package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExtension;

/**
 * Clearing the subtree and detroying the removed nodes.
 */
public class BehaviorTreeExtensionSubtreeDestroy extends BehaviorTreeSubtreeDestroy implements BehaviorTreeModification
{
   private final BehaviorTreeSubtreeDestroy extensionSubtreeDestruction;

   public BehaviorTreeExtensionSubtreeDestroy(BehaviorTreeNodeExtension<?, ?, ?, ?> subtreeToDestroy)
   {
      super(subtreeToDestroy);

      if (subtreeToDestroy.getExtendedNode() instanceof BehaviorTreeNodeExtension extendedSubtreeToDestroy)
      {
         // This will result in recursively performing the modification on all extended types
         extensionSubtreeDestruction = new BehaviorTreeExtensionSubtreeDestroy(extendedSubtreeToDestroy);
      }
      else
      {
         extensionSubtreeDestruction = new BehaviorTreeSubtreeDestroy(subtreeToDestroy.getExtendedNode());
      }
   }

   @Override
   public void performOperation()
   {
      super.performOperation();

      extensionSubtreeDestruction.performOperation();
   }
}

