package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExtension;

public class BehaviorTreeExtensionSubtreeDestruction extends BehaviorTreeSubtreeDestruction implements BehaviorTreeModification
{
   private final BehaviorTreeSubtreeDestruction extensionSubtreeDestruction;

   public BehaviorTreeExtensionSubtreeDestruction(BehaviorTreeNodeExtension<?, ?, ?, ?> subtreeToDestroy)
   {
      super(subtreeToDestroy);

      if (subtreeToDestroy.getExtendedNode() instanceof BehaviorTreeNodeExtension extendedSubtreeToDestroy)
      {
         // This will result in recuresively performing the modification on all extended types
         extensionSubtreeDestruction = new BehaviorTreeExtensionSubtreeDestruction(extendedSubtreeToDestroy);
      }
      else
      {
         extensionSubtreeDestruction = new BehaviorTreeSubtreeDestruction(subtreeToDestroy.getExtendedNode());
      }
   }

   @Override
   public void performOperation()
   {
      super.performOperation();

      extensionSubtreeDestruction.performOperation();
   }
}

