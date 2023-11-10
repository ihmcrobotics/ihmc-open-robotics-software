package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExtension;

public class BehaviorTreeExtensionSubtreeClear extends BehaviorTreeSubtreeClear implements BehaviorTreeModification
{
   private final BehaviorTreeSubtreeClear extensionSubtreeClear;

   public BehaviorTreeExtensionSubtreeClear(BehaviorTreeNodeExtension<?, ?, ?, ?> subtreeToClear)
   {
      super(subtreeToClear);

      if (subtreeToClear.getExtendedNode() instanceof BehaviorTreeNodeExtension extendedSubtreeToClear)
      {
         // This will result in recursively performing the modification on all extended types
         extensionSubtreeClear = new BehaviorTreeExtensionSubtreeClear(extendedSubtreeToClear);
      }
      else
      {
         extensionSubtreeClear = new BehaviorTreeSubtreeClear(subtreeToClear.getExtendedNode());
      }
   }

   @Override
   public void performOperation()
   {
      super.performOperation();

      extensionSubtreeClear.performOperation();
   }
}

