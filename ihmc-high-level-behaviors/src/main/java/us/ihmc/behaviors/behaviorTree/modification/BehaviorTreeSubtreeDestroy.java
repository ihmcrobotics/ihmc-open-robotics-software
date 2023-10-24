package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExtension;

/**
 * Clearing the subtree and detroying the removed nodes.
 */
public class BehaviorTreeSubtreeDestroy implements BehaviorTreeModification
{
   private final BehaviorTreeNode<?> subtreeToClear;

   public BehaviorTreeSubtreeDestroy(BehaviorTreeNode<?> subtreeToClear)
   {
      this.subtreeToClear = subtreeToClear;
   }

   @Override
   public void performOperation()
   {
      clearChildren(subtreeToClear);
   }

   private void clearChildren(BehaviorTreeNode<?> localNode)
   {
      for (BehaviorTreeNode<?> child : localNode.getChildren())
      {
         clearChildren(child);
      }

      localNode.getChildren().clear();

      if (localNode instanceof BehaviorTreeNodeExtension nodeExtension)
         nodeExtension.destroy();
   }
}

