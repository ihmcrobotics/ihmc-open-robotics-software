package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExtension;
import us.ihmc.communication.crdt.Freezable;

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
      if (subtreeToClear.getParent() != null)
      {
         if (subtreeToClear.getParent() instanceof Freezable parentNode)
            parentNode.freeze();

         subtreeToClear.getParent().getChildren().remove(subtreeToClear);
      }

      clearChildren(subtreeToClear);
   }

   private void clearChildren(BehaviorTreeNode<?> localNode)
   {
      for (BehaviorTreeNode<?> child : localNode.getChildren())
      {
         clearChildren(child);
      }

      BehaviorTreeNodeClear.clearChildren(localNode);

      if (localNode instanceof BehaviorTreeNodeExtension<?, ?, ?, ?> nodeExtension)
         nodeExtension.destroy();
   }
}

