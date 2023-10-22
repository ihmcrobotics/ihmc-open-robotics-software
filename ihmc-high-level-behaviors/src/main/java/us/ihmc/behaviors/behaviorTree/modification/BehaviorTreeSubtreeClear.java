package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;

/**
 * Clearing the subtree without detroying, to allow for reassembly.
 */
public class BehaviorTreeSubtreeClear<T extends BehaviorTreeNode> implements BehaviorTreeModification<T>
{
   private final T subtreeToClear;

   public BehaviorTreeSubtreeClear(T subtreeToClear)
   {
      this.subtreeToClear = subtreeToClear;
   }

   @Override
   public void performOperation()
   {
      clearChildren(subtreeToClear);
   }

   private void clearChildren(T localNode)
   {
      for (Object child : localNode.getChildren())
      {
         clearChildren((T) child);
      }

      localNode.getChildren().clear();
   }
}

