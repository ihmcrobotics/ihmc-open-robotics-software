package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;

/**
 * Clearing the subtree without detroying, to allow for reassembly.
 */
public class BehaviorTreeSubtreeClear implements BehaviorTreeModification
{
   private final BehaviorTreeNode<?> subtreeToClear;

   public BehaviorTreeSubtreeClear(BehaviorTreeNode<?> subtreeToClear)
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

      BehaviorTreeNodeClear.clearChildren(localNode);
   }
}

