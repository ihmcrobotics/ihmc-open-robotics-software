package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;

/**
 * Clearing the node's children without detroying, to allow for reassembly.
 */
public class BehaviorTreeNodeClear implements BehaviorTreeModification
{
   private final BehaviorTreeNode<?> nodeToClear;

   public BehaviorTreeNodeClear(BehaviorTreeNode<?> nodeToClear)
   {
      this.nodeToClear = nodeToClear;
   }

   @Override
   public void performOperation()
   {
      clearChildren(nodeToClear);
   }

   public static void clearChildren(BehaviorTreeNode<?> localNode)
   {
      for (BehaviorTreeNode<?> child : localNode.getChildren())
      {
         child.setParent(null);
      }

      localNode.getChildren().clear();
   }
}

