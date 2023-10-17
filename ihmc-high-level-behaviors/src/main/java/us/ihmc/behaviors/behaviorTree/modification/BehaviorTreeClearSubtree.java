package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;

public class BehaviorTreeClearSubtree implements BehaviorTreeModification
{
   private final BehaviorTreeNodeState nodeToClearChildrenOf;

   public BehaviorTreeClearSubtree(BehaviorTreeNodeState nodeToClearChildrenOf)
   {
      this.nodeToClearChildrenOf = nodeToClearChildrenOf;
   }

   @Override
   public void performOperation()
   {
      clearChildren(nodeToClearChildrenOf);
   }

   private void clearChildren(BehaviorTreeNodeState localNode)
   {
      for (BehaviorTreeNodeState child : localNode.getChildren())
      {
         clearChildren(child);
      }

      localNode.getChildren().clear();
   }
}

