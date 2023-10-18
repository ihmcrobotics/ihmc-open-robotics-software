package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;

public class BehaviorTreeStateDestroySubtree implements BehaviorTreeStateModification
{
   private final BehaviorTreeNodeState subtreeToDestroy;

   public BehaviorTreeStateDestroySubtree(BehaviorTreeNodeState subtreeToDestroy)
   {
      this.subtreeToDestroy = subtreeToDestroy;
   }

   @Override
   public void performOperation()
   {
      destroyChildren(subtreeToDestroy);
   }

   private void destroyChildren(BehaviorTreeNodeState localNode)
   {
      for (BehaviorTreeNodeState child : localNode.getChildren())
      {
         destroyChildren(child);
      }

      localNode.getChildren().clear();
      localNode.destroy();
   }
}

