package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;

public class BehaviorTreeStateSubtreeDestruction implements BehaviorTreeStateModification
{
   private final BehaviorTreeNodeState subtreeToDestroy;

   private final BehaviorTreeDefinitionSubtreeClear definitionSubtreeClear;

   public BehaviorTreeStateSubtreeDestruction(BehaviorTreeNodeState subtreeToDestroy)
   {
      this.subtreeToDestroy = subtreeToDestroy;

      definitionSubtreeClear = new BehaviorTreeDefinitionSubtreeClear(subtreeToDestroy.getDefinition());
   }

   @Override
   public void performOperation()
   {
      destroyChildren(subtreeToDestroy);

      definitionSubtreeClear.performOperation();
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

