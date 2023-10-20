package us.ihmc.rdx.ui.behavior.tree.modification;

import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeStateSubtreeDestruction;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;

public class RDXBehaviorTreeSubtreeDestruction implements RDXBehaviorTreeModification
{
   private final RDXBehaviorTreeNode subtreeToDestroy;

   private final BehaviorTreeStateSubtreeDestruction stateDestroySubtree;

   public RDXBehaviorTreeSubtreeDestruction(RDXBehaviorTreeNode subtreeToDestroy)
   {
      this.subtreeToDestroy = subtreeToDestroy;

      stateDestroySubtree = new BehaviorTreeStateSubtreeDestruction(subtreeToDestroy.getState());
   }

   @Override
   public void performOperation()
   {
      destroyChildren(subtreeToDestroy);

      stateDestroySubtree.performOperation();
   }

   private void destroyChildren(RDXBehaviorTreeNode localNode)
   {
      for (RDXBehaviorTreeNode child : localNode.getChildren())
      {
         destroyChildren(child);
      }

      localNode.getChildren().clear();
      localNode.destroy();
   }
}

