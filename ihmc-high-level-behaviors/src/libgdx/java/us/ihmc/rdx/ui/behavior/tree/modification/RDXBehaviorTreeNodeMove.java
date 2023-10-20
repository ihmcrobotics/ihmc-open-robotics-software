package us.ihmc.rdx.ui.behavior.tree.modification;

import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeStateNodeMove;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;

public class RDXBehaviorTreeNodeMove implements RDXBehaviorTreeModification
{
   private final RDXBehaviorTreeNode nodeToMove;
   private final RDXBehaviorTreeNode previousParent;
   private final RDXBehaviorTreeNode newParent;

   private final BehaviorTreeStateNodeMove stateMove;

   public RDXBehaviorTreeNodeMove(RDXBehaviorTreeNode nodeToMove, RDXBehaviorTreeNode previousParent, RDXBehaviorTreeNode newParent)
   {
      this.nodeToMove = nodeToMove;
      this.previousParent = previousParent;
      this.newParent = newParent;

      stateMove = new BehaviorTreeStateNodeMove(nodeToMove.getState(), previousParent.getState(), newParent.getState());
   }

   @Override
   public void performOperation()
   {
      previousParent.getChildren().remove(nodeToMove);

      stateMove.performOperation();

      newParent.getChildren().add(nodeToMove);
   }
}
