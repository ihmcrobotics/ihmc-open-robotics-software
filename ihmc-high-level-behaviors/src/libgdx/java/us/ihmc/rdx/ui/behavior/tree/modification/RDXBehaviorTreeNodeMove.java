package us.ihmc.rdx.ui.behavior.tree.modification;

import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeNodeStateMove;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;

public class RDXBehaviorTreeNodeMove extends RDXBehaviorTreeNodeAdditionBasics
{
   private final BehaviorTreeNodeStateMove stateMove;
   private final RDXBehaviorTreeNode previousParent;

   public RDXBehaviorTreeNodeMove(RDXBehaviorTreeNode nodeToMove, RDXBehaviorTreeNode previousParent, RDXBehaviorTreeNode newParent)
   {
      super(nodeToMove, newParent);
      this.previousParent = previousParent;

      stateMove = new BehaviorTreeNodeStateMove(nodeToMove.getState(), previousParent.getState(), newParent.getState());
   }

   @Override
   public void performOperation()
   {
      // Remove RDX node
      RDXBehaviorTreeNode nodeToMove = getNodeToAdd();
      previousParent.getChildren().remove(nodeToMove);

      stateMove.performOperation();

      // Add RDX node
      super.performOperation();
   }
}
