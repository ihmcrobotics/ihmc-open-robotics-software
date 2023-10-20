package us.ihmc.rdx.ui.behavior.tree.modification;

import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeStateNodeReplacement;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;

/**
 * Just adds and does not freeze.
 */
public class RDXBehaviorTreeNodeReplacement implements RDXBehaviorTreeModification
{
   private final RDXBehaviorTreeNode nodeToAdd;
   private final RDXBehaviorTreeNode parent;

   private final BehaviorTreeStateNodeReplacement stateReplacement;

   public RDXBehaviorTreeNodeReplacement(RDXBehaviorTreeNode nodeToAdd, RDXBehaviorTreeNode parent)
   {
      this.nodeToAdd = nodeToAdd;
      this.parent = parent;

      stateReplacement = new BehaviorTreeStateNodeReplacement(nodeToAdd.getState(), parent.getState());
   }

   @Override
   public void performOperation()
   {
      stateReplacement.performOperation();

      parent.getChildren().add(nodeToAdd);
   }
}
