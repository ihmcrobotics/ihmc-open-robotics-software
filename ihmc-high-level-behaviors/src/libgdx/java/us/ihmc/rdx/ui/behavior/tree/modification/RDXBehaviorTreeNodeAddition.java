package us.ihmc.rdx.ui.behavior.tree.modification;

import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeStateNodeAddition;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;

/**
 * An actionable behavior tree node addition to the tree.
 *
 * In the behavior tree, when node additions are requested, they are queued up
 * and performed later to avoid concurrent modifications of node children in the tree.
 */
public class RDXBehaviorTreeNodeAddition implements RDXBehaviorTreeModification
{
   private final RDXBehaviorTreeNode nodeToAdd;
   private final RDXBehaviorTreeNode parent;

   private final BehaviorTreeStateNodeAddition stateAddition;

   public RDXBehaviorTreeNodeAddition(RDXBehaviorTreeNode nodeToAdd, RDXBehaviorTreeNode parent)
   {
      this.nodeToAdd = nodeToAdd;
      this.parent = parent;

      stateAddition = new BehaviorTreeStateNodeAddition(nodeToAdd.getState(), parent.getState());
   }

   @Override
   public void performOperation()
   {
      stateAddition.performOperation();

      parent.getChildren().add(nodeToAdd);
   }
}
