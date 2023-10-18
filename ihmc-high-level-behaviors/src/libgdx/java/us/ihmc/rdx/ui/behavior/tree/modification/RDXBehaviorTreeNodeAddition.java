package us.ihmc.rdx.ui.behavior.tree.modification;

import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeNodeStateAddition;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;

/**
 * An actionable behavior tree node addition to the tree.
 *
 * In the behavior tree, when node additions are requested, they are queued up
 * and performed later to avoid concurrent modifications of node children in the tree.
 */
public class RDXBehaviorTreeNodeAddition extends RDXBehaviorTreeNodeReplacement
{
   private final BehaviorTreeNodeStateAddition stateAddition;

   public RDXBehaviorTreeNodeAddition(RDXBehaviorTreeNode nodeToAdd, RDXBehaviorTreeNode parent)
   {
      super(nodeToAdd, parent);

      stateAddition = new BehaviorTreeNodeStateAddition(nodeToAdd.getState(), parent.getState());
   }

   @Override
   public void performOperation()
   {
      stateAddition.performOperation();

      super.performOperation();
   }
}
