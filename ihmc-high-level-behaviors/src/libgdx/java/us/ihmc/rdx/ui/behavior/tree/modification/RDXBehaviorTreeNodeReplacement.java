package us.ihmc.rdx.ui.behavior.tree.modification;

import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeNodeStateReplacement;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;

/**
 * An actionable behavior tree node replacement back into the tree,
 * used primary for network synchronized subscriptions where the tree
 * is disassembled entirely and put back together on every newly
 * recieved message.
 *
 * In this case, you don't want to freeze the parent and you don't
 * need to recursively check children frames, as the tree is rebuilt
 * in depth first order.
 */
public class RDXBehaviorTreeNodeReplacement extends RDXBehaviorTreeNodeReplacementBasics
{
   private final BehaviorTreeNodeStateReplacement stateReplacement;

   public RDXBehaviorTreeNodeReplacement(RDXBehaviorTreeNode nodeToAdd, RDXBehaviorTreeNode parent)
   {
      super(nodeToAdd, parent);

      stateReplacement = new BehaviorTreeNodeStateReplacement(nodeToAdd.getState(), parent.getState());
   }

   @Override
   public void performOperation()
   {
      stateReplacement.performOperation();

      super.performOperation();
   }
}
