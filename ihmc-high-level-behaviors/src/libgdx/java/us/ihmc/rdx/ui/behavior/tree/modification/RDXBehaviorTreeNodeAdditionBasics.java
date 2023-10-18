package us.ihmc.rdx.ui.behavior.tree.modification;

import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;

/**
 * An actionable behavior tree node addition to the tree.
 *
 * In the behavior tree, when node additions are requested, they are queued up
 * and performed later to avoid concurrent modifications of node children in the tree.
 */
public class RDXBehaviorTreeNodeAdditionBasics extends RDXBehaviorTreeNodeReplacementBasics
{
   public RDXBehaviorTreeNodeAdditionBasics(RDXBehaviorTreeNode nodeToAdd, RDXBehaviorTreeNode parent)
   {
      super(nodeToAdd, parent);
   }

   @Override
   public void performOperation()
   {
      super.performOperation();
   }
}
