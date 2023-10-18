package us.ihmc.rdx.ui.behavior.tree.modification;

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
public class RDXBehaviorTreeNodeReplacementBasics implements RDXBehaviorTreeModification
{
   private final RDXBehaviorTreeNode nodeToAdd;
   private final RDXBehaviorTreeNode parent;

   public RDXBehaviorTreeNodeReplacementBasics(RDXBehaviorTreeNode nodeToAdd, RDXBehaviorTreeNode parent)
   {
      this.nodeToAdd = nodeToAdd;
      this.parent = parent;
   }

   @Override
   public void performOperation()
   {
      parent.getChildren().add(nodeToAdd);
   }

   protected RDXBehaviorTreeNode getNodeToAdd()
   {
      return nodeToAdd;
   }

   protected RDXBehaviorTreeNode getParent()
   {
      return parent;
   }
}
