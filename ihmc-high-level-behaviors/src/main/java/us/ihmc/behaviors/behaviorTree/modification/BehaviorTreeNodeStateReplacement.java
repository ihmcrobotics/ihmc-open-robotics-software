package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;

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
public class BehaviorTreeNodeStateReplacement implements BehaviorTreeStateModification
{
   private final BehaviorTreeNodeState nodeToAdd;
   private final BehaviorTreeNodeState parent;

   public BehaviorTreeNodeStateReplacement(BehaviorTreeNodeState nodeToAdd, BehaviorTreeNodeState parent)
   {
      this.nodeToAdd = nodeToAdd;
      this.parent = parent;
   }

   @Override
   public void performOperation()
   {
      parent.getChildren().add(nodeToAdd);
   }

   protected BehaviorTreeNodeState getNodeToAdd()
   {
      return nodeToAdd;
   }

   protected BehaviorTreeNodeState getParent()
   {
      return parent;
   }
}
