package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;

/**
 * An actionable behavior tree node addition to the tree.
 *
 * In the behavior tree, when node additions are requested, they are queued up
 * and performed later to avoid concurrent modifications of node children in the tree.
 */
public class BehaviorTreeNodeStateAddition extends BehaviorTreeNodeStateReplacement
{
   public BehaviorTreeNodeStateAddition(BehaviorTreeNodeState nodeToAdd, BehaviorTreeNodeState parent)
   {
      super(nodeToAdd, parent);
   }

   @Override
   public void performOperation()
   {
      super.performOperation();
      getParent().freezeFromModification();
   }
}
