package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;

/**
 * An actionable behavior tree node addition to the tree.
 *
 * In the behavior tree, when node additions are requested, they are queued up
 * and performed later to avoid concurrent modifications of node children in the tree.
 */
public class BehaviorTreeNodeAddition implements BehaviorTreeModification
{
   private final BehaviorTreeNodeState nodeToAdd;
   private final BehaviorTreeNodeState parent;

   public BehaviorTreeNodeAddition(BehaviorTreeNodeState nodeToAdd, BehaviorTreeNodeState parent)
   {
      this.nodeToAdd = nodeToAdd;
      this.parent = parent;
   }

   @Override
   public void performOperation()
   {
      parent.getChildren().add(nodeToAdd);
      parent.freezeFromModification();
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
