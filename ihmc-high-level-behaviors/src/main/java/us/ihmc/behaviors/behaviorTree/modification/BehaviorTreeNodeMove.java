package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;

public class BehaviorTreeNodeMove extends BehaviorTreeNodeAddition implements BehaviorTreeModification
{
   private final BehaviorTreeNodeState previousParent;

   public BehaviorTreeNodeMove(BehaviorTreeNodeState nodeToMove, BehaviorTreeNodeState previousParent, BehaviorTreeNodeState newParent)
   {
      super(nodeToMove, newParent);
      this.previousParent = previousParent;
   }

   @Override
   public void performOperation()
   {
      BehaviorTreeNodeState nodeToMove = getNodeToAdd();

      previousParent.getChildren().remove(nodeToMove);
      super.performOperation();
      previousParent.freezeFromModification();
   }
}
