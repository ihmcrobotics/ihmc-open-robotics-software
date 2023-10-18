package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;

public class BehaviorTreeNodeStateMove extends BehaviorTreeNodeStateAddition implements BehaviorTreeStateModification
{
   private final BehaviorTreeNodeState previousParent;

   public BehaviorTreeNodeStateMove(BehaviorTreeNodeState nodeToMove, BehaviorTreeNodeState previousParent, BehaviorTreeNodeState newParent)
   {
      super(nodeToMove, newParent);
      this.previousParent = previousParent;
   }

   @Override
   public void performOperation()
   {
      doRemovePart();
      doAddPart();
   }

   public void doRemovePart()
   {
      BehaviorTreeNodeState nodeToMove = getNodeToAdd();
      previousParent.getChildren().remove(nodeToMove);
      previousParent.freezeFromModification();
   }

   public void doAddPart()
   {
      super.performOperation();
   }
}
