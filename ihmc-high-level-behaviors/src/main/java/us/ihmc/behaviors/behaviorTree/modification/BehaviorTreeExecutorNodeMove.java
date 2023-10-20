package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;

public class BehaviorTreeExecutorNodeMove implements BehaviorTreeExecutorModification
{
   private final BehaviorTreeNodeExecutor nodeToMove;
   private final BehaviorTreeNodeExecutor previousParent;
   private final BehaviorTreeNodeExecutor newParent;

   private final BehaviorTreeStateNodeMove stateMove;

   public BehaviorTreeExecutorNodeMove(BehaviorTreeNodeExecutor nodeToMove, BehaviorTreeNodeExecutor previousParent, BehaviorTreeNodeExecutor newParent)
   {
      this.nodeToMove = nodeToMove;
      this.previousParent = previousParent;
      this.newParent = newParent;

      stateMove = new BehaviorTreeStateNodeMove(nodeToMove.getState(), previousParent.getState(), newParent.getState());
   }

   @Override
   public void performOperation()
   {
      previousParent.getChildren().remove(nodeToMove);

      stateMove.performOperation();

      newParent.getChildren().add(nodeToMove);
   }
}
