package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;

public class BehaviorTreeExecutorNodeReplacement implements BehaviorTreeExecutorModification
{
   private final BehaviorTreeNodeExecutor nodeToAdd;
   private final BehaviorTreeNodeExecutor parent;

   private final BehaviorTreeStateNodeReplacement stateReplacement;

   public BehaviorTreeExecutorNodeReplacement(BehaviorTreeNodeExecutor nodeToAdd, BehaviorTreeNodeExecutor parent)
   {
      this.nodeToAdd = nodeToAdd;
      this.parent = parent;

      stateReplacement = new BehaviorTreeStateNodeReplacement(nodeToAdd.getState(), parent.getState());
   }

   @Override
   public void performOperation()
   {
      stateReplacement.performOperation();

      parent.getChildren().add(nodeToAdd);
   }
}
