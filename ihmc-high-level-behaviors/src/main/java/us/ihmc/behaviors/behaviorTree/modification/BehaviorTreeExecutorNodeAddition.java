package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;

public class BehaviorTreeExecutorNodeAddition implements BehaviorTreeExecutorModification
{
   private final BehaviorTreeNodeExecutor nodeToAdd;
   private final BehaviorTreeNodeExecutor parent;

   private final BehaviorTreeStateNodeAddition stateAddition;

   public BehaviorTreeExecutorNodeAddition(BehaviorTreeNodeExecutor nodeToAdd, BehaviorTreeNodeExecutor parent)
   {
      this.nodeToAdd = nodeToAdd;
      this.parent = parent;

      stateAddition = new BehaviorTreeStateNodeAddition(nodeToAdd.getState(), parent.getState());
   }

   @Override
   public void performOperation()
   {
      stateAddition.performOperation();

      parent.getChildren().add(nodeToAdd);
   }
}
