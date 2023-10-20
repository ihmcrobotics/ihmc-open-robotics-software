package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;

public class BehaviorTreeExecutorNodeRemoval implements BehaviorTreeExecutorModification
{
   private final BehaviorTreeNodeExecutor nodeToRemove;
   private final BehaviorTreeNodeExecutor rootNode;

   private final BehaviorTreeStateNodeRemoval stateRemoval;

   public BehaviorTreeExecutorNodeRemoval(BehaviorTreeNodeExecutor nodeToRemove, BehaviorTreeNodeExecutor rootNode)
   {
      this.nodeToRemove = nodeToRemove;
      this.rootNode = rootNode;

      stateRemoval = new BehaviorTreeStateNodeRemoval(nodeToRemove.getState(), rootNode.getState());
   }

   @Override
   public void performOperation()
   {
      findAndRemove(rootNode);

      stateRemoval.performOperation();
   }

   private void findAndRemove(BehaviorTreeNodeExecutor node)
   {
      if (!node.getChildren().remove(nodeToRemove))
      {
         for (BehaviorTreeNodeExecutor child : node.getChildren())
         {
            findAndRemove(child);
         }
      }
   }
}
