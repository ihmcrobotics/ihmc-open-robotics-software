package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeState;

public class BehaviorTreeNodeRemoval implements BehaviorTreeModification
{
   private final BehaviorTreeNodeState nodeToRemove;
   private final BehaviorTreeState behaviorTreeState;

   public BehaviorTreeNodeRemoval(BehaviorTreeNodeState nodeToRemove, BehaviorTreeState behaviorTreeState)
   {
      this.nodeToRemove = nodeToRemove;
      this.behaviorTreeState = behaviorTreeState;
   }

   @Override
   public void performOperation()
   {
      findAndRemove(behaviorTreeState.getRootNode());
   }

   private void findAndRemove(BehaviorTreeNodeState node)
   {
      if (node.getChildren().remove(nodeToRemove))
      {
         node.freezeFromModification();
      }
      else
      {
         for (BehaviorTreeNodeState child : node.getChildren())
         {
            findAndRemove(child);
         }
      }
   }
}
