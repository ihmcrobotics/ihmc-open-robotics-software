package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;

public class BehaviorTreeNodeStateRemoval implements BehaviorTreeStateModification
{
   private final BehaviorTreeNodeState nodeToRemove;
   private final BehaviorTreeNodeState rootNode;

   public BehaviorTreeNodeStateRemoval(BehaviorTreeNodeState nodeToRemove, BehaviorTreeNodeState rootNode)
   {
      this.nodeToRemove = nodeToRemove;
      this.rootNode = rootNode;
   }

   @Override
   public void performOperation()
   {
      findAndRemove(rootNode);
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
