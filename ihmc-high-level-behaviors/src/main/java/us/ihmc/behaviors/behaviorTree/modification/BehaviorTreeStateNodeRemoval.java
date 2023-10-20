package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;

public class BehaviorTreeStateNodeRemoval implements BehaviorTreeStateModification
{
   private final BehaviorTreeNodeState nodeToRemove;
   private final BehaviorTreeNodeState rootNode;

   private final BehaviorTreeDefinitionNodeRemoval definitionNodeRemoval;

   public BehaviorTreeStateNodeRemoval(BehaviorTreeNodeState nodeToRemove, BehaviorTreeNodeState rootNode)
   {
      this.nodeToRemove = nodeToRemove;
      this.rootNode = rootNode;

      definitionNodeRemoval = new BehaviorTreeDefinitionNodeRemoval(nodeToRemove.getDefinition(), rootNode.getDefinition());
   }

   @Override
   public void performOperation()
   {
      findAndRemove(rootNode);

      definitionNodeRemoval.performOperation();
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
