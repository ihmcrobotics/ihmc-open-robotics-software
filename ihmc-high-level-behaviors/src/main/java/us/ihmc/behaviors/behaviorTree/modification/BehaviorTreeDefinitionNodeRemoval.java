package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;

public class BehaviorTreeDefinitionNodeRemoval implements BehaviorTreeStateModification
{
   private final BehaviorTreeNodeDefinition nodeToRemove;
   private final BehaviorTreeNodeDefinition rootNode;

   public BehaviorTreeDefinitionNodeRemoval(BehaviorTreeNodeDefinition nodeToRemove, BehaviorTreeNodeDefinition rootNode)
   {
      this.nodeToRemove = nodeToRemove;
      this.rootNode = rootNode;
   }

   @Override
   public void performOperation()
   {
      findAndRemove(rootNode);
   }

   private void findAndRemove(BehaviorTreeNodeDefinition node)
   {
      if (!node.getChildren().remove(nodeToRemove))
      {
         for (BehaviorTreeNodeDefinition child : node.getChildren())
         {
            findAndRemove(child);
         }
      }
   }
}
