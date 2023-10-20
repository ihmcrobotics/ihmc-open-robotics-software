package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;

public class BehaviorTreeDefinitionNodeMove implements BehaviorTreeStateModification
{
   private final BehaviorTreeNodeDefinition nodeToMove;
   private final BehaviorTreeNodeDefinition previousParent;
   private final BehaviorTreeNodeDefinition newParent;

   public BehaviorTreeDefinitionNodeMove(BehaviorTreeNodeDefinition nodeToMove, BehaviorTreeNodeDefinition previousParent, BehaviorTreeNodeDefinition newParent)
   {
      this.nodeToMove = nodeToMove;
      this.previousParent = previousParent;
      this.newParent = newParent;
   }

   @Override
   public void performOperation()
   {
      previousParent.getChildren().remove(nodeToMove);
      newParent.getChildren().add(nodeToMove);
   }
}
