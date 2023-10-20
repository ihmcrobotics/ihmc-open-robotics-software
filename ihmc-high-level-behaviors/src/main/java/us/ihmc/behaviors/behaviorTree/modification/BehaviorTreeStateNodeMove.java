package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;

public class BehaviorTreeStateNodeMove implements BehaviorTreeStateModification
{
   private final BehaviorTreeNodeState nodeToMove;
   private final BehaviorTreeNodeState previousParent;
   private final BehaviorTreeNodeState newParent;

   private final BehaviorTreeDefinitionNodeMove definitionMove;

   public BehaviorTreeStateNodeMove(BehaviorTreeNodeState nodeToMove, BehaviorTreeNodeState previousParent, BehaviorTreeNodeState newParent)
   {
      this.nodeToMove = nodeToMove;
      this.previousParent = previousParent;
      this.newParent = newParent;

      definitionMove = new BehaviorTreeDefinitionNodeMove(nodeToMove.getDefinition(), previousParent.getDefinition(), newParent.getDefinition());
   }

   @Override
   public void performOperation()
   {
      previousParent.getChildren().remove(nodeToMove);
      previousParent.freezeFromModification();

      definitionMove.performOperation();

      newParent.getChildren().add(nodeToMove);
      newParent.freezeFromModification();
   }
}
