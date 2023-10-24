package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;
import us.ihmc.communication.crdt.Freezable;

public class BehaviorTreeNodeMove<T extends BehaviorTreeNode<T>> implements BehaviorTreeModification
{
   private final T nodeToMove;
   private final T previousParent;
   private final T newParent;

   public BehaviorTreeNodeMove(T nodeToMove, T previousParent, T newParent)
   {
      this.nodeToMove = nodeToMove;
      this.previousParent = previousParent;
      this.newParent = newParent;
   }

   @Override
   public void performOperation()
   {
      doRemovePart();
      doAddPart();
   }

   protected void doRemovePart()
   {
      previousParent.getChildren().remove(nodeToMove);
      if (previousParent instanceof Freezable freezableParent)
         freezableParent.freezeFromModification();
   }

   protected void doAddPart()
   {
      newParent.getChildren().add(nodeToMove);
      if (newParent instanceof Freezable freezableParent)
         freezableParent.freezeFromModification();
   }
}
