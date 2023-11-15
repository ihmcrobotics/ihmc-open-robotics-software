package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;
import us.ihmc.communication.crdt.Freezable;

/**
 * Inserts a node before or after another node as a child at the same level.
 * Freezes the node afterward.
 */
public class BehaviorTreeNodeInsert<T extends BehaviorTreeNode<T>> implements BehaviorTreeModification
{
   private final T nodeToInsert;
   private final T existingNode;
   private final BehaviorTreeNodeInsertionType insertionType;

   public BehaviorTreeNodeInsert(T nodeToInsert, T existingNode, BehaviorTreeNodeInsertionType insertionType)
   {
      this.nodeToInsert = nodeToInsert;
      this.existingNode = existingNode;
      this.insertionType = insertionType;
   }

   @Override
   public void performOperation()
   {
      switch (insertionType)
      {
         case INSERT_BEFORE ->
         {
            int index = existingNode.getParent().getChildren().indexOf(existingNode);
            existingNode.getParent().getChildren().add(index, nodeToInsert);

            if (existingNode.getParent() instanceof Freezable freezable)
               freezable.freeze();
         }
         case INSERT_AFTER ->
         {
            int index = existingNode.getParent().getChildren().indexOf(existingNode);
            existingNode.getParent().getChildren().add(index + 1, nodeToInsert);

            if (existingNode.getParent() instanceof Freezable freezable)
               freezable.freeze();
         }
      }
   }

   protected BehaviorTreeNode<?> getNodeToInsert()
   {
      return nodeToInsert;
   }

   protected BehaviorTreeNode<?> getExistingNode()
   {
      return existingNode;
   }
}
