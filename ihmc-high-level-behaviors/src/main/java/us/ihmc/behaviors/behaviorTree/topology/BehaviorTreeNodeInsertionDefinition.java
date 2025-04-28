package us.ihmc.behaviors.behaviorTree.topology;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;

/**
 * @param <T> The generic type of this node: RDX or Executor
 */
public class BehaviorTreeNodeInsertionDefinition<T extends BehaviorTreeNode<T, ?, ?>>
{
   private final BehaviorTreeNodeInsertionType insertionType;
   private T nodeToInsert;
   private T sibling;
   private T parent;
   private int insertionIndex;

   public BehaviorTreeNodeInsertionDefinition(BehaviorTreeNodeInsertionType insertionType, T nodeToInsert, T relativeNode)
   {
      this.insertionType = insertionType;

      switch (insertionType)
      {
         case INSERT_BEFORE ->
         {
            this.nodeToInsert = nodeToInsert;
            this.sibling = relativeNode;

            parent = checkSiblingParent();
            insertionIndex = parent.getChildren().indexOf(sibling);
         }
         case INSERT_AFTER ->
         {
            this.nodeToInsert = nodeToInsert;
            this.sibling = relativeNode;

            parent = checkSiblingParent();
            insertionIndex = parent.getChildren().indexOf(sibling) + 1;
         }
         case INSERT_AS_CHILD ->
         {
            this.nodeToInsert = nodeToInsert;
            this.parent = relativeNode;

            insertionIndex = parent.getChildren().size();
         }
         case INSERT_ROOT ->
         {
            this.nodeToInsert = nodeToInsert;
         }
      }
   }

   private T checkSiblingParent()
   {
      T parent = sibling.getParent();
      if (parent == null)
         throw new RuntimeException("Sibling's parent cannot be null.");

      return parent;
   }

   public BehaviorTreeNodeInsertionType getInsertionType()
   {
      return insertionType;
   }

   public T getNodeToInsert()
   {
      return nodeToInsert;
   }

   public T getSibling()
   {
      return sibling;
   }

   public T getParent()
   {
      return parent;
   }

   public int getInsertionIndex()
   {
      return insertionIndex;
   }
}
