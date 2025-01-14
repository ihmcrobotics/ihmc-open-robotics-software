package us.ihmc.behaviors.behaviorTree.topology;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;

/**
 * @param <HLT> The generic type of this node: RDX or Executor
 */
public class BehaviorTreeNodeInsertionDefinition<HLT extends BehaviorTreeNode<HLT, ?, ?>>
{
   private final BehaviorTreeNodeInsertionType insertionType;
   private HLT nodeToInsert;
   private HLT sibling;
   private HLT parent;
   private int insertionIndex;

   public BehaviorTreeNodeInsertionDefinition(BehaviorTreeNodeInsertionType insertionType, HLT nodeToInsert, HLT relativeNode)
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

   private HLT checkSiblingParent()
   {
      HLT parent = sibling.getParent();
      if (parent == null)
         throw new RuntimeException("Sibling's parent cannot be null.");

      return parent;
   }

   public BehaviorTreeNodeInsertionType getInsertionType()
   {
      return insertionType;
   }

   public HLT getNodeToInsert()
   {
      return nodeToInsert;
   }

   public HLT getSibling()
   {
      return sibling;
   }

   public HLT getParent()
   {
      return parent;
   }

   public int getInsertionIndex()
   {
      return insertionIndex;
   }
}
