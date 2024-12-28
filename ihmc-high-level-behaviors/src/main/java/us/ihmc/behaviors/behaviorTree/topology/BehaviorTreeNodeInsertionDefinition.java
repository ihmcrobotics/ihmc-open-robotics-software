package us.ihmc.behaviors.behaviorTree.topology;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeHighLayer;
import us.ihmc.communication.crdt.LatestTimestampModifiable;

import java.util.function.Consumer;

/**
 * @param <HLT> The generic type of this node high layer: RDX or Executor
 */
public class BehaviorTreeNodeInsertionDefinition<HLT extends BehaviorTreeNodeHighLayer<HLT, ?, ?>>
{
   private final BehaviorTreeNodeInsertionType insertionType;
   private HLT nodeToInsert;
   private HLT sibling;
   private HLT parent;
   private int insertionIndex;
   private LatestTimestampModifiable rootNodeModifiable;
   private Consumer<HLT> rootNodeSetter;

   public BehaviorTreeNodeInsertionDefinition(HLT nodeToInsert,
                                              HLT relativeNode,
                                              Consumer<HLT> rootNodeSetter,
                                              LatestTimestampModifiable rootNodeModifiable,
                                              BehaviorTreeNodeInsertionType insertionType)
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
            this.rootNodeModifiable = rootNodeModifiable;
            this.rootNodeSetter = rootNodeSetter;
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

   public Consumer<HLT> getRootNodeSetter()
   {
      return rootNodeSetter;
   }

   public LatestTimestampModifiable getRootNodeModifiable()
   {
      return rootNodeModifiable;
   }

   public BehaviorTreeNodeInsertionType getInsertionType()
   {
      return insertionType;
   }

   public int getInsertionIndex()
   {
      return insertionIndex;
   }
}
