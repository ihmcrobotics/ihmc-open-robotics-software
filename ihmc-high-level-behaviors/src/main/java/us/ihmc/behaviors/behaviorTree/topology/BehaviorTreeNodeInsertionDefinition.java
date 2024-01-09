package us.ihmc.behaviors.behaviorTree.topology;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeLayer;
import us.ihmc.communication.crdt.Freezable;

import java.util.function.Consumer;

/**
 * To use this class, use static {@link #build} method.
 */
public class BehaviorTreeNodeInsertionDefinition<T extends BehaviorTreeNodeLayer<T, ?, ?, ?>>
{
   // A lot of these fields may be null depending on the insertion type
   private T nodeToInsert;
   private T sibling;
   private T parent;
   private Freezable freezableRootNodeHolder;
   private Consumer<T> rootNodeSetter;
   private BehaviorTreeNodeInsertionType insertionType;
   private int insertionIndex;

   public static <R extends BehaviorTreeNodeLayer<R, ?, ?, ?>>
   BehaviorTreeNodeInsertionDefinition<R> build(R nodeToInsert,
                                                Freezable freezableRootNodeHolder,
                                                Consumer<R> rootNodeSetter,
                                                R relativeNode,
                                                BehaviorTreeNodeInsertionType insertionType)
   {
      BehaviorTreeNodeInsertionDefinition<R> definition = new BehaviorTreeNodeInsertionDefinition<>();
      switch (insertionType)
      {
         case INSERT_BEFORE ->
         {
            definition.setupInsertBefore(nodeToInsert, relativeNode);
         }
         case INSERT_AFTER ->
         {
            definition.setupInsertAfter(nodeToInsert, relativeNode);
         }
         case INSERT_AS_CHILD ->
         {
            definition.setupInsertChild(nodeToInsert, relativeNode);
         }
         case INSERT_ROOT ->
         {
            definition.setupInsertRoot(nodeToInsert, freezableRootNodeHolder, rootNodeSetter);
         }
      }
      return definition;
   }

   private BehaviorTreeNodeInsertionDefinition()
   {
      // Disallow public construction
   }

   private void setupInsertRoot(T newRoot,
                                Freezable freezableRootNodeHolder,
                                Consumer<T> rootNodeSetter)
   {
      this.nodeToInsert = newRoot;
      this.freezableRootNodeHolder = freezableRootNodeHolder;
      this.rootNodeSetter = rootNodeSetter;

      insertionType = BehaviorTreeNodeInsertionType.INSERT_ROOT;
   }

   private void setupInsertBefore(T nodeToInsert,
                                  T sibling)
   {
      this.nodeToInsert = nodeToInsert;
      this.sibling = sibling;

      parent = checkSiblingParent();
      insertionType = BehaviorTreeNodeInsertionType.INSERT_BEFORE;
      insertionIndex = parent.getChildren().indexOf(sibling);
   }

   private void setupInsertAfter(T nodeToInsert,
                                 T sibling)
   {
      this.nodeToInsert = nodeToInsert;
      this.sibling = sibling;

      parent = checkSiblingParent();
      insertionType = BehaviorTreeNodeInsertionType.INSERT_AFTER;
      insertionIndex = parent.getChildren().indexOf(sibling) + 1;
   }

   private void setupInsertChild(T nodeToInsert,
                                 T parent)
   {
      this.nodeToInsert = nodeToInsert;
      this.parent = parent;

      insertionType = BehaviorTreeNodeInsertionType.INSERT_AS_CHILD;
      insertionIndex = parent.getChildren().size();
   }

   private T checkSiblingParent()
   {
      T parent = sibling.getParent();
      if (parent == null)
         throw new RuntimeException("Sibling's parent cannot be null.");

      return parent;
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

   public Consumer<T> getRootNodeSetter()
   {
      return rootNodeSetter;
   }

   public Freezable getFreezableRootNodeHolder()
   {
      return freezableRootNodeHolder;
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
