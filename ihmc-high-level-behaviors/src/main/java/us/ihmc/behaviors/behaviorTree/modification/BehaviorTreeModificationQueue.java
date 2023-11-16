package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExtension;
import us.ihmc.communication.crdt.Freezable;

import java.util.LinkedList;
import java.util.Queue;
import java.util.function.Consumer;

/**
 * This interface just exists to provide a better name to what this is,
 * which gets passed down from BehaviorTree's modifyTree method and serves
 * to queue up tree modifications.
 * We are intentionally not checking the types in this class, because it gets
 * to complicated to use and doesn't add much value.
 */
public class BehaviorTreeModificationQueue
{
   private final Queue<BehaviorTreeModification> modificationQueue = new LinkedList<>();

   public boolean performModifications()
   {
      boolean modified = !modificationQueue.isEmpty();

      while (!modificationQueue.isEmpty())
      {
         BehaviorTreeModification modification = modificationQueue.poll();
         modification.performOperation();
      }

      return modified;
   }

   public <T extends BehaviorTreeNodeExtension<T, ?, ?, ?>> void queueInsertNode(BehaviorTreeNodeInsertionDefinition<T> insertionDefinition)
   {
      if (insertionDefinition.getInsertionType() == BehaviorTreeNodeInsertionType.INSERT_ROOT)
      {
         queueSetAndFreezeRootNode(insertionDefinition.getNodeToInsert(),
                                   insertionDefinition.getRootNodeSetter(),
                                   insertionDefinition.getFreezableRootNodeHolder());
      }
      else
      {
         queueAddAndFreezeNode(insertionDefinition.getNodeToInsert(),
                               insertionDefinition.getParent(),
                               insertionDefinition.getInsertionIndex());
      }
   }

   public <T extends BehaviorTreeNodeExtension<T, ?, ?, ?>> void queueSetRootNode(T node, Consumer<T> setter)
   {
      modificationQueue.add(() -> setter.accept(node));
   }

   public <T extends BehaviorTreeNodeExtension<T, ?, ?, ?>> void queueSetAndFreezeRootNode(T node,
                                                                                           Consumer<T> setter,
                                                                                           Freezable freezableRootHolder)
   {
      modificationQueue.add(() ->
      {
         setter.accept(node);
         freezableRootHolder.freeze();
      });
   }

   public void queueDestroySubtree(BehaviorTreeNodeExtension<?, ?, ?, ?> subtree)
   {
      modificationQueue.add(() -> BehaviorTreeTopologyOperations.detachAndDestroySubtree(subtree));
   }

   public <T extends BehaviorTreeNodeExtension<T, ?, ?, ?>> void queueAddNode(T nodeToAdd, T parent)
   {
      modificationQueue.add(() -> BehaviorTreeTopologyOperations.addChild(nodeToAdd, parent));
   }

   public <T extends BehaviorTreeNodeExtension<T, ?, ?, ?>> void queueAddAndFreezeNode(T nodeToAdd, T parent, int insertionIndex)
   {
      modificationQueue.add(() ->
      {
         BehaviorTreeTopologyOperations.insertAndFreezeChild(nodeToAdd, parent, insertionIndex);
      });
   }

   public <T extends BehaviorTreeNodeExtension<T, ?, ?, ?>> void queueAddAndFreezeNode(T nodeToAdd, T parent)
   {
      modificationQueue.add(() -> BehaviorTreeTopologyOperations.addAndFreezeChild(nodeToAdd, parent));
   }

   public void queueModification(BehaviorTreeModification modification)
   {
      modificationQueue.add(modification);
   }
}
