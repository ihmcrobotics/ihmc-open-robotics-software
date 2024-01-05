package us.ihmc.behaviors.behaviorTree.topology;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeLayer;
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
public class BehaviorTreeTopologyOperationQueue
{
   private final Queue<BehaviorTreeTopologyOperation> topologyOperationQueue = new LinkedList<>();

   public boolean performAllQueuedOperations()
   {
      boolean atLeastOneOperationPerformed = !topologyOperationQueue.isEmpty();

      while (!topologyOperationQueue.isEmpty())
      {
         BehaviorTreeTopologyOperation topologyOperation = topologyOperationQueue.poll();
         topologyOperation.performOperation();
      }

      return atLeastOneOperationPerformed;
   }

   public <T extends BehaviorTreeNodeLayer<T, ?, ?, ?>> void queueInsertNode(BehaviorTreeNodeInsertionDefinition<T> insertionDefinition)
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

   public <T extends BehaviorTreeNodeLayer<T, ?, ?, ?>> void queueSetRootNode(T node, Consumer<T> setter)
   {
      topologyOperationQueue.add(() -> setter.accept(node));
   }

   public <T extends BehaviorTreeNodeLayer<T, ?, ?, ?>> void queueSetAndFreezeRootNode(T node,
                                                                                       Consumer<T> setter,
                                                                                       Freezable freezableRootHolder)
   {
      topologyOperationQueue.add(() ->
      {
         setter.accept(node);
         freezableRootHolder.freeze();
      });
   }

   public void queueDestroySubtree(BehaviorTreeNodeLayer<?, ?, ?, ?> subtree)
   {
      topologyOperationQueue.add(() -> BehaviorTreeTopologyOperations.detachAndDestroySubtree(subtree));
   }

   public <T extends BehaviorTreeNodeLayer<T, ?, ?, ?>> void queueAddNode(T nodeToAdd, T parent)
   {
      topologyOperationQueue.add(() -> BehaviorTreeTopologyOperations.add(nodeToAdd, parent));
   }

   public <T extends BehaviorTreeNodeLayer<T, ?, ?, ?>> void queueAddAndFreezeNode(T nodeToAdd, T parent, int insertionIndex)
   {
      topologyOperationQueue.add(() ->
      {
         BehaviorTreeTopologyOperations.insertAndFreeze(nodeToAdd, parent, insertionIndex);
      });
   }

   public <T extends BehaviorTreeNodeLayer<T, ?, ?, ?>> void queueMoveAndFreezeNode(T nodeToMove,
                                                                                    T previousParent,
                                                                                    T nextParent,
                                                                                    T relativeNode,
                                                                                    BehaviorTreeNodeInsertionType insertionType)
   {
      topologyOperationQueue.add(() ->
      {
         int indexOfNodeToMove = previousParent.getChildren().indexOf(nodeToMove);
         int indexOfRelativeNode = nextParent.getChildren().indexOf(relativeNode);

         int insertionIndex = indexOfRelativeNode;

         if (insertionType == BehaviorTreeNodeInsertionType.INSERT_AFTER)
            ++insertionIndex;

         if (previousParent == nextParent && indexOfRelativeNode > indexOfNodeToMove) // Avoid out of bounds after node's been removed
            --insertionIndex;

         BehaviorTreeTopologyOperations.moveAndFreeze(nodeToMove, previousParent, nextParent, insertionIndex);
      });
   }

   public <T extends BehaviorTreeNodeLayer<T, ?, ?, ?>> void queueAddAndFreezeNode(T nodeToAdd, T parent)
   {
      topologyOperationQueue.add(() -> BehaviorTreeTopologyOperations.addAndFreeze(nodeToAdd, parent));
   }

   public void queueOperation(BehaviorTreeTopologyOperation topologyOperation)
   {
      topologyOperationQueue.add(topologyOperation);
   }
}
