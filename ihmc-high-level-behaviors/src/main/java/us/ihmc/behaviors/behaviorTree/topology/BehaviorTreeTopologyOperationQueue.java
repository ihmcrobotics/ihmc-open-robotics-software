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
      topologyOperationQueue.add(() -> BehaviorTreeTopologyOperations.addChild(nodeToAdd, parent));
   }

   public <T extends BehaviorTreeNodeLayer<T, ?, ?, ?>> void queueAddAndFreezeNode(T nodeToAdd, T parent, int insertionIndex)
   {
      topologyOperationQueue.add(() ->
      {
         BehaviorTreeTopologyOperations.insertAndFreezeChild(nodeToAdd, parent, insertionIndex);
      });
   }

   public <T extends BehaviorTreeNodeLayer<T, ?, ?, ?>> void queueAddAndFreezeNode(T nodeToAdd, T parent)
   {
      topologyOperationQueue.add(() -> BehaviorTreeTopologyOperations.addAndFreezeChild(nodeToAdd, parent));
   }

   public void queueOperation(BehaviorTreeTopologyOperation topologyOperation)
   {
      topologyOperationQueue.add(topologyOperation);
   }
}
