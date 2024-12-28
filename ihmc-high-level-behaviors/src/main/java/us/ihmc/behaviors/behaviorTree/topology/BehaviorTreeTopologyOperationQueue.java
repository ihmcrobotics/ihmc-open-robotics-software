package us.ihmc.behaviors.behaviorTree.topology;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeHighLayer;
import us.ihmc.communication.crdt.LatestTimestampModifiable;

import java.util.LinkedList;
import java.util.Queue;
import java.util.function.Consumer;

/**
 * This interface just exists to provide a better name to what this is,
 * which gets passed down from BehaviorTree's modifyTreeTopology method and serves
 * to queue up tree modifications.
 * We are intentionally not checking the types in this class, because it gets
 * to complicated to use and doesn't add much value.
 *
 * @param <HLT> The generic type of this node high layer: RDX or Executor
 */
public class BehaviorTreeTopologyOperationQueue<HLT extends BehaviorTreeNodeHighLayer<HLT, ?, ?>>
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

   public void queueInsertNode(BehaviorTreeNodeInsertionDefinition<HLT> insertionDefinition)
   {
      if (insertionDefinition.getInsertionType() == BehaviorTreeNodeInsertionType.INSERT_ROOT)
      {
         queueSetAndModifyRootNode(insertionDefinition.getNodeToInsert(),
                                   insertionDefinition.getRootNodeSetter(),
                                   insertionDefinition.getRootNodeModifiable());
      }
      else
      {
         queueAddAndModifyNode(insertionDefinition.getNodeToInsert(),
                               insertionDefinition.getParent(),
                               insertionDefinition.getInsertionIndex());
      }
   }

   public void queueSetAndModifyRootNode(HLT node, Consumer<HLT> setter, LatestTimestampModifiable freezableRootHolder)
   {
      topologyOperationQueue.add(() ->
      {
         setter.accept(node);
         if (node != null)
            node.getDefinition().modify();
         freezableRootHolder.modify();
      });
   }

   public void queueClearChildren(BehaviorTreeNodeHighLayer<?, ?, ?> node)
   {
      topologyOperationQueue.add(() -> BehaviorTreeTopologyOperations.clearChildren(node));
   }

   public void queueDestroySubtree(BehaviorTreeNodeHighLayer<?, ?, ?> subtree)
   {
      topologyOperationQueue.add(() -> BehaviorTreeTopologyOperations.detachAndDestroySubtree(subtree));
   }

   public void queueAddNode(HLT nodeToAdd, HLT parent)
   {
      topologyOperationQueue.add(() -> BehaviorTreeTopologyOperations.add(nodeToAdd, parent));
   }

   public void queueAddAndModifyNode(HLT nodeToAdd, HLT parent, int insertionIndex)
   {
      topologyOperationQueue.add(() ->
      {
         BehaviorTreeTopologyOperations.insertAndModify(nodeToAdd, parent, insertionIndex);
      });
   }

   public void queueMoveAndModifyNode(HLT nodeToMove,
                                      HLT previousParent,
                                      HLT nextParent,
                                      HLT relativeNode,
                                      BehaviorTreeNodeInsertionType insertionType)
   {
      topologyOperationQueue.add(() ->
      {
         int indexOfNodeToMove = previousParent.getChildren().indexOf(nodeToMove);
         int insertionIndex = nextParent.getChildren().size();

         if (insertionType != BehaviorTreeNodeInsertionType.INSERT_AS_CHILD)
         {
            int indexOfRelativeNode = nextParent.getChildren().indexOf(relativeNode);

            insertionIndex = indexOfRelativeNode;

            if (insertionType == BehaviorTreeNodeInsertionType.INSERT_AFTER)
               ++insertionIndex;

            if (previousParent == nextParent && indexOfRelativeNode > indexOfNodeToMove) // Avoid out of bounds after node's been removed
               --insertionIndex;
         }

         BehaviorTreeTopologyOperations.moveAndModify(nodeToMove, previousParent, nextParent, insertionIndex);
      });
   }

   public void queueAddAndModifyNode(HLT nodeToAdd, HLT parent)
   {
      topologyOperationQueue.add(() -> BehaviorTreeTopologyOperations.addAndModify(nodeToAdd, parent));
   }

   public void queueOperation(BehaviorTreeTopologyOperation topologyOperation)
   {
      topologyOperationQueue.add(topologyOperation);
   }
}
