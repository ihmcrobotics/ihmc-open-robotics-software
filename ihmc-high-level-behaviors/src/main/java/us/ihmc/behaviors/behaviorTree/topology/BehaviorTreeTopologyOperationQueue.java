package us.ihmc.behaviors.behaviorTree.topology;

import us.ihmc.behaviors.behaviorTree.BehaviorTree;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeHighLayer;

import java.util.LinkedList;
import java.util.Queue;

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
   private final BehaviorTree<HLT> behaviorTree;
   private final Queue<BehaviorTreeTopologyOperation> topologyOperationQueue = new LinkedList<>();

   public BehaviorTreeTopologyOperationQueue(BehaviorTree<HLT> behaviorTree)
   {
      this.behaviorTree = behaviorTree;
   }

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

   public void queueInsertNodeModify(BehaviorTreeNodeInsertionDefinition<HLT> insertionDefinition)
   {
      if (insertionDefinition.getInsertionType() == BehaviorTreeNodeInsertionType.INSERT_ROOT)
      {
         queueSetRootNodeModify(insertionDefinition.getNodeToInsert());
      }
      else
      {
         queueInsertChildModify(insertionDefinition.getParent(), insertionDefinition.getNodeToInsert(), insertionDefinition.getInsertionIndex());
      }
   }

   public void queueSetRootNodeModify(HLT rootNode)
   {
      topologyOperationQueue.add(() ->
      {
         behaviorTree.setRootNode(rootNode);
         behaviorTree.getRootReferenceModification().modify();
      });
   }

   public void queueDestroySubtreeModify(HLT subtreeRoot)
   {
      topologyOperationQueue.add(() -> BehaviorTreeTopologyOperations.destroySubtreeModify(subtreeRoot));
   }

   public void queueInsertChildModify(HLT parent, HLT child, int insertionIndex)
   {
      topologyOperationQueue.add(() ->
      {
         BehaviorTreeTopologyOperations.insertChildModify(parent, child, insertionIndex);
      });
   }

   public void queueMoveChildModify(HLT fromParent, HLT toParent, HLT child, HLT relativeNode, BehaviorTreeNodeInsertionType insertionType)
   {
      topologyOperationQueue.add(() ->
      {
         int indexOfNodeToMove = fromParent.getChildren().indexOf(child);
         int insertionIndex = toParent.getChildren().size();

         if (insertionType != BehaviorTreeNodeInsertionType.INSERT_AS_CHILD)
         {
            // Start with INSERT_BEFORE
            int indexOfRelativeNode = toParent.getChildren().indexOf(relativeNode);

            insertionIndex = indexOfRelativeNode;

            if (insertionType == BehaviorTreeNodeInsertionType.INSERT_AFTER)
               ++insertionIndex;

            // When node is moved to a different index of the same parent. i.e. reordering
            if (fromParent == toParent && indexOfRelativeNode > indexOfNodeToMove) // Avoid out of bounds after node's been removed
               --insertionIndex;
         }

         BehaviorTreeTopologyOperations.moveChildModify(toParent, child, insertionIndex);
      });
   }

   public void queueAppendChildModify(HLT parent, HLT child)
   {
      topologyOperationQueue.add(() -> BehaviorTreeTopologyOperations.appendChildModify(parent, child));
   }

   public void queueClearImmediateChildren(HLT node)
   {
      topologyOperationQueue.add(() -> BehaviorTreeTopologyOperations.clearImmediateChildren(node));
   }

   public void queueAppendChild(HLT parent, HLT child)
   {
      topologyOperationQueue.add(() -> BehaviorTreeTopologyOperations.appendChild(parent, child));
   }

   public void queueOperation(BehaviorTreeTopologyOperation topologyOperation)
   {
      topologyOperationQueue.add(topologyOperation);
   }
}
