package us.ihmc.behaviors.behaviorTree.topology;

import us.ihmc.behaviors.behaviorTree.BehaviorTree;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;

import java.util.LinkedList;
import java.util.Queue;

/**
 * This interface just exists to provide a better name to what this is,
 * which gets passed down from BehaviorTree's modifyTreeTopology method and serves
 * to queue up tree modifications.
 * We are intentionally not checking the types in this class, because it gets
 * to complicated to use and doesn't add much value.
 *
 * @param <T> The generic type of this node: RDX or Executor
 */
public class BehaviorTreeTopologyOperationQueue<T extends BehaviorTreeNode<T, ?, ?>>
{
   private final BehaviorTree<T> behaviorTree;
   private final Queue<BehaviorTreeTopologyOperation> topologyOperationQueue = new LinkedList<>();

   public BehaviorTreeTopologyOperationQueue(BehaviorTree<T> behaviorTree)
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

      if (atLeastOneOperationPerformed && behaviorTree.getRootNode() != null)
         behaviorTree.getRootNode().update(); // Must validate fields after topology changes

      return atLeastOneOperationPerformed;
   }

   public void queueInsertNodeModify(BehaviorTreeNodeInsertionDefinition<T> insertionDefinition)
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

   public void queueSetRootNode(T rootNode)
   {
      topologyOperationQueue.add(() ->
      {
         behaviorTree.setRootNode(rootNode);
      });
   }

   public void queueSetRootNodeModify(T rootNode)
   {
      topologyOperationQueue.add(() ->
      {
         behaviorTree.setRootNode(rootNode);
         behaviorTree.getRootReferenceModification().modify();
      });
   }

   public void queueDestroyEntireTreeModify()
   {
      topologyOperationQueue.add(() ->
      {
         T rootNode = behaviorTree.getRootNode();
         behaviorTree.setRootNode(null);
         behaviorTree.getRootReferenceModification().modify();
         if (rootNode != null)
            BehaviorTreeTopologyOperations.destroySubtreeModify(rootNode);
      });
   }

   public void queueDestroyEntireTree()
   {
      topologyOperationQueue.add(() ->
      {
         T rootNode = behaviorTree.getRootNode();
         behaviorTree.setRootNode(null);
         if (rootNode != null)
            BehaviorTreeTopologyOperations.destroySubtree(rootNode);
      });
   }

   public void queueDestroySubtreeModify(T subtreeRoot)
   {
      topologyOperationQueue.add(() -> BehaviorTreeTopologyOperations.destroySubtreeModify(subtreeRoot));
   }

   public void queueInsertChildModify(T parent, T child, int insertionIndex)
   {
      topologyOperationQueue.add(() ->
      {
         BehaviorTreeTopologyOperations.insertChildModify(parent, child, insertionIndex);
      });
   }

   public void queueMoveChildModify(T fromParent, T toParent, T child, T relativeNode, BehaviorTreeNodeInsertionType insertionType)
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

   public void queueAppendChildModify(T parent, T child)
   {
      topologyOperationQueue.add(() -> BehaviorTreeTopologyOperations.appendChildModify(parent, child));
   }

   public void queueClearImmediateChildren(T node)
   {
      topologyOperationQueue.add(() -> BehaviorTreeTopologyOperations.clearImmediateChildren(node));
   }

   public void queueAppendChild(T parent, T child)
   {
      topologyOperationQueue.add(() -> BehaviorTreeTopologyOperations.appendChild(parent, child));
   }

   public void queueOperation(BehaviorTreeTopologyOperation topologyOperation)
   {
      topologyOperationQueue.add(topologyOperation);
   }
}
