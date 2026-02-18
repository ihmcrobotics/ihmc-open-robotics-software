package us.ihmc.behaviors.behaviorTree.topology;

import us.ihmc.behaviors.behaviorTree.BehaviorTree;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNode;

import java.util.LinkedList;
import java.util.Queue;

/**
 * This class allows to queue multiple tree modifications and then execute them
 * all at once via performAllQueuedOperations(). This ensures the tree structure
 * remains consistent during complex operations.
 *
 * @param <T> The generic type of this node: RDX or Executor
 */
public class BehaviorTreeTopologyOperationQueue<T extends BehaviorTreeNode<T, ?, ?>> extends BehaviorTreeTopologyOperations<T>
{
   private final BehaviorTree<BehaviorTreeRootNode<T>, T> behaviorTree;
   private final Queue<BehaviorTreeTopologyOperation> topologyOperationQueue = new LinkedList<>();

   public BehaviorTreeTopologyOperationQueue(BehaviorTree<BehaviorTreeRootNode<T>, T> behaviorTree)
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
         queueSetRootNodeModify((BehaviorTreeRootNode<T>) insertionDefinition.getNodeToInsert());
      }
      else
      {
         queueInsertChildModify(insertionDefinition.getParent(), insertionDefinition.getNodeToInsert(), insertionDefinition.getInsertionIndex());
      }
   }

   public void queueSetRootNode(BehaviorTreeRootNode<T> rootNode)
   {
      topologyOperationQueue.add(() ->
      {
         behaviorTree.setRootNode(rootNode);
      });
   }

   public void queueSetRootNodeModify(BehaviorTreeRootNode<T> rootNode)
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
         T rootNode = (T) behaviorTree.getRootNode();
         behaviorTree.setRootNode(null);
         behaviorTree.getRootReferenceModification().modify();
         if (rootNode != null)
            destroySubtreeModify(rootNode);
      });
   }

   public void queueDestroyEntireTree()
   {
      topologyOperationQueue.add(() ->
      {
         T rootNode = (T) behaviorTree.getRootNode();
         behaviorTree.setRootNode(null);
         if (rootNode != null)
            destroySubtree(rootNode);
      });
   }

   public void queueDestroySubtreeModify(T subtreeRoot)
   {
      topologyOperationQueue.add(() -> destroySubtreeModify(subtreeRoot));
   }

   public void queueInsertChildModify(T parent, T child, int insertionIndex)
   {
      topologyOperationQueue.add(() ->
      {
         insertChildModify(parent, child, insertionIndex);
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

         moveChildModify(toParent, child, insertionIndex);
      });
   }

   public void queueAppendChildModify(T parent, T child)
   {
      topologyOperationQueue.add(() -> appendChildModify(parent, child));
   }

   public void queueClearImmediateChildren(T node)
   {
      topologyOperationQueue.add(() -> clearImmediateChildren(node));
   }

   public void queueDetachChildModify(T child)
   {
      topologyOperationQueue.add(() -> detachChildModify(child));
   }

   public void queueAppendChild(T parent, T child)
   {
      topologyOperationQueue.add(() -> appendChild(parent, child));
   }

   public void queueOperation(BehaviorTreeTopologyOperation topologyOperation)
   {
      topologyOperationQueue.add(topologyOperation);
   }
}
