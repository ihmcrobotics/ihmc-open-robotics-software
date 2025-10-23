package us.ihmc.behaviors.behaviorTree.topology;

import us.ihmc.behaviors.behaviorTree.BehaviorTree;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNode;
import us.ihmc.behaviors.behaviorTree.TreeNode;

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
//         queueSetRootNodeModify(insertionDefinition.getNodeToInsert()); TODO
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
         T rootNode = (T) behaviorTree.getRootNode(); // FIXME: Unchecked cast; but this is hard af
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
         T rootNode = (T) behaviorTree.getRootNode(); // FIXME: Unchecked cast; but this is hard af
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

   public void queueAppendChild(T parent, T child)
   {
      topologyOperationQueue.add(() -> appendChild(parent, child));
   }

   public void queueOperation(BehaviorTreeTopologyOperation topologyOperation)
   {
      topologyOperationQueue.add(topologyOperation);
   }

   /*
    * Static topological behavior tree operations to keep the logic in one place.
    * The {@code *Modify} methods are for the purpose of CRDT synchronization.
    * When a user or automated algorithm initiates a modification to the tree topology
    * (i.e. changing the set of children), then we mark the children field as modified
    * through the associated LatestTimestampModifiable.
    * The methods without {@code *Modify} would be exclusively operations as a result
    * of getting in sync with the changes that happened on other networked peers.
    */

   private void destroySubtreeModify(T subtreeRoot)
   {
      // Avoiding concurrent modifications
      while (!subtreeRoot.getChildren().isEmpty())
         destroySubtreeModify(subtreeRoot.getChildren().get(0));

      detachChildModify(subtreeRoot);
      subtreeRoot.destroy();
   }

   private void destroySubtree(T subtreeRoot)
   {
      // Avoiding concurrent modifications
      while (!subtreeRoot.getChildren().isEmpty())
         destroySubtree(subtreeRoot.getChildren().get(0));

      detachChild(subtreeRoot);
      subtreeRoot.destroy();
   }

   private void moveChildModify(T toParent, T child, int insertionIndex)
   {
      detachChildModify(child);
      insertChildModify(toParent, child, insertionIndex);
   }

   private void appendChildModify(T parent, T child)
   {
      appendChildAbstract(parent, child);
      appendChildAbstract(parent.getState(), child.getState());
      appendChildAbstract(parent.getDefinition(), child.getDefinition());
      parent.getDefinition().getChildrenModification().modify();
   }

   private void insertChildModify(T parent, T child, int insertionIndex)
   {
      insertChildAbstract(parent, child, insertionIndex);
      insertChildAbstract(parent.getState(), child.getState(), insertionIndex);
      insertChildAbstract(parent.getDefinition(), child.getDefinition(), insertionIndex);
      parent.getDefinition().getChildrenModification().modify();
   }

   private void detachChild(T child)
   {
      detachChildAbstract(child);
      detachChildAbstract(child.getState());
      detachChildAbstract(child.getDefinition());
   }

   private void detachChildModify(T child)
   {
      T parent = child.getParent();
      if (parent != null)
         parent.getDefinition().getChildrenModification().modify();

      detachChildAbstract(child);
      detachChildAbstract(child.getState());
      detachChildAbstract(child.getDefinition());
   }

   private void clearImmediateChildren(T parent)
   {
      clearImmediateChildrenAbstract(parent);
      clearImmediateChildrenAbstract(parent.getState());
      clearImmediateChildrenAbstract(parent.getDefinition());
   }

   private void appendChild(T parent, T child)
   {
      appendChildAbstract(parent, child);
      appendChildAbstract(parent.getState(), child.getState());
      appendChildAbstract(parent.getDefinition(), child.getDefinition());
   }

   // PRIVATE ABSTRACT OPERATIONS

   private <N extends TreeNode<N>> void appendChildAbstract(N parent, N child)
   {
      insertChildAbstract(parent, child, parent.getChildren().size());
   }

   private <N extends TreeNode<N>> void detachChildAbstract(N child)
   {
      N parent = child.getParent();
      if (parent != null)
         parent.getChildren().remove(child);
      child.setParent(null);
   }

   private <N extends TreeNode<N>> void insertChildAbstract(N parent, N child, int insertionIndex)
   {
      child.setParent(parent);
      parent.getChildren().add(insertionIndex, child);
   }

   private <N extends TreeNode<N>> void clearImmediateChildrenAbstract(N parent)
   {
      for (N child : parent.getChildren())
         child.setParent(null);

      parent.getChildren().clear();
   }
}
