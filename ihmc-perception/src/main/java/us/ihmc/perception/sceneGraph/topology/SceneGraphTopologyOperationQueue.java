package us.ihmc.perception.sceneGraph.topology;

import us.ihmc.perception.sceneGraph.SceneNode;

import java.util.LinkedList;
import java.util.Queue;

/**
 * This interface just exists to provide a better name to what this is,
 * which gets passed down from SceneGraph's modifyTreeTopology method and serves
 * to queue up tree modifications.
 */
public class SceneGraphTopologyOperationQueue
{
   private final Queue<SceneGraphTopologyOperation> topologyOperationQueue = new LinkedList<>();

   public boolean performAllQueuedOperations()
   {
      boolean atLeastOneOperationPerformed = !topologyOperationQueue.isEmpty();

      while (!topologyOperationQueue.isEmpty())
      {
         SceneGraphTopologyOperation topologyOperation = topologyOperationQueue.poll();
         topologyOperation.performOperation();
      }

      return atLeastOneOperationPerformed;
   }

   public void queueDestroySubtree(SceneNode subtree)
   {
      topologyOperationQueue.add(() -> SceneGraphTopologyOperations.detachAndDestroySubtree(subtree));
   }

   public void queueAddNode(SceneNode nodeToAdd, SceneNode parent)
   {
      topologyOperationQueue.add(() -> SceneGraphTopologyOperations.add(nodeToAdd, parent));
   }

   public void queueAddAndFreezeNode(SceneNode nodeToAdd, SceneNode parent, int insertionIndex)
   {
      topologyOperationQueue.add(() ->
      {
         SceneGraphTopologyOperations.insertChildAndFreeze(nodeToAdd, parent, insertionIndex);
      });
   }

   public void queueMoveAndFreezeNode(SceneNode nodeToMove, SceneNode previousParent, SceneNode nextParent, SceneNode relativeNode,
                                      SceneGraphNodeInsertionType insertionType)
   {
      topologyOperationQueue.add(() ->
      {
         int indexOfNodeToMove = previousParent.getChildren().indexOf(nodeToMove);
         int insertionIndex = nextParent.getChildren().size();

         if (insertionType != SceneGraphNodeInsertionType.INSERT_AS_CHILD)
         {
            int indexOfRelativeNode = nextParent.getChildren().indexOf(relativeNode);

            insertionIndex = indexOfRelativeNode;

            if (insertionType == SceneGraphNodeInsertionType.INSERT_AFTER)
               ++insertionIndex;

            if (previousParent == nextParent && indexOfRelativeNode > indexOfNodeToMove) // Avoid out of bounds after node's been removed
               --insertionIndex;
         }

         SceneGraphTopologyOperations.moveAndFreeze(nodeToMove, previousParent, nextParent, insertionIndex);
      });
   }

   public void queueAddAndFreezeNode(SceneNode nodeToAdd, SceneNode parent)
   {
      topologyOperationQueue.add(() -> SceneGraphTopologyOperations.addAndFreeze(nodeToAdd, parent));
   }

   public void queueOperation(SceneGraphTopologyOperation topologyOperation)
   {
      topologyOperationQueue.add(topologyOperation);
   }
}
