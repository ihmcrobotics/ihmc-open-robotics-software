package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExtension;

import java.util.LinkedList;
import java.util.Queue;
import java.util.function.Consumer;

/**
 * This interface just exists to provide a better name to what this is,
 * which gets passed down from BehaviorTree's modifyTree method and serves
 * to queue up tree modifications.
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

   public void queueSetRootNode(BehaviorTreeNodeExtension<?, ?, ?, ?> node, Consumer<BehaviorTreeNodeExtension<?, ?, ?, ?>> setter)
   {
      modificationQueue.add(new BehaviorTreeNodeSetRoot(node, setter));
   }

   public void queueDestroySubtree(BehaviorTreeNodeExtension<?, ?, ?, ?> subtree)
   {
      modificationQueue.add(new BehaviorTreeExtensionSubtreeDestroy(subtree));
   }

   public void queueDestroySubtree(BehaviorTreeNode<?> subtree)
   {
      modificationQueue.add(new BehaviorTreeSubtreeDestroy(subtree));
   }

   public void queueClearSubtree(BehaviorTreeNodeExtension<?, ?, ?, ?> subtree)
   {
      modificationQueue.add(new BehaviorTreeExtensionSubtreeClear(subtree));
   }

   public void queueClearSubtree(BehaviorTreeNode<?> subtree)
   {
      modificationQueue.add(new BehaviorTreeSubtreeClear(subtree));
   }

   public <T extends BehaviorTreeNode<T>> void queueAddNode(T nodeToAdd, T parent)
   {
      modificationQueue.add(new BehaviorTreeNodeAdd<>(nodeToAdd, parent));
   }

   public void queueAddNode(BehaviorTreeNodeExtension<?, ?, ?, ?> nodeToAdd, BehaviorTreeNodeExtension<?, ?, ?, ?> parent)
   {
      modificationQueue.add(new BehaviorTreeNodeExtensionAdd(nodeToAdd, parent));
   }

   public <T extends BehaviorTreeNodeExtension<T, ?, ?, ?>> void queueAddAndFreezeNode(T nodeToAdd, T parent)
   {
      modificationQueue.add(new BehaviorTreeNodeExtensionAddAndFreeze<>(nodeToAdd, parent));
   }

   public <T extends BehaviorTreeNode<T>> void queueAddAndFreezeNode(T nodeToAdd, T parent)
   {
      modificationQueue.add(new BehaviorTreeNodeAddAndFreeze<>(nodeToAdd, parent));
   }

   public void queueClearNode(BehaviorTreeNodeExtension<?, ?, ?, ?> node)
   {
      modificationQueue.add(new BehaviorTreeNodeExtensionClear(node));
   }

   public void queueClearNode(BehaviorTreeNode<?> node)
   {
      modificationQueue.add(new BehaviorTreeNodeClear(node));
   }

   public <T extends BehaviorTreeNodeExtension<T, ?, ?, ?>> void queueRemoveNode(T nodeToRemove, T rootNode)
   {
      modificationQueue.add(new BehaviorTreeNodeExtensionRemove<>(nodeToRemove, rootNode));
   }

   public <T extends BehaviorTreeNode<T>> void queueRemoveNode(T nodeToRemove, T rootNode)
   {
      modificationQueue.add(new BehaviorTreeNodeRemove<>(nodeToRemove, rootNode));
   }

   public <T extends BehaviorTreeNodeExtension<T, ?, ?, ?>> void queueMoveNode(T nodeToMove, T previousParent, T newParent)
   {
      modificationQueue.add(new BehaviorTreeNodeExtensionMove<>(nodeToMove, previousParent, newParent));
   }

   public <T extends BehaviorTreeNode<T>> void queueMoveNode(T nodeToMove, T previousParent, T newParent)
   {
      modificationQueue.add(new BehaviorTreeNodeMove<>(nodeToMove, previousParent, newParent));
   }

   public void queueModification(BehaviorTreeModification modification)
   {
      modificationQueue.add(modification);
   }
}
