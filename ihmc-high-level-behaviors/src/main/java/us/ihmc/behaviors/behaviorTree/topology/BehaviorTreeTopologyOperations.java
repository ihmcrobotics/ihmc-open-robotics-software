package us.ihmc.behaviors.behaviorTree.topology;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;
import us.ihmc.behaviors.behaviorTree.TreeNode;

/**
 * Static topological behavior tree operations to keep the logic in one place.
 * The {@code *Modify} methods are for the purpose of CRDT synchronization.
 * When a user or automated algorithm initiates a modification to the tree topology
 * (i.e. changing the set of children), then we mark the children field as modified
 * through the associated LatestTimestampModifiable.
 * The methods without {@code *Modify} would be exclusively operations as a result
 * of getting in sync with the changes that happened on other networked peers.
 */
public class BehaviorTreeTopologyOperations
{
   static <T extends BehaviorTreeNode<T, ?, ?>> void destroySubtreeModify(T subtreeRoot)
   {
      // Avoiding concurrent modifications
      while (!subtreeRoot.getChildren().isEmpty())
         destroySubtreeModify(subtreeRoot.getChildren().get(0));

      detachChildModify(subtreeRoot);
      subtreeRoot.destroy();
   }

   static <T extends BehaviorTreeNode<T, ?, ?>> void destroySubtree(T subtreeRoot)
   {
      // Avoiding concurrent modifications
      while (!subtreeRoot.getChildren().isEmpty())
         destroySubtree(subtreeRoot.getChildren().get(0));

      detachChild(subtreeRoot);
      subtreeRoot.destroy();
   }

   static <T extends BehaviorTreeNode<T, ?, ?>> void moveChildModify(T toParent, T child, int insertionIndex)
   {
      detachChildModify(child);
      insertChildModify(toParent, child, insertionIndex);
   }

   static <T extends BehaviorTreeNode<T, ?, ?>> void appendChildModify(T parent, T child)
   {
      appendChildAbstract(parent, child);
      appendChildAbstract(parent.getState(), child.getState());
      appendChildAbstract(parent.getDefinition(), child.getDefinition());
      parent.getDefinition().getChildrenModification().modify();
   }

   static <T extends BehaviorTreeNode<T, ?, ?>> void insertChildModify(T parent, T child, int insertionIndex)
   {
      insertChildAbstract(parent, child, insertionIndex);
      insertChildAbstract(parent.getState(), child.getState(), insertionIndex);
      insertChildAbstract(parent.getDefinition(), child.getDefinition(), insertionIndex);
      parent.getDefinition().getChildrenModification().modify();
   }

   static <T extends BehaviorTreeNode<T, ?, ?>> void detachChild(T child)
   {
      detachChildAbstract(child);
      detachChildAbstract(child.getState());
      detachChildAbstract(child.getDefinition());
   }

   static <T extends BehaviorTreeNode<T, ?, ?>> void detachChildModify(T child)
   {
      T parent = child.getParent();
      if (parent != null)
         parent.getDefinition().getChildrenModification().modify();

      detachChildAbstract(child);
      detachChildAbstract(child.getState());
      detachChildAbstract(child.getDefinition());
   }

   static <T extends BehaviorTreeNode<T, ?, ?>> void clearImmediateChildren(T parent)
   {
      clearImmediateChildrenAbstract(parent);
      clearImmediateChildrenAbstract(parent.getState());
      clearImmediateChildrenAbstract(parent.getDefinition());
   }

   static <T extends BehaviorTreeNode<T, ?, ?>> void appendChild(T parent, T child)
   {
      appendChildAbstract(parent, child);
      appendChildAbstract(parent.getState(), child.getState());
      appendChildAbstract(parent.getDefinition(), child.getDefinition());
   }

   // PRIVATE ABSTRACT OPERATIONS

   private static <LT extends TreeNode<LT>> void appendChildAbstract(LT parent, LT child)
   {
      insertChildAbstract(parent, child, parent.getChildren().size());
   }

   private static <LT extends TreeNode<LT>> void detachChildAbstract(LT child)
   {
      LT parent = child.getParent();
      if (parent != null)
         parent.getChildren().remove(child);
      child.setParent(null);
   }

   private static <LT extends TreeNode<LT>> void insertChildAbstract(LT parent, LT child, int insertionIndex)
   {
      child.setParent(parent);
      parent.getChildren().add(insertionIndex, child);
   }

   private static <LT extends TreeNode<LT>> void clearImmediateChildrenAbstract(LT parent)
   {
      for (LT child : parent.getChildren())
         child.setParent(null);

      parent.getChildren().clear();
   }
}
