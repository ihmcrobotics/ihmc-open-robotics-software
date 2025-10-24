package us.ihmc.behaviors.behaviorTree.topology;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;
import us.ihmc.behaviors.behaviorTree.TreeNode;

/**
 * Topological behavior tree operations to keep the logic in one place.
 * The {@code *Modify} methods are for the purpose of CRDT synchronization.
 * When a user or automated algorithm initiates a modification to the tree topology
 * (i.e. changing the set of children), then we mark the children field as modified
 * through the associated LatestTimestampModifiable.
 * The methods without {@code *Modify} would be exclusively operations as a result
 * of getting in sync with the changes that happened on other networked peers.
 */
public class BehaviorTreeTopologyOperations<T extends BehaviorTreeNode<T, ?, ?>>
{
   protected void destroySubtreeModify(T subtreeRoot)
   {
      // Avoiding concurrent modifications
      while (!subtreeRoot.getChildren().isEmpty())
         destroySubtreeModify(subtreeRoot.getChildren().get(0));

      detachChildModify(subtreeRoot);
      subtreeRoot.destroy();
   }

   protected void destroySubtree(T subtreeRoot)
   {
      // Avoiding concurrent modifications
      while (!subtreeRoot.getChildren().isEmpty())
         destroySubtree(subtreeRoot.getChildren().get(0));

      detachChild(subtreeRoot);
      subtreeRoot.destroy();
   }

   protected void moveChildModify(T toParent, T child, int insertionIndex)
   {
      detachChildModify(child);
      insertChildModify(toParent, child, insertionIndex);
   }

   protected void appendChildModify(T parent, T child)
   {
      appendChildAbstract(parent, child);
      appendChildAbstract(parent.getState(), child.getState());
      appendChildAbstract(parent.getDefinition(), child.getDefinition());
      parent.getDefinition().getChildrenModification().modify();
   }

   protected void insertChildModify(T parent, T child, int insertionIndex)
   {
      insertChildAbstract(parent, child, insertionIndex);
      insertChildAbstract(parent.getState(), child.getState(), insertionIndex);
      insertChildAbstract(parent.getDefinition(), child.getDefinition(), insertionIndex);
      parent.getDefinition().getChildrenModification().modify();
   }

   protected void detachChild(T child)
   {
      detachChildAbstract(child);
      detachChildAbstract(child.getState());
      detachChildAbstract(child.getDefinition());
   }

   protected void detachChildModify(T child)
   {
      T parent = child.getParent();
      if (parent != null)
         parent.getDefinition().getChildrenModification().modify();

      detachChildAbstract(child);
      detachChildAbstract(child.getState());
      detachChildAbstract(child.getDefinition());
   }

   protected void clearImmediateChildren(T parent)
   {
      clearImmediateChildrenAbstract(parent);
      clearImmediateChildrenAbstract(parent.getState());
      clearImmediateChildrenAbstract(parent.getDefinition());
   }

   protected void appendChild(T parent, T child)
   {
      appendChildAbstract(parent, child);
      appendChildAbstract(parent.getState(), child.getState());
      appendChildAbstract(parent.getDefinition(), child.getDefinition());
   }

   // PRIVATE ABSTRACT OPERATIONS

   protected <N extends TreeNode<N>> void appendChildAbstract(N parent, N child)
   {
      insertChildAbstract(parent, child, parent.getChildren().size());
   }

   protected <N extends TreeNode<N>> void detachChildAbstract(N child)
   {
      N parent = child.getParent();
      if (parent != null)
         parent.getChildren().remove(child);
      child.setParent(null);
   }

   protected <N extends TreeNode<N>> void insertChildAbstract(N parent, N child, int insertionIndex)
   {
      child.setParent(parent);
      parent.getChildren().add(insertionIndex, child);
   }

   protected <N extends TreeNode<N>> void clearImmediateChildrenAbstract(N parent)
   {
      for (N child : parent.getChildren())
         child.setParent(null);

      parent.getChildren().clear();
   }
}
