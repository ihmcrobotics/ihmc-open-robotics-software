package us.ihmc.behaviors.behaviorTree.topology;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeHighLayer;

/**
 * Static topological behavior tree operations to keep the logic in one place.
 * The {@code *Modify} methods mark any node who's children gets changed's children field as modified
 * for the purpose of CRDT synchronization.
 */
public class BehaviorTreeTopologyOperations
{
   static <HLT extends BehaviorTreeNodeHighLayer<HLT, ?, ?>> void destroySubtreeModify(HLT subtreeRoot)
   {
      // Avoiding concurrent modifications
      while (!subtreeRoot.getChildren().isEmpty())
         destroySubtreeModify(subtreeRoot.getChildren().get(0));

      detachChildModify(subtreeRoot);
      subtreeRoot.destroy();
   }

   static <HLT extends BehaviorTreeNodeHighLayer<HLT, ?, ?>> void moveChildModify(HLT toParent, HLT child, int insertionIndex)
   {
      detachChildModify(child);
      insertChildModify(toParent, child, insertionIndex);
   }

   static <HLT extends BehaviorTreeNodeHighLayer<HLT, ?, ?>> void appendChildModify(HLT parent, HLT child)
   {
      appendChildAbstract(parent, child);
      appendChildAbstract(parent.getState(), child.getState());
      appendChildAbstract(parent.getDefinition(), child.getDefinition());
      parent.getDefinition().getChildrenModification().modify();
   }

   static <HLT extends BehaviorTreeNodeHighLayer<HLT, ?, ?>> void insertChildModify(HLT parent, HLT child, int insertionIndex)
   {
      insertChildAbstract(parent, child, insertionIndex);
      insertChildAbstract(parent.getState(), child.getState(), insertionIndex);
      insertChildAbstract(parent.getDefinition(), child.getDefinition(), insertionIndex);
      parent.getDefinition().getChildrenModification().modify();
   }

   static <HLT extends BehaviorTreeNodeHighLayer<HLT, ?, ?>> void detachChildModify(HLT child)
   {
      HLT parent = child.getParent();
      if (parent != null)
         parent.getDefinition().getChildrenModification().modify();

      detachChildAbstract(child);
      detachChildAbstract(child.getState());
      detachChildAbstract(child.getDefinition());
   }

   static <HLT extends BehaviorTreeNodeHighLayer<HLT, ?, ?>> void clearImmediateChildren(HLT parent)
   {
      clearImmediateChildrenAbstract(parent);
      clearImmediateChildrenAbstract(parent.getState());
      clearImmediateChildrenAbstract(parent.getDefinition());
   }

   static <HLT extends BehaviorTreeNodeHighLayer<HLT, ?, ?>> void appendChild(HLT parent, HLT child)
   {
      appendChildAbstract(parent, child);
      appendChildAbstract(parent.getState(), child.getState());
      appendChildAbstract(parent.getDefinition(), child.getDefinition());
   }

   // PRIVATE ABSTRACT OPERATIONS

   private static <LT extends BehaviorTreeNode<LT>> void appendChildAbstract(LT parent, LT child)
   {
      insertChildAbstract(parent, child, parent.getChildren().size());
   }

   private static <LT extends BehaviorTreeNode<LT>> void detachChildAbstract(LT child)
   {
      LT parent = child.getParent();
      if (parent != null)
         parent.getChildren().remove(child);
      child.setParent(null);
   }

   private static <LT extends BehaviorTreeNode<LT>> void insertChildAbstract(LT parent, LT child, int insertionIndex)
   {
      child.setParent(parent);
      parent.getChildren().add(insertionIndex, child);
   }

   private static <LT extends BehaviorTreeNode<LT>> void clearImmediateChildrenAbstract(LT parent)
   {
      for (LT child : parent.getChildren())
         child.setParent(null);

      parent.getChildren().clear();
   }
}
