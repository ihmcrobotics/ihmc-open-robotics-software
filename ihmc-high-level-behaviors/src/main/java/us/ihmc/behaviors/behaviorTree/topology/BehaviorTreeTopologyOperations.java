package us.ihmc.behaviors.behaviorTree.topology;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeHighLayer;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.tools.Destroyable;

/**
 * Static topological behavior tree operations to keep the logic in one place.
 * We are intentionally not checking the types in this class, because it gets
 * to complicated to use and doesn't add much value.
 */
public class BehaviorTreeTopologyOperations
{
   public static void detachAndDestroySubtree(BehaviorTreeNodeHighLayer<?, ?, ?> node)
   {
      detachAndDestroySubtreeBasic(node);
      detachAndDestroySubtreeBasic(node.getState());
      detachAndDestroySubtreeBasic(node.getDefinition());
   }

   public static void clearChildren(BehaviorTreeNodeHighLayer<?, ?, ?> node)
   {
      clearChildrenBasic(node);
      clearChildrenBasic(node.getState());
      clearChildrenBasic(node.getDefinition());
   }

   public static <HLT extends BehaviorTreeNodeHighLayer<HLT, ?, ?>> void addAndModify(HLT nodeToAdd, HLT parent)
   {
      insertAndModify(nodeToAdd, parent, parent.getChildren().size());
   }

   public static <HLT extends BehaviorTreeNodeHighLayer<HLT, ?, ?>> void add(HLT nodeToAdd, HLT parent)
   {
      insert(nodeToAdd, parent, parent.getChildren().size());
   }

   public static <HLT extends BehaviorTreeNodeHighLayer<HLT, ?, ?>> void moveAndModify(HLT nodeToAdd, HLT previousParent, HLT nextParent, int insertionIndex)
   {
      removeAndModify(nodeToAdd, previousParent);
      insertAndModify(nodeToAdd, nextParent, insertionIndex);
   }

   public static <HLT extends BehaviorTreeNodeHighLayer<HLT, ?, ?>> void removeAndModify(HLT nodeToRemove, HLT parent)
   {
      removeAndModifyBasic(nodeToRemove, parent);removeAndModifyBasic(nodeToRemove.getState(), parent.getState());
      removeAndModifyBasic(nodeToRemove.getDefinition(), parent.getDefinition());
   }

   public static <HLT extends BehaviorTreeNodeHighLayer<HLT, ?, ?>> void remove(HLT nodeToRemove, HLT parent)
   {
      removeBasic(nodeToRemove, parent);removeBasic(nodeToRemove.getState(), parent.getState());
      removeBasic(nodeToRemove.getDefinition(), parent.getDefinition());
   }

   public static <HLT extends BehaviorTreeNodeHighLayer<HLT, ?, ?>> void insertAndModify(HLT nodeToAdd, HLT parent, int insertionIndex)
   {
      insertChildAndModifyBasic(nodeToAdd, parent, insertionIndex);
      insertChildAndModifyBasic(nodeToAdd.getState(), parent.getState(), insertionIndex);
      insertChildAndModifyBasic(nodeToAdd.getDefinition(), parent.getDefinition(), insertionIndex);
   }

   public static <HLT extends BehaviorTreeNodeHighLayer<HLT, ?, ?>> void insert(HLT nodeToAdd, HLT parent, int insertionIndex)
   {
      insertBasic(nodeToAdd, parent, insertionIndex);
      insertBasic(nodeToAdd.getState(), parent.getState(), insertionIndex);
      insertBasic(nodeToAdd.getDefinition(), parent.getDefinition(), insertionIndex);
   }

   // BASIC OPERATIONS

   public static void detachAndDestroySubtreeBasic(BehaviorTreeNode<?> node)
   {
      BehaviorTreeNode<?> parent = node.getParent();
      if (parent != null)
      {
         parent.getChildren().remove(node);
         attemptModify(parent);
      }
      node.setParent(null);

      clearSubtreeAndDestroyBasic(node);
   }

   public static void clearSubtreeAndDestroyBasic(BehaviorTreeNode<?> node)
   {
      for (BehaviorTreeNode<?> child : node.getChildren())
      {
         clearSubtreeAndDestroyBasic(child);
      }

      clearChildrenBasic(node);
      attemptDestroy(node);
   }

   public static void clearSubtreeBasic(BehaviorTreeNode<?> node)
   {
      for (BehaviorTreeNode<?> child : node.getChildren())
      {
         clearSubtreeBasic(child);
      }

      clearChildrenBasic(node);
   }

   public static <LT extends BehaviorTreeNode<LT>> void addChildAndModifyBasic(LT nodeToAdd, LT parent)
   {
      addChildBasic(nodeToAdd, parent);
      attemptModify(nodeToAdd);
      attemptModify(parent);
   }

   public static <LT extends BehaviorTreeNode<LT>> void insertChildAndModifyBasic(LT nodeToAdd, LT parent, int insertionIndex)
   {
      insertBasic(nodeToAdd, parent, insertionIndex);
      attemptModify(nodeToAdd);
      attemptModify(parent);
   }

   public static <LT extends BehaviorTreeNode<LT>> void removeAndModifyBasic(LT nodeToRemove, LT parent)
   {
      removeBasic(nodeToRemove, parent);
      attemptModify(parent);
   }

   public static <LT extends BehaviorTreeNode<LT>> void addChildBasic(LT nodeToAdd, LT parent)
   {
      insertBasic(nodeToAdd, parent, parent.getChildren().size());
   }

   // FUNDAMENTAL OPERATIONS

   public static void clearChildrenBasic(BehaviorTreeNode<?> node)
   {
      for (BehaviorTreeNode<?> child : node.getChildren())
      {
         child.setParent(null);
      }

      node.getChildren().clear();
   }

   public static <LT extends BehaviorTreeNode<LT>> void removeBasic(LT nodeToRemove, LT parent)
   {
      parent.getChildren().remove(nodeToRemove);
      nodeToRemove.setParent(null);
   }

   public static <LT extends BehaviorTreeNode<LT>> void insertBasic(LT nodeToAdd, LT parent, int insertionIndex)
   {
      parent.getChildren().add(insertionIndex, nodeToAdd);
      nodeToAdd.setParent(parent);
   }

   public static void attemptModify(Object thingToModify)
   {
      if (thingToModify instanceof LatestTimestampModifiable freezable)
         freezable.modify();
   }

   public static void attemptDestroy(Object thingToDestroy)
   {
      if (thingToDestroy instanceof Destroyable destroyable)
         destroyable.destroy();
   }
}
