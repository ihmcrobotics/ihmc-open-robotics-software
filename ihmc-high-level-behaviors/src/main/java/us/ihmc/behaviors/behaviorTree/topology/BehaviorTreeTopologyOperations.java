package us.ihmc.behaviors.behaviorTree.topology;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeLayer;
import us.ihmc.communication.crdt.Freezable;
import us.ihmc.tools.Destroyable;

/**
 * Static topological behavior tree operations to keep the logic in one place.
 * We are intentionally not checking the types in this class, because it gets
 * to complicated to use and doesn't add much value.
 */
public class BehaviorTreeTopologyOperations
{

   public static void detachAndDestroySubtree(BehaviorTreeNodeLayer<?, ?, ?, ?> node)
   {
      detachAndDestroySubtreeBasic(node);
      if (node.isLayerOverState())
         detachAndDestroySubtreeBasic(node.getState());
      detachAndDestroySubtreeBasic(node.getDefinition());
   }

   public static void clearChildren(BehaviorTreeNodeLayer<?, ?, ?, ?> node)
   {
      clearChildrenBasic(node);
      if (node.isLayerOverState())
         clearChildrenBasic(node.getState());
      clearChildrenBasic(node.getDefinition());
   }

   public static <T extends BehaviorTreeNodeLayer<T, ?, ?, ?>> void addAndFreezeChild(T nodeToAdd, T parent)
   {
      insertAndFreezeChild(nodeToAdd, parent, parent.getChildren().size());
   }

   public static <T extends BehaviorTreeNodeLayer<T, ?, ?, ?>> void addChild(T nodeToAdd, T parent)
   {
      insertChild(nodeToAdd, parent, parent.getChildren().size());
   }

   public static <T extends BehaviorTreeNodeLayer<T, ?, ?, ?>> void insertAndFreezeChild(T nodeToAdd, T parent, int insertionIndex)
   {
      insertChildAndFreezeBasic(nodeToAdd, parent, insertionIndex);
      if (nodeToAdd.isLayerOverState())
         insertChildAndFreezeBasic(nodeToAdd.getState(), parent.getState(), insertionIndex);
      insertChildAndFreezeBasic(nodeToAdd.getDefinition(), parent.getDefinition(), insertionIndex);
   }

   public static <T extends BehaviorTreeNodeLayer<T, ?, ?, ?>> void insertChild(T nodeToAdd, T parent, int insertionIndex)
   {
      insertChildBasic(nodeToAdd, parent, insertionIndex);
      if (nodeToAdd.isLayerOverState())
         insertChildBasic(nodeToAdd.getState(), parent.getState(), insertionIndex);
      insertChildBasic(nodeToAdd.getDefinition(), parent.getDefinition(), insertionIndex);
   }

   // PRIVATE BASIC OPERATIONS

   private static void detachAndDestroySubtreeBasic(BehaviorTreeNode<?> node)
   {
      BehaviorTreeNode<?> parent = node.getParent();
      if (parent != null)
      {
         parent.getChildren().remove(node);
         attemptFreeze(parent);
      }
      node.setParent(null);

      clearSubtreeAndDestroyBasic(node);
   }

   private static void clearSubtreeAndDestroyBasic(BehaviorTreeNode<?> node)
   {
      for (BehaviorTreeNode<?> child : node.getChildren())
      {
         clearSubtreeAndDestroyBasic(child);
      }

      clearChildrenBasic(node);
      attemptDestroy(node);
   }

   private static void clearSubtreeBasic(BehaviorTreeNode<?> node)
   {
      for (BehaviorTreeNode<?> child : node.getChildren())
      {
         clearSubtreeBasic(child);
      }

      clearChildrenBasic(node);
   }

   private static <T extends BehaviorTreeNode<T>> void addChildAndFreezeBasic(T nodeToAdd, T parent)
   {
      addChildBasic(nodeToAdd, parent);
      attemptFreeze(parent);
   }

   private static <T extends BehaviorTreeNode<T>> void insertChildAndFreezeBasic(T nodeToAdd, T parent, int insertionIndex)
   {
      insertChildBasic(nodeToAdd, parent, insertionIndex);
      attemptFreeze(parent);
   }

   private static <T extends BehaviorTreeNode<T>> void addChildBasic(T nodeToAdd, T parent)
   {
      insertChildBasic(nodeToAdd, parent, parent.getChildren().size());
   }

   // FUNDAMENTAL OPERATIONS

   private static void clearChildrenBasic(BehaviorTreeNode<?> node)
   {
      for (BehaviorTreeNode<?> child : node.getChildren())
      {
         child.setParent(null);
      }

      node.getChildren().clear();
   }

   private static <T extends BehaviorTreeNode<T>> void insertChildBasic(T nodeToAdd, T parent, int insertionIndex)
   {
      parent.getChildren().add(insertionIndex, nodeToAdd);
      nodeToAdd.setParent(parent);
   }

   private static void attemptFreeze(Object thingToFreeze)
   {
      if (thingToFreeze instanceof Freezable freezable)
         freezable.freeze();
   }

   private static void attemptDestroy(Object thingToDestroy)
   {
      if (thingToDestroy instanceof Destroyable destroyable)
         destroyable.destroy();
   }
}
