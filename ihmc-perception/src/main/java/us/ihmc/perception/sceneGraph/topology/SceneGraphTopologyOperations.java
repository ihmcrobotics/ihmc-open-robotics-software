package us.ihmc.perception.sceneGraph.topology;

import us.ihmc.communication.crdt.Freezable;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.tools.Destroyable;

/**
 * Static topological scene tree operations to keep the logic in one place.
 * We are intentionally not checking the types in this class, because it gets
 * to complicated to use and doesn't add much value.
 */
public class SceneGraphTopologyOperations
{
   public static void addAndFreeze(SceneNode nodeToAdd, SceneNode parent)
   {
      insertChildAndFreeze(nodeToAdd, parent, parent.getChildren().size());
   }

   public static void add(SceneNode nodeToAdd, SceneNode parent)
   {
      insert(nodeToAdd, parent, parent.getChildren().size());
   }

   public static void moveAndFreeze(SceneNode nodeToAdd, SceneNode previousParent, SceneNode nextParent, int insertionIndex)
   {
      removeAndFreeze(nodeToAdd, previousParent);
      insertChildAndFreeze(nodeToAdd, nextParent, insertionIndex);
   }

   // PRIVATE BASIC OPERATIONS

   public static void detachAndDestroySubtree(SceneNode node)
   {
      SceneNode parent = node.getParent();
      if (parent != null)
      {
         parent.getChildren().remove(node);
         attemptFreeze(parent);
      }
      node.setParent(null);

      clearSubtreeAndDestroy(node);
   }

   public static void clearSubtreeAndDestroy(SceneNode node)
   {
      for (SceneNode child : node.getChildren())
      {
         clearSubtreeAndDestroy(child);
      }

      clearChildren(node);
      attemptDestroy(node);
   }

   public static void clearSubtree(SceneNode node)
   {
      for (SceneNode child : node.getChildren())
      {
         clearSubtree(child);
      }

      clearChildren(node);
   }

   public static void addChildAndFreeze(SceneNode nodeToAdd, SceneNode parent)
   {
      addChild(nodeToAdd, parent);
      attemptFreeze(parent);
   }

   public static void insertChildAndFreeze(SceneNode nodeToAdd, SceneNode parent, int insertionIndex)
   {
      insert(nodeToAdd, parent, insertionIndex);
      attemptFreeze(parent);
   }

   public static void removeAndFreeze(SceneNode nodeToRemove, SceneNode parent)
   {
      remove(nodeToRemove, parent);
      attemptFreeze(parent);
   }

   public static void addChild(SceneNode nodeToAdd, SceneNode parent)
   {
      insert(nodeToAdd, parent, parent.getChildren().size());
   }

   // FUNDAMENTAL OPERATIONS

   public static void clearChildren(SceneNode node)
   {
      for (SceneNode child : node.getChildren())
      {
         child.setParent(null);
      }

      node.getChildren().clear();
   }

   public static void remove(SceneNode nodeToRemove, SceneNode parent)
   {
      parent.getChildren().remove(nodeToRemove);
      nodeToRemove.setParent(null);
   }

   public static void insert(SceneNode nodeToAdd, SceneNode parent, int insertionIndex)
   {
      parent.getChildren().add(insertionIndex, nodeToAdd);
      nodeToAdd.setParent(parent);
   }

   public static void attemptFreeze(Object thingToFreeze)
   {
      if (thingToFreeze instanceof Freezable freezable)
         freezable.freeze();
   }

   public static void attemptDestroy(Object thingToDestroy)
   {
      if (thingToDestroy instanceof Destroyable destroyable)
         destroyable.destroy();
   }
}
