package us.ihmc.perception.sceneGraph.modification;

import us.ihmc.perception.sceneGraph.SceneNode;

/**
 * Used to remove children recursively from a node in the tree.
 */
public class SceneGraphClearSubtree implements SceneGraphTreeModification
{
   private final SceneNode nodeToClearChildrenOf;

   public SceneGraphClearSubtree(SceneNode nodeToClearChildrenOf)
   {
      this.nodeToClearChildrenOf = nodeToClearChildrenOf;
   }

   @Override
   public void performOperation()
   {
      clearChildren(nodeToClearChildrenOf);
   }

   private void clearChildren(SceneNode localNode)
   {
      for (SceneNode child : localNode.getChildren())
      {
         clearChildren(child);
      }

      localNode.getChildren().clear();
   }
}
