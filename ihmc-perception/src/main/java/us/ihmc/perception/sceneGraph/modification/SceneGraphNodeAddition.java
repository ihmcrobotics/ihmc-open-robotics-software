package us.ihmc.perception.sceneGraph.modification;

import us.ihmc.perception.sceneGraph.SceneNode;

/**
 * An actionable scene node addition to the tree.
 *
 * In the scene graph, when node additions are requested, they are queued up
 * and performed later to avoid concurrent modifications of node children in the tree.
 */
public class SceneGraphNodeAddition implements SceneGraphTreeModification
{
   private final SceneNode nodeToAdd;
   private final SceneNode parent;

   public SceneGraphNodeAddition(SceneNode nodeToAdd, SceneNode parent)
   {
      this.nodeToAdd = nodeToAdd;
      this.parent = parent;
   }

   @Override
   public void performOperation()
   {
      parent.getChildren().add(nodeToAdd);
      nodeToAdd.ensureParentFrame(parent.getNodeFrame());
      nodeToAdd.ensureFramesMatchParentsRecursively(parent.getNodeFrame());
      parent.freezeFromModification();
   }

   protected SceneNode getNodeToAdd()
   {
      return nodeToAdd;
   }

   protected SceneNode getParent()
   {
      return parent;
   }
}
