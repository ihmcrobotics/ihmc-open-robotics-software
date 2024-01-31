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
      ensureParentFramesAreConsistent(nodeToAdd, parent);
      parent.freeze();
   }

   /**
    * This is necessary both to make sure added nodes have consistent frames
    * and also that when nodes are moved in the tree, their children are
    * updated accordingly, which is absolutely necessary.
    */
   private void ensureParentFramesAreConsistent(SceneNode node, SceneNode parent)
   {
      node.ensureParentFrameIsConsistent(parent.getNodeFrame());

      for (SceneNode child : node.getChildren())
      {
         ensureParentFramesAreConsistent(child, node);
      }
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
