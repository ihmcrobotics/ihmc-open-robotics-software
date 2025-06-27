package us.ihmc.perception.sceneGraph.modification;

import us.ihmc.perception.sceneGraph.SceneGraph;
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
   private final SceneGraph sceneGraph;

   public SceneGraphNodeAddition(SceneNode nodeToAdd, SceneNode parent, SceneGraph sceneGraph)
   {
      this.nodeToAdd = nodeToAdd;
      this.parent = parent;
      this.sceneGraph = sceneGraph;
   }

   @Override
   public void performOperation()
   {
      if (nodeToAdd.getName().equals(nodeToAdd.getNonUniqueName()))
         nodeToAdd.createUniqueName(sceneGraph);

      parent.getChildren().add(nodeToAdd);
      ensureParentFramesAreConsistent(nodeToAdd, parent);
      parent.freeze();
      nodeToAdd.freeze();
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
