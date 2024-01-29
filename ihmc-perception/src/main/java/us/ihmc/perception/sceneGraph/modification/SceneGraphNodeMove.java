package us.ihmc.perception.sceneGraph.modification;

import us.ihmc.perception.sceneGraph.SceneNode;

/**
 * An actionable move of a node from one parent to the other.
 *
 * In the scene graph, when node moves are requested, they are queued up
 * and performed later to avoid concurrent modifications of node children in the tree.
 */
public class SceneGraphNodeMove extends SceneGraphNodeAddition implements SceneGraphTreeModification
{
   private final SceneNode previousParent;

   public SceneGraphNodeMove(SceneNode nodeToMove, SceneNode previousParent, SceneNode newParent)
   {
      super(nodeToMove, newParent);
      this.previousParent = previousParent;
   }

   @Override
   public void performOperation()
   {
      SceneNode nodeToMove = getNodeToAdd();
      SceneNode newParent = getParent();

      previousParent.getChildren().remove(nodeToMove);
      nodeToMove.changeFrame(newParent.getNodeFrame());
      super.performOperation();
      previousParent.freeze();
   }
}
