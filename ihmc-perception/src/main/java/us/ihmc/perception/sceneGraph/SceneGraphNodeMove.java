package us.ihmc.perception.sceneGraph;

/**
 * A record describing a move of a node from one parent to the other.
 *
 * In the scene graph, when node moves are requested, they are queued up
 * and performed later to avoid concurrent modifications of node children in the tree.
 */
public class SceneGraphNodeMove
{
   private final SceneNode nodeToMove;
   private final SceneNode previousParent;
   private final SceneNode newParent;

   public SceneGraphNodeMove(SceneNode nodeToMove, SceneNode previousParent, SceneNode newParent)
   {
      this.nodeToMove = nodeToMove;
      this.previousParent = previousParent;
      this.newParent = newParent;
   }

   public void performMove()
   {
      previousParent.getChildren().remove(nodeToMove);
      newParent.getChildren().add(nodeToMove);
      nodeToMove.changeParentFrameWithoutMoving(newParent.getNodeFrame());
      nodeToMove.ensureFramesMatchParentsRecursively(newParent.getNodeFrame());
      previousParent.freezeFromModification();
      newParent.freezeFromModification();
   }
}
