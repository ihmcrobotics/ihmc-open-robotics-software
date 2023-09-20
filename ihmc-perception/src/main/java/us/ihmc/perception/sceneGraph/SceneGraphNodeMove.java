package us.ihmc.perception.sceneGraph;

/**
 * Used to queue up desired node moves from one parent to another
 * to avoid concurrent modifications of node children in the tree.
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
      previousParent.markModifiedByOperator();
      newParent.markModifiedByOperator();
   }
}
