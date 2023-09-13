package us.ihmc.perception.sceneGraph;

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
      nodeToMove.ensureFramesMatchParentsRecursively(newParent.getNodeFrame());
   }
}
