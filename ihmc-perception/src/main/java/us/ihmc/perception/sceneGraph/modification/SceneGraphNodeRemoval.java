package us.ihmc.perception.sceneGraph.modification;

import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;

public class SceneGraphNodeRemoval implements SceneGraphTreeModification
{
   private final SceneNode nodeToRemove;
   private final SceneGraph sceneGraph;

   public SceneGraphNodeRemoval(SceneNode nodeToRemove, SceneGraph sceneGraph)
   {
      this.nodeToRemove = nodeToRemove;
      this.sceneGraph = sceneGraph;
   }

   @Override
   public void performOperation()
   {
      findAndRemove(sceneGraph.getRootNode());
   }

   private void findAndRemove(SceneNode node)
   {
      if (node.getChildren().remove(nodeToRemove))
      {
         node.freeze();
      }
      else
      {
         for (SceneNode child : node.getChildren())
         {
            findAndRemove(child);
         }
      }
   }
}
