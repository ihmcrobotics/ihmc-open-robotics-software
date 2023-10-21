package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.modification.SceneGraphClearSubtree;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeRemoval;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;

import java.util.Set;

public class RDXSceneNode
{
   private final SceneNode sceneNode;
   private final RDXReferenceFrameGraphic referenceFrameGraphic;
   private final String detailsText;

   public RDXSceneNode(SceneNode sceneNode)
   {
      this.sceneNode = sceneNode;
      referenceFrameGraphic = new RDXReferenceFrameGraphic(0.05, Color.BLUE);
      detailsText = "ID: %d, Type: %s".formatted(sceneNode.getID(), sceneNode.getClass().getSuperclass().getSimpleName());
   }

   public void update(SceneGraphModificationQueue modificationQueue)
   {
      referenceFrameGraphic.setToReferenceFrame(sceneNode.getNodeFrame());
   }

   public void renderImGuiWidgets(SceneGraphModificationQueue modificationQueue, SceneGraph sceneGraph)
   {
      ImGui.text(detailsText);

      if (sceneNode != sceneGraph.getRootNode()) // Don't allow removing root node
      {
         if (ImGui.button("Remove##" + sceneNode.getID()))
         {
            remove(modificationQueue, sceneGraph);
         }
      }
   }

   public void remove(SceneGraphModificationQueue modificationQueue, SceneGraph sceneGraph)
   {
      modificationQueue.accept(new SceneGraphClearSubtree(sceneNode));
      modificationQueue.accept(new SceneGraphNodeRemoval(sceneNode, sceneGraph));
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (sceneLevels.contains(RDXSceneLevel.VIRTUAL))
         referenceFrameGraphic.getRenderables(renderables, pool);
   }

   public SceneNode getSceneNode()
   {
      return sceneNode;
   }
}
