package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;

import java.util.Set;

public class RDXSceneNodeBasics
{
   private final SceneNode sceneNode;
   private final RDXReferenceFrameGraphic referenceFrameGraphic;
   private final String nameIDText;

   public RDXSceneNodeBasics(SceneNode sceneNode)
   {
      this.sceneNode = sceneNode;
      referenceFrameGraphic = new RDXReferenceFrameGraphic(0.05, Color.BLUE);
      nameIDText = "%s (ID: %d)".formatted(sceneNode.getName(), sceneNode.getID());
   }

   public void update()
   {
      referenceFrameGraphic.setToReferenceFrame(sceneNode.getNodeFrame());
   }

   public void renderImGuiWidgets()
   {
      ImGui.text(nameIDText);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (sceneLevels.contains(RDXSceneLevel.VIRTUAL))
         referenceFrameGraphic.getRenderables(renderables, pool);
   }
}
