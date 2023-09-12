package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

/**
 * A reference frame graphic in the virtual scene
 * to be used for debugging and know where the frame is.
 */
public class RDXSceneNode
{
   private final SceneNode sceneNode;
   private final List<RDXSceneNode> children = new ArrayList<>();
   private final RDXReferenceFrameGraphic referenceFrameGraphic;

   public RDXSceneNode(SceneNode sceneNode)
   {
      this.sceneNode = sceneNode;

      referenceFrameGraphic = new RDXReferenceFrameGraphic(0.05, Color.BLUE);
   }

   public void update()
   {
      referenceFrameGraphic.setToReferenceFrame(sceneNode.getNodeFrame());
   }

   public void renderImGuiWidgets()
   {

   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (sceneLevels.contains(RDXSceneLevel.VIRTUAL))
         referenceFrameGraphic.getRenderables(renderables, pool);
   }

   public List<RDXSceneNode> getChildren()
   {
      return children;
   }
}
