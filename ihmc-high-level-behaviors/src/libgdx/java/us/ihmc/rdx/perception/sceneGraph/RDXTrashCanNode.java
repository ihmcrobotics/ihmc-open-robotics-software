package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.rigidBody.trashcan.TrashCanNode;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;

import java.util.Set;

public class RDXTrashCanNode extends RDXDetectableSceneNode
{
   private final RDXReferenceFrameGraphic trashCanFrameGraphic = new RDXReferenceFrameGraphic(0.2, Color.BLUE);
   private final RDXReferenceFrameGraphic handleFrameGraphic = new RDXReferenceFrameGraphic(0.2, Color.RED);

   public RDXTrashCanNode(TrashCanNode trashCanNode)
   {
      super(trashCanNode);

      trashCanFrameGraphic.setToReferenceFrame(trashCanNode.getTrashCanFrame());
      handleFrameGraphic.setToReferenceFrame(trashCanNode.getHandleFrame());
   }

   @Override
   public void update(SceneGraph sceneGraph)
   {
      trashCanFrameGraphic.updateFromLastGivenFrame();
      handleFrameGraphic.updateFromLastGivenFrame();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      super.getRenderables(renderables, pool, sceneLevels);

      if (!trashCanFrameGraphic.getFramePose3D().containsNaN())
         trashCanFrameGraphic.getRenderables(renderables, pool);
      if (!handleFrameGraphic.getFramePose3D().containsNaN())
         handleFrameGraphic.getRenderables(renderables, pool);
   }
}
